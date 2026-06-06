"""
State machine for the SO-101 autonomous pick-and-place task.

Pure Python — no ROS2 imports. The task_planner_node drives this object from
a timer callback and translates transitions into ROS2 topic/service calls.

States mirror the v2 app's approach state machine (ARCHITECTURE.md).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Optional


class S(str, Enum):
    IDLE                    = "IDLE"
    SCAN_OBJECTS            = "SCAN_OBJECTS"
    SELECT_OBJECT           = "SELECT_OBJECT"
    PLAN_PICK               = "PLAN_PICK"
    MOVE_TO_PREGRASP        = "MOVE_TO_PREGRASP"
    ALIGN_WITH_GRIPPER_CAM  = "ALIGN_WITH_GRIPPER_CAM"
    GRASP                   = "GRASP"
    VERIFY_GRASP            = "VERIFY_GRASP"
    MOVE_TO_DROP            = "MOVE_TO_DROP"
    PLACE                   = "PLACE"
    VERIFY_PLACE            = "VERIFY_PLACE"
    RETRY_OR_DONE           = "RETRY_OR_DONE"


MOTOR_NAMES = ('base', 'shoulder', 'elbow', 'palm', 'wrist', 'gripper')

# ── Tick constants mirrored from config.py ────────────────────────────────────
DEFAULT_TICKS   = dict(base=2048, shoulder=2048, elbow=2048, palm=2048, wrist=3200, gripper=3000)
GRIPPER_OPEN    = 3000
GRIPPER_CLOSE   = 2100

# Joint limits (tick ranges from config.py)
SH_MIN, SH_MAX     = 1000, 3000
EL_MIN, EL_MAX     =  400, 3000
PALM_MIN, PALM_MAX = 1000, 3500

# IBVS gains and targets mirrored from config.py
K_BASE          = 140
K_SHOULDER      = 450
K_ELBOW         = 65
SHOULDER_DIR    = -1
ELBOW_DIR       = -1
PALM_DIR        = -1
APPROACH_AIM_X  = 3.2 / 4
APPROACH_AIM_Y  = 2.0 / 4
CENTERED_X      = 0.12
CENTERED_Y      = 0.12
ELBOW_CENTERING_GATE = 0.3
VL53_LOCK_DIST_MM    = 130
VL53_GRIP_DIST_MM    = 110
VL53_MAX_APPROACH_MM = 400
VL53_SHOULDER_RATIO  = 0.3
VL53_PREGRASP_PALM_DELTA = -50
PLACE_DIST_MM        = 200
HOME_TOL             = 25


def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


@dataclass
class PlannerState:
    state:          S = S.IDLE
    previous_state: S = S.IDLE
    attempt:        int = 0
    max_retries:    int = 3
    dry_run:        bool = True

    pick_query:     str = ""
    place_query:    str = ""

    # Sensor snapshots (updated by node from subscriptions)
    target_x:       float = 320.0
    target_y:       float = 240.0
    has_target:     bool  = False
    range_mm:       Optional[float] = None
    gripper_load:   float = 0.0
    gripper_current: float = 0.0

    # Joint state (ticks)
    curr:   dict = field(default_factory=lambda: dict(DEFAULT_TICKS))
    target: dict = field(default_factory=lambda: dict(DEFAULT_TICKS))

    # Phase flags
    arm_locked:      bool = False
    pre_grasp_palm:  bool = False
    gripping:        bool = False
    gripper_closed:  bool = False
    returning_home:  bool = False
    place_mode:      bool = False

    grip_frames:     int = 0
    frame_w:         float = 640.0
    frame_h:         float = 480.0

    # For home arrival detection
    home_ticks: dict = field(default_factory=lambda: dict(DEFAULT_TICKS))


class StateMachine:
    """
    Drives PlannerState through the pick-and-place FSM.
    Call tick() at ~25 Hz from the ROS2 node timer.

    The node supplies current sensor data by mutating ps.* fields before
    calling tick(), and reads the joint command from ps.target after each tick.
    """

    def __init__(self, ps: PlannerState, log: Callable[[str], None] = print) -> None:
        self.ps  = ps
        self._log = log

    def transition(self, new_state: S) -> None:
        if new_state == self.ps.state:
            return
        self._log(f'[FSM] {self.ps.state.value} → {new_state.value}  attempt={self.ps.attempt}')
        self.ps.previous_state = self.ps.state
        self.ps.state          = new_state

    # ── public entry points ───────────────────────────────────────────────────

    def start_task(self, pick_query: str, place_query: str) -> bool:
        if self.ps.state != S.IDLE:
            self._log(f'[FSM] start_task rejected — current state is {self.ps.state.value}')
            return False
        self.ps.pick_query  = pick_query
        self.ps.place_query = place_query
        self.ps.attempt     = 0
        self._reset_flags()
        self.transition(S.SCAN_OBJECTS)
        return True

    def abort(self) -> None:
        self._log('[FSM] abort requested')
        self._reset_flags()
        self.transition(S.IDLE)

    # ── per-frame tick ────────────────────────────────────────────────────────

    def tick(self) -> dict:
        """Run one state-machine step. Returns target ticks dict."""
        ps = self.ps
        s  = ps.state

        if s == S.IDLE:
            pass

        elif s == S.SCAN_OBJECTS:
            # Detection is triggered asynchronously by the node calling
            # /detect_object service. The node transitions us to SELECT_OBJECT
            # when a result arrives. Nothing to do in tick.
            pass

        elif s == S.SELECT_OBJECT:
            # bbox received and /initialize_tracking called by node.
            # Transition happens after the tracking init service responds.
            pass

        elif s == S.PLAN_PICK:
            # Short-circuit: no separate plan step needed (IBVS is reactive).
            self.transition(S.MOVE_TO_PREGRASP)

        elif s == S.MOVE_TO_PREGRASP:
            self._ibvs_approach()
            vl53 = ps.range_mm
            if vl53 is not None and vl53 <= VL53_LOCK_DIST_MM:
                ps.arm_locked = True
                self.transition(S.ALIGN_WITH_GRIPPER_CAM)

        elif s == S.ALIGN_WITH_GRIPPER_CAM:
            self._ibvs_locked()
            vl53 = ps.range_mm
            if vl53 is not None and vl53 <= VL53_GRIP_DIST_MM:
                if not ps.pre_grasp_palm:
                    ps.pre_grasp_palm = True
                    ps.target['palm'] = int(_clamp(
                        ps.curr['palm'] + VL53_PREGRASP_PALM_DELTA,
                        PALM_MIN, PALM_MAX
                    ))
                    self._log('[FSM] Pre-grasp palm move started')
                elif abs(ps.curr['palm'] - ps.target['palm']) <= HOME_TOL:
                    self.transition(S.GRASP)

        elif s == S.GRASP:
            if not ps.gripper_closed:
                ps.target['gripper'] = GRIPPER_CLOSE
                ps.gripper_closed    = True
                ps.gripping          = True
                ps.grip_frames       = 0
                self._log('[FSM] Gripper closing')
            else:
                ps.grip_frames += 1
                if ps.grip_frames >= 10:  # GRIP_LOAD_MIN_FRAMES
                    self.transition(S.VERIFY_GRASP)

        elif s == S.VERIFY_GRASP:
            # Gripper current/load is checked by the node; it calls
            # on_grip_success() / on_grip_miss() based on hardware feedback.
            pass

        elif s == S.MOVE_TO_DROP:
            if not ps.returning_home:
                self._set_target_home()
                ps.returning_home = True
                self._log('[FSM] Moving to home before drop')
            if self._arrived_home():
                ps.returning_home = False
                ps.place_mode     = True
                ps.gripper_closed = True  # keep closed during place approach
                self.transition(S.PLACE)

        elif s == S.PLACE:
            self._ibvs_approach()
            vl53 = ps.range_mm
            if vl53 is not None and vl53 <= PLACE_DIST_MM:
                self._log('[FSM] Place distance reached — opening gripper')
                ps.target['gripper'] = GRIPPER_OPEN
                ps.gripper_closed    = False
                self.transition(S.VERIFY_PLACE)

        elif s == S.VERIFY_PLACE:
            self._set_target_home()
            self.transition(S.IDLE)
            self._log('[FSM] Task complete')

        elif s == S.RETRY_OR_DONE:
            if ps.attempt < ps.max_retries:
                ps.attempt += 1
                self._log(f'[FSM] Retry {ps.attempt}/{ps.max_retries}')
                self._reset_grip_flags()
                self._set_target_home()
                self.transition(S.SCAN_OBJECTS)
            else:
                self._log('[FSM] Max retries reached — task failed')
                self._set_target_home()
                self.transition(S.IDLE)

        return dict(ps.target)

    # ── event callbacks from node ─────────────────────────────────────────────

    def on_detection(self, x_min: float, y_min: float, x_max: float, y_max: float) -> None:
        """Called by the node when /detect_object service succeeds."""
        self.transition(S.SELECT_OBJECT)

    def on_tracking_ready(self) -> None:
        """Called by the node when /initialize_tracking service succeeds."""
        self.transition(S.PLAN_PICK)

    def on_grip_success(self) -> None:
        """Called by the node when gripper current/load confirms object caught."""
        self._log('[FSM] Grip success!')
        self.transition(S.MOVE_TO_DROP)

    def on_grip_miss(self) -> None:
        """Called by the node when GRIP_CHECK_FRAMES elapsed with no current spike."""
        self._log(f'[FSM] Grip miss (attempt {self.ps.attempt + 1})')
        self.ps.target['gripper'] = GRIPPER_OPEN
        self.ps.gripper_closed    = False
        self.transition(S.RETRY_OR_DONE)

    # ── IBVS helpers ──────────────────────────────────────────────────────────

    def _ibvs_approach(self) -> None:
        ps = self.ps
        if not ps.has_target:
            return
        w, h   = ps.frame_w, ps.frame_h
        err_x  = (w * APPROACH_AIM_X - ps.target_x) / w
        err_y  = (h * APPROACH_AIM_Y - ps.target_y) / h

        d_base = int(K_BASE * err_x)
        if abs(err_x) > CENTERED_X:
            ps.target['base'] = int(_clamp(ps.curr['base'] + d_base, 1000, 3000))

        if not ps.arm_locked:
            d_shoulder      = int(K_SHOULDER * err_y) * SHOULDER_DIR
            cf              = max(0.0, 1.0 - abs(err_y) / ELBOW_CENTERING_GATE)
            vl53            = ps.range_mm
            if vl53 is not None:
                err_area    = max(0.0, (vl53 - VL53_GRIP_DIST_MM) / VL53_MAX_APPROACH_MM)
            else:
                err_area    = 0.0
            d_elbow         = int(K_ELBOW * err_area * cf) * ELBOW_DIR
            ps.target['shoulder'] = int(_clamp(ps.curr['shoulder'] + d_shoulder, SH_MIN, SH_MAX))
            ps.target['elbow']    = int(_clamp(ps.curr['elbow']    + d_elbow,    EL_MIN, EL_MAX))

    def _ibvs_locked(self) -> None:
        ps  = self.ps
        vl53 = ps.range_mm
        if vl53 is None:
            return
        err         = max(0.0, (vl53 - VL53_GRIP_DIST_MM) / VL53_MAX_APPROACH_MM)
        d_elbow     = int(K_ELBOW * err) * ELBOW_DIR
        d_shoulder  = int(K_ELBOW * VL53_SHOULDER_RATIO * err) * SHOULDER_DIR
        ps.target['elbow']    = int(_clamp(ps.curr['elbow']    + d_elbow,    EL_MIN, EL_MAX))
        ps.target['shoulder'] = int(_clamp(ps.curr['shoulder'] + d_shoulder, SH_MIN, SH_MAX))

    def _set_target_home(self) -> None:
        self.ps.target = dict(self.ps.home_ticks)
        self.ps.target['gripper'] = GRIPPER_OPEN if not self.ps.gripper_closed else GRIPPER_CLOSE

    def _arrived_home(self) -> bool:
        ps = self.ps
        for joint in ('shoulder', 'elbow', 'palm'):
            if abs(ps.curr.get(joint, 2048) - ps.home_ticks.get(joint, 2048)) > HOME_TOL:
                return False
        return True

    def _reset_flags(self) -> None:
        ps = self.ps
        ps.arm_locked     = False
        ps.pre_grasp_palm = False
        ps.gripping       = False
        ps.gripper_closed = False
        ps.returning_home = False
        ps.place_mode     = False
        ps.grip_frames    = 0

    def _reset_grip_flags(self) -> None:
        ps = self.ps
        ps.arm_locked     = False
        ps.pre_grasp_palm = False
        ps.gripping       = False
        ps.gripper_closed = False
        ps.grip_frames    = 0
