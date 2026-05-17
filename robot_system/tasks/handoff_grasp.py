"""
Step 11 — Handoff grasp task.

Sequences the full pick-from-hand pipeline:

  IDLE
    │  start(frame, click_xy)
    ▼
  CENTERING  — VisualServo centres the object in frame
    │  state.is_centred
    ▼
  APPROACHING — VisualServo drives elbow; DistanceServo watches distance
    │  dist_state.should_grip  (stable + close)
    ▼
  GRIPPING   — close gripper, freeze arm
    │  (immediate)
    ▼
  DONE       — success

  Any state → FAILED on timeout or explicit abort().

Usage (dry-run, no hardware):
    import numpy as np
    from perception.sam2_adapter import Sam2Adapter
    from control.lerobot_adapter import LeRobotAdapter
    from control.visual_servo import VisualServo
    from control.distance_servo import DistanceServo, MockDistanceSensor
    from tasks.handoff_grasp import HandoffGrasp, GraspConfig

    sam     = Sam2Adapter(dry_run=True)
    adapter = LeRobotAdapter(dry_run=True)
    v_servo = VisualServo(sam, adapter)
    d_servo = DistanceServo(MockDistanceSensor([300, 200, 100, 70, 68, 65]))
    grasp   = HandoffGrasp(v_servo, d_servo, adapter)

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    grasp.start(frame, click_xy=(320, 240))

    for _ in range(50):
        result = grasp.step(frame)
        print(result.phase, result.message)
        if result.done:
            break
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional

import numpy as np

from control.visual_servo import VisualServo, ServoState
from control.distance_servo import DistanceServo, DistanceState
from control.lerobot_adapter import LeRobotAdapter


# ------------------------------------------------------------------
# Phase enum
# ------------------------------------------------------------------

class GraspPhase(Enum):
    IDLE       = auto()
    CENTERING  = auto()
    APPROACHING= auto()
    GRIPPING   = auto()
    DONE       = auto()
    FAILED     = auto()


# ------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------

@dataclass
class GraspConfig:
    """Timeouts and thresholds for the grasp task."""
    centering_timeout_s:   float = 15.0   # max time to centre the object
    approach_timeout_s:    float = 30.0   # max time for distance approach
    grip_hold_s:           float = 0.5    # pause after closing gripper
    return_home_after:     bool  = False  # go home on success (needs real HW)


# ------------------------------------------------------------------
# Per-step result
# ------------------------------------------------------------------

@dataclass
class GraspResult:
    """Returned by HandoffGrasp.step() each cycle."""
    phase:        GraspPhase
    message:      str
    done:         bool          # True when terminal (DONE or FAILED)
    success:      bool          # True only on DONE
    v_state:      Optional[ServoState]   = None
    d_state:      Optional[DistanceState] = None
    elapsed_s:    float = 0.0

    def __str__(self) -> str:
        return f"[{self.phase.name:12s}] {self.message}  ({self.elapsed_s:.1f}s)"


# ------------------------------------------------------------------
# Task
# ------------------------------------------------------------------

class HandoffGrasp:
    """
    Single-object handoff grasp orchestrator.

    Call start() once to lock on, then call step() in a loop until
    result.done is True.
    """

    def __init__(
        self,
        visual_servo:   VisualServo,
        distance_servo: DistanceServo,
        adapter:        LeRobotAdapter,
        config:         GraspConfig = GraspConfig(),
    ) -> None:
        self.vs     = visual_servo
        self.ds     = distance_servo
        self.adapter = adapter
        self.cfg    = config

        self._phase:   GraspPhase = GraspPhase.IDLE
        self._t_phase: float = 0.0   # wall time when current phase started
        self._t_start: float = 0.0   # wall time when task started

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, frame: np.ndarray, click_xy: tuple[float, float]) -> None:
        """Lock on to the object and begin the grasp sequence."""
        self.vs.start_tracking(frame, click_xy)
        self.ds.reset()
        self._enter(GraspPhase.CENTERING)
        self._t_start = self._t_phase

    def abort(self) -> GraspResult:
        """Immediately stop and mark as failed."""
        self.vs.stop_tracking()
        self._enter(GraspPhase.FAILED)
        return self._result("Aborted by caller.", v_state=None, d_state=None)

    def step(self, frame: np.ndarray) -> GraspResult:
        """
        Advance the state machine by one cycle.

        Must be called after start().  Pass the latest camera frame each cycle.
        Returns a GraspResult; check result.done to know when to stop.
        """
        if self._phase in (GraspPhase.IDLE, GraspPhase.DONE, GraspPhase.FAILED):
            return self._result("Task not running.", v_state=None, d_state=None)

        elapsed = time.monotonic() - self._t_start

        if self._phase == GraspPhase.CENTERING:
            return self._centering(frame, elapsed)

        if self._phase == GraspPhase.APPROACHING:
            return self._approaching(frame, elapsed)

        if self._phase == GraspPhase.GRIPPING:
            return self._gripping(elapsed)

        return self._result("Unknown phase.", v_state=None, d_state=None)

    @property
    def phase(self) -> GraspPhase:
        return self._phase

    # ------------------------------------------------------------------
    # Phase handlers
    # ------------------------------------------------------------------

    def _centering(self, frame: np.ndarray, elapsed: float) -> GraspResult:
        v = self.vs.step(frame)

        if elapsed > self.cfg.centering_timeout_s:
            self.vs.stop_tracking()
            self._enter(GraspPhase.FAILED)
            return self._result("Centering timed out.", v_state=v)

        if v.is_centred:
            self._enter(GraspPhase.APPROACHING)
            return self._result("Object centred — starting approach.", v_state=v)

        nx, ny = v.centroid_norm
        return self._result(
            f"Centering  norm=({nx:+.2f},{ny:+.2f})", v_state=v
        )

    def _approaching(self, frame: np.ndarray, elapsed: float) -> GraspResult:
        v = self.vs.step(frame, repredict=True)
        d = self.ds.step(self.adapter)

        phase_elapsed = time.monotonic() - self._t_phase
        if phase_elapsed > self.cfg.approach_timeout_s:
            self.vs.stop_tracking()
            self._enter(GraspPhase.FAILED)
            return self._result("Approach timed out.", v_state=v, d_state=d)

        if d.should_grip:
            self._enter(GraspPhase.GRIPPING)
            return self._result("Stable and close — gripping.", v_state=v, d_state=d)

        dist_str = f"{d.dist_mm} mm" if d.dist_mm is not None else "no sensor"
        return self._result(
            f"Approaching  dist={dist_str}  fill={v.frame_fill:.3f}  "
            f"confirm={d.confirm_count}",
            v_state=v, d_state=d,
        )

    def _gripping(self, elapsed: float) -> GraspResult:
        self.vs.stop_tracking()
        self.adapter.close_gripper()
        time.sleep(self.cfg.grip_hold_s)

        if self.cfg.return_home_after:
            self.adapter.move_to_home()

        self._enter(GraspPhase.DONE)
        return self._result("Grip complete.", v_state=None, d_state=None, success=True)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _enter(self, phase: GraspPhase) -> None:
        self._phase   = phase
        self._t_phase = time.monotonic()

    def _result(
        self,
        message: str,
        v_state: Optional[ServoState] = None,
        d_state: Optional[DistanceState] = None,
        success: bool = False,
    ) -> GraspResult:
        done = self._phase in (GraspPhase.DONE, GraspPhase.FAILED)
        return GraspResult(
            phase     = self._phase,
            message   = message,
            done      = done,
            success   = success,
            v_state   = v_state,
            d_state   = d_state,
            elapsed_s = time.monotonic() - self._t_start,
        )
