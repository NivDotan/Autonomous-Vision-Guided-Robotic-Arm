"""
Step 13 — Table-pick task.

Picks an object from a table surface using:
  1. Base camera + calibration  -> robot-frame target (x, y, z)
  2. IK                         -> joint angles for pre-grasp pose
  3. Trajectory                 -> smooth move to pre-grasp
  4. VisualServo + DistanceServo -> fine approach and grip

State machine phases:
  IDLE -> DETECTING -> MOVING_TO_PREGRASP -> FINE_APPROACH -> GRIPPING -> DONE / FAILED

Usage (dry-run, no hardware):
    import numpy as np
    from tasks.table_pick import TablePick, TablePickConfig
    from perception.base_camera_detector import BaseDetector
    from calibration.base_camera_to_robot import CameraRobotCalibration
    from perception.sam2_adapter import Sam2Adapter
    from control.lerobot_adapter import LeRobotAdapter
    from control.visual_servo import VisualServo
    from control.distance_servo import DistanceServo, MockDistanceSensor

    cal  = _make_dummy_calibration()
    task = TablePick(
        detector=BaseDetector(dry_run=True),
        calibration=cal,
        sam=Sam2Adapter(dry_run=True),
        adapter=LeRobotAdapter(dry_run=True),
        distance_sensor=MockDistanceSensor([200, 100, 70, 65, 65, 65]),
    )
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    task.start(frame)
    for _ in range(60):
        r = task.step(frame)
        if r.done:
            break
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

from perception.base_camera_detector import BaseDetector, DetectedObject
from calibration.base_camera_to_robot import CameraRobotCalibration
from perception.sam2_adapter import Sam2Adapter
from control.lerobot_adapter import LeRobotAdapter
from control.visual_servo import VisualServo, ServoConfig
from control.distance_servo import DistanceServo, DistanceConfig, DistanceSensor
from control.trajectory import Trajectory, EasingCurve
from kinematics.inverse_kinematics import inverse_kinematics, UnreachableTargetError, JointLimitError


# ------------------------------------------------------------------
# Phase + result
# ------------------------------------------------------------------

class PickPhase(Enum):
    IDLE             = auto()
    DETECTING        = auto()
    MOVING_TO_PREGRASP = auto()
    FINE_APPROACH    = auto()
    GRIPPING         = auto()
    DONE             = auto()
    FAILED           = auto()


@dataclass
class PickResult:
    phase:   PickPhase
    message: str
    done:    bool
    success: bool
    elapsed_s: float = 0.0
    target_robot: Optional[tuple[float, float, float]] = None

    def __str__(self) -> str:
        return f"[{self.phase.name:20s}] {self.message}  ({self.elapsed_s:.1f}s)"


# ------------------------------------------------------------------
# Config
# ------------------------------------------------------------------

@dataclass
class TablePickConfig:
    pregrasp_z_offset:   float = 0.08    # metres above target before descending
    pregrasp_phi:        float = -0.4    # tool pitch for pre-grasp (rad, ~-23 deg)
    grasp_phi:           float = -0.7    # tool pitch for final grasp
    trajectory_duration: float = 3.0    # seconds for pre-grasp move
    detect_timeout_s:    float = 5.0
    approach_timeout_s:  float = 20.0
    grip_hold_s:         float = 0.4
    return_home_after:   bool  = False


# ------------------------------------------------------------------
# Task
# ------------------------------------------------------------------

class TablePick:
    """
    Full table-pick orchestrator.

    Parameters
    ----------
    detector:        BaseDetector for the overhead camera
    calibration:     fitted CameraRobotCalibration
    sam:             Sam2Adapter for wrist camera
    adapter:         LeRobotAdapter (dry-run or live)
    distance_sensor: VL53 sensor or MockDistanceSensor
    v_config:        VisualServo tuning
    d_config:        DistanceServo tuning
    config:          task-level parameters
    """

    def __init__(
        self,
        detector:        BaseDetector,
        calibration:     CameraRobotCalibration,
        sam:             Sam2Adapter,
        adapter:         LeRobotAdapter,
        distance_sensor: DistanceSensor,
        v_config:        ServoConfig    = ServoConfig(),
        d_config:        DistanceConfig = DistanceConfig(),
        config:          TablePickConfig = TablePickConfig(),
    ) -> None:
        self.det   = detector
        self.cal   = calibration
        self.sam   = sam
        self.adapter = adapter
        self.vs    = VisualServo(sam, adapter, config=v_config)
        self.ds    = DistanceServo(distance_sensor, config=d_config)
        self.cfg   = config

        self._phase:   PickPhase = PickPhase.IDLE
        self._t_start: float = 0.0
        self._t_phase: float = 0.0
        self._target:  Optional[tuple[float, float, float]] = None
        self._detected: Optional[DetectedObject] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, base_frame: np.ndarray) -> None:
        """Begin the pick sequence using the given base-camera frame."""
        self._t_start = time.monotonic()
        self._enter(PickPhase.DETECTING)
        self._detect(base_frame)

    def step(self, wrist_frame: np.ndarray) -> PickResult:
        """Advance the state machine.  Pass the current wrist-camera frame."""
        if self._phase in (PickPhase.IDLE, PickPhase.DONE, PickPhase.FAILED):
            return self._result("Task not running.")

        elapsed = time.monotonic() - self._t_start

        if self._phase == PickPhase.DETECTING:
            return self._detecting(elapsed)
        if self._phase == PickPhase.MOVING_TO_PREGRASP:
            return self._moving_to_pregrasp(elapsed)
        if self._phase == PickPhase.FINE_APPROACH:
            return self._fine_approach(wrist_frame, elapsed)
        if self._phase == PickPhase.GRIPPING:
            return self._gripping(elapsed)

        return self._result("Unknown phase.")

    @property
    def phase(self) -> PickPhase:
        return self._phase

    # ------------------------------------------------------------------
    # Phase handlers
    # ------------------------------------------------------------------

    def _detect(self, frame: np.ndarray) -> None:
        objects = self.det.detect(frame)
        if not objects:
            self._enter(PickPhase.FAILED)
            return
        obj = objects[0]   # largest object
        self._detected = obj
        try:
            x, y, z = self.cal.pixel_to_robot(*obj.centroid_px)
            # Pre-grasp: hover above target
            self._target = (x, y, z + self.cfg.pregrasp_z_offset)
        except Exception:
            self._enter(PickPhase.FAILED)
            return
        self._enter(PickPhase.MOVING_TO_PREGRASP)

    def _detecting(self, elapsed: float) -> PickResult:
        # Detection is synchronous in _detect(); this phase is transient.
        if elapsed > self.cfg.detect_timeout_s:
            self._enter(PickPhase.FAILED)
            return self._result("Detection timed out.")
        return self._result("Detecting object...")

    def _moving_to_pregrasp(self, elapsed: float) -> PickResult:
        assert self._target is not None
        x, y, z = self._target

        try:
            q = inverse_kinematics(x, y, z, phi=self.cfg.pregrasp_phi)
        except (UnreachableTargetError, JointLimitError) as e:
            self._enter(PickPhase.FAILED)
            return self._result(f"IK failed: {e}")

        start = self.adapter.observe()
        goal  = {
            "shoulder_pan":  q["q1"],
            "shoulder_lift": q["q2"],
            "elbow_flex":    q["q3"],
            "wrist_flex":    q["q4"],
            "wrist_roll":    q["q5"],
        }
        traj = Trajectory.between(
            start, goal,
            duration=self.cfg.trajectory_duration,
            easing=EasingCurve.EASE_IN_OUT,
        )
        traj.execute(self.adapter, hz=50)

        # Kick off visual servo on the wrist camera
        px, py = (
            self._detected.centroid_px
            if self._detected else (320, 240)
        )
        self.ds.reset()
        self._enter(PickPhase.FINE_APPROACH)
        return self._result(
            f"At pre-grasp ({x:.3f},{y:.3f},{z:.3f}) — starting fine approach.",
            target=self._target,
        )

    def _fine_approach(self, frame: np.ndarray, elapsed: float) -> PickResult:
        if not self.vs.is_tracking:
            # First frame in this phase: start tracking centre of wrist image
            H, W = frame.shape[:2]
            self.vs.start_tracking(frame, click_xy=(W // 2, H // 2))

        phase_elapsed = time.monotonic() - self._t_phase
        if phase_elapsed > self.cfg.approach_timeout_s:
            self.vs.stop_tracking()
            self._enter(PickPhase.FAILED)
            return self._result("Fine approach timed out.")

        v = self.vs.step(frame)
        d = self.ds.step(self.adapter)

        if d.should_grip:
            self.vs.stop_tracking()
            self._enter(PickPhase.GRIPPING)
            return self._result("Stable and close — gripping.", target=self._target)

        dist_str = f"{d.dist_mm} mm" if d.dist_mm is not None else "no sensor"
        return self._result(
            f"Fine approach  dist={dist_str}  fill={v.frame_fill:.3f}",
            target=self._target,
        )

    def _gripping(self, elapsed: float) -> PickResult:
        self.adapter.close_gripper()
        time.sleep(self.cfg.grip_hold_s)
        if self.cfg.return_home_after:
            self.adapter.move_to_home()
        self._enter(PickPhase.DONE)
        return self._result("Pick complete.", success=True, target=self._target)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _enter(self, phase: PickPhase) -> None:
        self._phase   = phase
        self._t_phase = time.monotonic()

    def _result(
        self,
        message: str,
        success: bool = False,
        target: Optional[tuple[float, float, float]] = None,
    ) -> PickResult:
        done = self._phase in (PickPhase.DONE, PickPhase.FAILED)
        return PickResult(
            phase        = self._phase,
            message      = message,
            done         = done,
            success      = success,
            elapsed_s    = time.monotonic() - self._t_start,
            target_robot = target,
        )
