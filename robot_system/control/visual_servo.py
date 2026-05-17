"""
Step 9 — Visual servo controller.

Uses SAM2 mask features (centroid offset, object area) to drive the arm
so the tracked object stays centred in the frame and the arm approaches
until the object fills a target fraction of the frame.

Control law (P-controller):
    Δshoulder_pan  = -Kp_pan  * centroid_norm.x
    Δshoulder_lift = -Kp_tilt * centroid_norm.y
    Δelbow_flex    = +Kp_reach * (target_fill - frame_fill)   [approach only]

Dead zones prevent oscillation around the setpoint.

Usage (dry-run, no robot or model required):
    import numpy as np
    from perception.sam2_adapter import Sam2Adapter
    from control.lerobot_adapter import LeRobotAdapter
    from control.visual_servo import VisualServo, ServoConfig

    sam     = Sam2Adapter(dry_run=True)
    adapter = LeRobotAdapter(dry_run=True)
    servo   = VisualServo(sam, adapter)

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    servo.start_tracking(frame, click_xy=(320, 240))

    for _ in range(10):
        state = servo.step(frame)
        print(state)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from perception.sam2_adapter import Sam2Adapter, PredictionResult
from perception.mask_features import MaskFeatures
from control.lerobot_adapter import LeRobotAdapter


@dataclass
class ServoConfig:
    """Tuning knobs for the visual servo controller."""

    # P-gains (rad per unit normalised error)
    Kp_pan:   float = 0.3    # base yaw correction per unit x-error
    Kp_tilt:  float = 0.2    # shoulder tilt correction per unit y-error
    Kp_reach: float = 0.4    # elbow extension per unit fill-error

    # Dead zones (normalised units) — no correction inside these bands
    dead_pan:   float = 0.05   # ±5 % of half-frame width
    dead_tilt:  float = 0.05
    dead_reach: float = 0.02   # ±2 % of frame area

    # Joint limits (radians) — clamp corrections to stay safe
    pan_limit:   tuple[float, float] = (-math.pi,      math.pi)
    tilt_limit:  tuple[float, float] = (-math.pi / 2,  math.pi / 2)
    elbow_limit: tuple[float, float] = (-math.pi * 3/4, math.pi * 3/4)

    # Approach target: stop driving elbow when object fills this much of frame
    target_fill: float = 0.12    # 12 % → "object reached"

    # Convergence thresholds
    centre_tol:  float = 0.08    # |centroid_norm| < this in both axes → centred
    reached_fill: float = 0.10   # frame_fill above this → object reached


@dataclass
class ServoState:
    """Snapshot returned by VisualServo.step()."""
    centroid_norm:  tuple[float, float]   # (x, y) in [-1, 1]
    frame_fill:     float                 # object area / frame area
    pan_correction:   float               # joint delta applied this step (rad)
    tilt_correction:  float
    reach_correction: float
    score:          float                 # SAM2 confidence
    is_centred:     bool                  # centroid inside dead zone
    object_reached: bool                  # fill above reached_fill
    tracking:       bool                  # True while a target is locked

    def __str__(self) -> str:
        cx, cy = self.centroid_norm
        return (
            f"ServoState  norm=({cx:+.3f},{cy:+.3f})  fill={self.frame_fill:.3f}  "
            f"score={self.score:.2f}  centred={self.is_centred}  "
            f"reached={self.object_reached}"
        )


class VisualServo:
    """
    Single-object visual servo controller.

    Call ``start_tracking(frame, click_xy)`` once to lock on, then call
    ``step(frame)`` every control cycle.
    """

    def __init__(
        self,
        sam:     Sam2Adapter,
        adapter: LeRobotAdapter,
        config:  ServoConfig = ServoConfig(),
        approach_enabled: bool = True,
    ) -> None:
        self.sam      = sam
        self.adapter  = adapter
        self.cfg      = config
        self.approach = approach_enabled

        self._tracking:   bool = False
        self._last_click: Optional[tuple[float, float]] = None

    # ------------------------------------------------------------------
    # Control interface
    # ------------------------------------------------------------------

    def start_tracking(
        self,
        frame:    np.ndarray,
        click_xy: tuple[float, float],
    ) -> PredictionResult:
        """
        Lock on to the object at click_xy.

        Sets the image in the SAM2 adapter and runs an initial prediction
        so the first step() has a reference.
        """
        self._last_click = click_xy
        self._tracking   = True

        self.sam.set_image(frame)
        result = self.sam.predict(click_xy)
        return result

    def stop_tracking(self) -> None:
        self._tracking   = False
        self._last_click = None
        self.sam.reset()

    def step(
        self,
        frame: np.ndarray,
        repredict: bool = True,
    ) -> ServoState:
        """
        Run one servo cycle.

        Args:
            frame:      current camera frame (H×W×3 uint8 RGB)
            repredict:  if True, re-run SAM2 on the new frame.
                        Set False to reuse the last mask (faster, less accurate).

        Returns:
            ServoState with the corrections applied and convergence flags.
        """
        if not self._tracking:
            return ServoState(
                centroid_norm=(0.0, 0.0), frame_fill=0.0,
                pan_correction=0.0, tilt_correction=0.0, reach_correction=0.0,
                score=0.0, is_centred=False, object_reached=False, tracking=False,
            )

        # -- Perception ------------------------------------------------
        self.sam.set_image(frame)
        result = self.sam.predict(self._last_click)  # type: ignore[arg-type]
        f = result.features

        # -- Errors ----------------------------------------------------
        ex, ey = f.centroid_norm          # signed: positive = right/down
        fill_err = self.cfg.target_fill - f.frame_fill   # positive = need to approach

        # -- Dead-zone P-control ---------------------------------------
        # ex > 0 = object right → increase shoulder_pan to track right
        # ey > 0 = object below  → increase shoulder_lift to track down
        d_pan   = +self.cfg.Kp_pan  * _dead(ex, self.cfg.dead_pan)
        d_tilt  = +self.cfg.Kp_tilt * _dead(ey, self.cfg.dead_tilt)
        d_reach = (
            self.cfg.Kp_reach * _dead(fill_err, self.cfg.dead_reach)
            if self.approach and not f.frame_fill >= self.cfg.reached_fill
            else 0.0
        )

        # -- Apply to adapter ------------------------------------------
        curr = self.adapter.observe()

        new_pan   = _clamp(curr.get("shoulder_pan",  0.0) + d_pan,   self.cfg.pan_limit)
        new_tilt  = _clamp(curr.get("shoulder_lift", 0.0) + d_tilt,  self.cfg.tilt_limit)
        new_elbow = _clamp(curr.get("elbow_flex",    0.0) + d_reach, self.cfg.elbow_limit)

        joints: dict[str, float] = {"shoulder_pan": new_pan, "shoulder_lift": new_tilt}
        if self.approach:
            joints["elbow_flex"] = new_elbow

        self.adapter.move_joints(joints, blocking=False)

        # -- Convergence flags -----------------------------------------
        is_centred = abs(ex) < self.cfg.centre_tol and abs(ey) < self.cfg.centre_tol
        reached    = f.frame_fill >= self.cfg.reached_fill

        return ServoState(
            centroid_norm=(ex, ey),
            frame_fill=f.frame_fill,
            pan_correction=d_pan,
            tilt_correction=d_tilt,
            reach_correction=d_reach,
            score=result.score,
            is_centred=is_centred,
            object_reached=reached,
            tracking=True,
        )

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def is_tracking(self) -> bool:
        return self._tracking


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _dead(value: float, zone: float) -> float:
    """Return value with a dead zone around zero."""
    if abs(value) < zone:
        return 0.0
    return value - math.copysign(zone, value)


def _clamp(value: float, limits: tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))
