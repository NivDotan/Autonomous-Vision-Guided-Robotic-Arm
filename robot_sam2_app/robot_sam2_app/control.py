from __future__ import annotations

import cv2

from . import config as cfg
from .state import RobotState
from .tracking import TrackingResult
from .utils import clamp, count_fingers


class MotionController:
    """Vision-to-joint target logic."""

    def update_from_object(self, state: RobotState, result: TrackingResult, frame_width: int, frame_height: int) -> str:
        if not result.success:
            return "CLICK OBJECT OR PRESS U/T"

        if state.auto_palm:
            self._auto_align_palm(state, result.width, result.height)

        if result.area >= cfg.APPROACH_THRESHOLD:
            state.object_reached = True

        cx_frame, cy_frame = frame_width // 2, frame_height // 2
        err_x = (cx_frame * cfg.AIM_X - result.center_x) / frame_width
        err_y = (cy_frame * cfg.AIM_Y - result.center_y) / frame_height

        d_base = int(cfg.K_BASE * err_x)
        d_shoulder = int(cfg.K_SHOULDER * err_y) * cfg.SHOULDER_DIR
        d_elbow = 0

        centered = abs(err_x) < cfg.CENTERED_X and abs(err_y) < cfg.CENTERED_Y
        status = "OBJECT TRACKING"
        if state.approach_mode and not state.object_reached:
            status = "APPROACHING..." if centered else "CENTERING..."
            if centered and not state.vl53_controls_elbow:
                # Sensor takes over elbow when active — camera only centers base/shoulder
                d_elbow = -int(cfg.K_ELBOW) * cfg.ELBOW_DIR
                compensation = d_elbow * cfg.SHOULDER_COMPENSATION_RATIO * cfg.SHOULDER_DIR
                d_shoulder -= int(compensation)
        elif state.object_reached:
            status = "REACHED"

        state.target["base"] = int(clamp(state.curr["base"] + d_base, 1000, 3000))
        state.target["shoulder"] = int(clamp(state.curr["shoulder"] + d_shoulder, cfg.SH_MIN, cfg.SH_MAX))
        if not state.vl53_controls_elbow:
            state.target["elbow"] = int(clamp(state.curr["elbow"] + d_elbow, cfg.EL_MIN, cfg.EL_MAX))
        return status

    def update_from_hand(self, state: RobotState, results) -> str:
        if not results.multi_hand_landmarks:
            return "IDLE"
        hand = results.multi_hand_landmarks[0]
        if state.is_frozen:
            return "FROZEN"

        hand_x, hand_y = hand.landmark[9].x, hand.landmark[9].y
        err_x, err_y = 0.5 - hand_x, 0.5 - hand_y
        if abs(err_x) > cfg.DEADBAND_X:
            state.target["base"] = int(clamp(state.curr["base"] + err_x * 150, 1000, 3000))
        if abs(err_y) > cfg.DEADBAND_Y:
            state.target["shoulder"] = int(clamp(state.curr["shoulder"] + err_y * 150 * cfg.SHOULDER_DIR, cfg.SH_MIN, cfg.SH_MAX))

        fingers = count_fingers(hand)
        if fingers == 2:
            state.target["elbow"] = int(clamp(state.target["elbow"] + 30 * cfg.ELBOW_DIR, cfg.EL_MIN, cfg.EL_MAX))
        elif fingers <= 1:
            state.target["elbow"] = int(clamp(state.target["elbow"] - 30 * cfg.ELBOW_DIR, cfg.EL_MIN, cfg.EL_MAX))
        return "HAND CONTROL"

    @staticmethod
    def _auto_align_palm(state: RobotState, width: int, height: int) -> None:
        if height <= 0:
            return
        ratio = width / height
        if ratio > 1.2:
            target = cfg.PALM_MIN
        elif ratio < 0.8:
            target = cfg.PALM_MAX
        else:
            target = state.curr["palm"]
        state.target["palm"] = int(clamp(target, cfg.PALM_MIN, cfg.PALM_MAX))


def draw_overlay(frame, status: str, state: RobotState, target_name: str,
                 vl53_dist_mm: int | None = None) -> None:
    cv2.putText(frame, f"Status: {status}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Palm: {'AUTO' if state.auto_palm else 'MANUAL'}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)
    ctrl = "SIM JOG" if state.sim_jog_active else "vision"
    cv2.putText(frame, f"Arm ctrl: {ctrl}", (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (180, 180, 255), 2)
    cv2.putText(frame, f"RF-DETR: {target_name}", (20, 155), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 255), 2)

    if vl53_dist_mm is not None:
        dist_cm = vl53_dist_mm / 10.0
        label = f"VL53: {dist_cm:.1f} cm"
        color = (0, 255, 255) if dist_cm > 5 else (0, 80, 255)  # yellow → red when very close
        cv2.putText(frame, label, (20, 190), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
