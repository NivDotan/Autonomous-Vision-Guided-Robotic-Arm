from __future__ import annotations

import cv2

from . import config as cfg
from .state import RobotState
from .tracking import TrackingResult
from .utils import clamp, count_fingers


class MotionController:
    """Vision-to-joint target logic."""

    def update_from_object(self, state: RobotState, result: TrackingResult, frame_width: int, frame_height: int) -> str:
        # Palm adjusting before grip — freeze everything, don't touch any targets
        if state.pre_grasp_palm:
            return "PRE-GRASP"

        if not result.success:
            state.target["base"] = state.curr["base"]
            state.target["shoulder"] = state.curr["shoulder"]
            state.is_centered = False
            # If locked and VL53 is active, keep elbow extending even without SAM2
            if state.arm_locked and state.approach_mode and state.vl53_dist_mm is not None:
                err_area = max(0.0, (state.vl53_dist_mm - cfg.VL53_GRIP_DIST_MM) / cfg.VL53_MAX_APPROACH_MM)
                d_elbow = int(cfg.K_ELBOW * err_area) * cfg.ELBOW_DIR
                state.target["elbow"] = int(clamp(state.curr["elbow"] + d_elbow, cfg.EL_MIN, cfg.EL_MAX))
                return "LOCKED - VL53 ONLY"
            return "TRACKING LOST - HOLDING"

        if state.auto_palm and not state.approach_mode and not state.arm_locked and not state.pre_grasp_palm:
            self._auto_align_palm(state, result.width, result.height)

        # Motor 1 (base): correct X — target shifts during approach to 4x4 cell 6/10 region
        # Use dynamic aim from retry state machine if set, otherwise use config defaults
        aim_x = (state.current_aim_x if state.approach_mode and state.current_aim_x is not None
                 else (cfg.APPROACH_AIM_X if state.approach_mode else 0.5))
        err_x = (frame_width * aim_x - result.center_x) / frame_width
        d_base = int(cfg.K_BASE * err_x)
        centered_x = abs(err_x) < cfg.CENTERED_X

        # Motors 2+3+4: only during approach
        d_shoulder = 0
        d_elbow = 0
        d_palm = 0
        col = min(int(result.center_x * 4 / frame_width), 3)
        row = min(int(result.center_y * 4 / frame_height), 3)
        cell = row * 4 + col + 1  # 1-16
        if state.approach_mode:
            aim_y = (state.current_aim_y if state.current_aim_y is not None
                     else cfg.APPROACH_AIM_Y)
            err_y    = (frame_height * aim_y - result.center_y) / frame_height
            if state.vl53_dist_mm is not None:
                err_area = max(0.0, (state.vl53_dist_mm - cfg.VL53_GRIP_DIST_MM) / cfg.VL53_MAX_APPROACH_MM)
            else:
                err_area = max(0.0, (cfg.APPROACH_THRESHOLD - result.area) / cfg.APPROACH_THRESHOLD)

            # IBVS: each image feature error drives one joint
            d_shoulder = int(cfg.K_SHOULDER * err_y) * cfg.SHOULDER_DIR
            centering_factor = max(0.0, 1.0 - abs(err_y) / cfg.ELBOW_CENTERING_GATE)
            d_elbow = int(cfg.K_ELBOW * err_area * centering_factor) * cfg.ELBOW_DIR

            # Motor 4 (palm): help when object not in middle two rows (rows 1-2 of 4x4)
            if row not in (1, 2):
                d_palm = int(cfg.K_SHOULDER * err_y * 0.5) * cfg.PALM_DIR

        _aim_y_for_centered = (state.current_aim_y if state.approach_mode and state.current_aim_y is not None
                               else cfg.APPROACH_AIM_Y)
        centered_y = abs((frame_height * _aim_y_for_centered - result.center_y) / frame_height) < cfg.CENTERED_Y
        state.is_centered = centered_x and centered_y

        # Status
        if state.approach_mode:
            status = f"APPROACHING area={result.area} cell={cell}"
        else:
            status = "CENTERING..." if not centered_x else "CENTERED"

        # When locked (too close for vision): elbow + shoulder both driven by VL53, no centering gate
        if state.arm_locked:
            if state.vl53_dist_mm is not None:
                _err = max(0.0, (state.vl53_dist_mm - cfg.VL53_GRIP_DIST_MM) / cfg.VL53_MAX_APPROACH_MM)
                _d_elbow    = int(cfg.K_ELBOW * _err) * cfg.ELBOW_DIR
                _d_shoulder = int(cfg.K_ELBOW * cfg.VL53_SHOULDER_RATIO * _err) * cfg.SHOULDER_DIR
                state.target["elbow"]    = int(clamp(state.curr["elbow"]    + _d_elbow,    cfg.EL_MIN,  cfg.EL_MAX))
                state.target["shoulder"] = int(clamp(state.curr["shoulder"] + _d_shoulder, cfg.SH_MIN,  cfg.SH_MAX))
            return "LOCKED - VL53 ONLY"

        # Apply targets
        state.target["base"] = state.curr["base"] if centered_x else int(clamp(state.curr["base"] + d_base, 1000, 3000))
        state.target["shoulder"] = int(clamp(state.curr["shoulder"] + d_shoulder, cfg.SH_MIN, cfg.SH_MAX))
        state.target["elbow"] = int(clamp(state.curr["elbow"] + d_elbow, cfg.EL_MIN, cfg.EL_MAX))
        if d_palm != 0:
            state.target["palm"] = int(clamp(state.curr["palm"] + d_palm, cfg.PALM_MIN, cfg.PALM_MAX))
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
