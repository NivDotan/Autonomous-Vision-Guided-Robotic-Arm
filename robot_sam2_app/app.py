from __future__ import annotations

import cv2
import mediapipe as mp

from . import config as cfg
from .control import MotionController, draw_overlay
from .hardware import FeetechHardware, home_ticks_to_state
from .simulation import PyBulletArmSim
from .state import RobotState
from .tracking import ObjectTracker
from .utils import clamp, step_toward
from .vision.rfdetr_selector import RFDETRTargetSelector
from .vision.sam2_segmenter import SAM2Segmenter


class RobotApp:
    """Main orchestration class for camera, vision, sim, and hardware."""

    def __init__(self):
        self.state = RobotState()
        self.hardware = FeetechHardware()
        self.sim: PyBulletArmSim | None = None
        self.segmenter = SAM2Segmenter()
        self.detector = RFDETRTargetSelector()
        self.tracker = ObjectTracker()
        self.controller = MotionController()
        self.mp_hands = mp.solutions.hands
        self.cap = cv2.VideoCapture(0)
        self.frame_index = 0
        self.last_frame_bgr = None
        self.auto_target_name = cfg.DEFAULT_TARGET_CLASS

    def setup(self) -> None:
        self.hardware.connect()
        self.state.home = self.hardware.load_home(cfg.HOME_POSITION_PATH)
        if self.state.home:
            self.state.set_curr_and_target(home_ticks_to_state(self.state.home))
            self.hardware.write_home(self.state.home)
        self._start_sim()

    def _start_sim(self) -> None:
        try:
            self.sim = PyBulletArmSim(cfg.SIM_CALIBRATION_PATH)
            self.sim.connect()
            self.sim.recreate_sliders_from_ticks(self.state.ticks())
            print("PyBullet sim ready. Press J for sim jog, R to resync sliders.")
        except Exception as exc:
            self.sim = None
            print(f"PyBullet sim not started: {exc}")

    def run(self) -> None:
        self.setup()
        print("Controls: S motors, M mode, A approach, Space grab/release, U auto cup, T typed target")
        print("          J sim jog, R sync sim, Z/X manual palm, C auto palm, Q quit")
        cv2.namedWindow("Robot Brain")
        cv2.setMouseCallback("Robot Brain", self._on_mouse)

        with self.mp_hands.Hands(min_detection_confidence=0.6) as hands:
            while self.cap.isOpened():
                ok, frame = self.cap.read()
                if not ok:
                    break
                frame = cv2.flip(frame, 1)
                self.last_frame_bgr = frame.copy()
                h, w, _ = frame.shape
                results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

                status = self._update_vision(results, frame, w, h)
                self._update_sim()
                self._update_motion()
                draw_overlay(frame, status, self.state, self.auto_target_name)

                cv2.imshow("Robot Brain", frame)
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    break
                self._handle_key(key)
                self.frame_index += 1

        self.close()

    def close(self) -> None:
        if self.sim is not None:
            self.sim.disconnect()
        self.hardware.disconnect()
        self.cap.release()
        cv2.destroyAllWindows()

    def _on_mouse(self, event, x, y, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN and self.state.tracking_mode == "OBJECT":
            self.tracker.request_click(x, y)
            self.state.object_reached = False

    def _update_vision(self, results, frame, width: int, height: int) -> str:
        grip_status = self._handle_grip_state()
        if grip_status is not None:
            return grip_status

        if self.state.sim_jog_active:
            return "SIM JOG"

        if self.state.tracking_mode == "OBJECT":
            tracking = self.tracker.process(frame, self.segmenter, self.frame_index, self.state.approach_mode)
            return self.controller.update_from_object(self.state, tracking, width, height)
        return self.controller.update_from_hand(self.state, results)

    def _handle_grip_state(self) -> str | None:
        if not self.state.gripper_closed:
            return None
        if self.state.returning_home:
            return "OBJECT CAUGHT - RETURNING HOME"
        if not self.hardware.gripper_load_detected():
            return None

        print("Object caught. Returning home.")
        self.state.returning_home = True
        self.tracker.reset()
        self.state.approach_mode = False
        self.state.object_reached = False

        if not self.state.home:
            self.state.home = self.hardware.load_home(cfg.HOME_POSITION_PATH)

        home_ticks = home_ticks_to_state(self.state.home)
        self.state.target["base"] = home_ticks["base"]
        self.state.target["shoulder"] = home_ticks["shoulder"]
        self.state.target["elbow"] = home_ticks["elbow"]
        self.state.target["palm"] = home_ticks["palm"]
        self.state.target["wrist"] = cfg.GRIPPER_ROT_90_POS
        self.state.target["gripper"] = cfg.GRIPPER_CLOSE
        return "OBJECT CAUGHT - RETURNING HOME"

    def _update_sim(self) -> None:
        if self.sim is None:
            return
        self.sim.step_gui()
        if self.state.sim_jog_active:
            slider_ticks = self.sim.read_sliders_as_ticks()
            for name, tick in slider_ticks.items():
                self.state.target[name] = int(tick)
                if cfg.SIM_INSTANT_WHEN_JOG:
                    self.state.curr[name] = int(tick)
        else:
            hardware_ticks = self.hardware.read_ticks() if self.hardware.connected and not self.state.motors_enabled else None
            self.sim.set_visual_from_ticks(hardware_ticks or self.state.ticks())
            return
        self.sim.set_visual_from_ticks(self.state.ticks())

    def _update_motion(self) -> None:
        if self.state.sim_jog_active and cfg.SIM_INSTANT_WHEN_JOG:
            self.state.curr.update({k: int(v) for k, v in self.state.target.items()})
        elif self.state.motors_enabled:
            for name in ("base", "shoulder", "elbow", "palm", "wrist"):
                self.state.curr[name] = step_toward(self.state.curr[name], self.state.target[name], cfg.SPEED_LIMIT)
            self.state.curr["gripper"] = self.state.target["gripper"]
        else:
            return

        if self.state.motors_enabled and self.hardware.connected:
            self.hardware.write_ticks(self.state.curr)

    def _handle_key(self, key: int) -> None:
        if key == 255:
            return
        if key == ord("s"):
            self.state.motors_enabled = not self.state.motors_enabled
            if self.state.motors_enabled:
                ticks = self.hardware.read_ticks()
                if ticks:
                    self.state.set_curr_and_target(ticks)
            print(f"Motors {'enabled' if self.state.motors_enabled else 'disabled'}")
        elif key == ord("m"):
            self.state.tracking_mode = "OBJECT" if self.state.tracking_mode == "HAND" else "HAND"
            self.state.approach_mode = False
            self.state.returning_home = False
            self.tracker.reset()
            print(f"Mode: {self.state.tracking_mode}")
        elif key == ord("a"):
            if self.state.tracking_mode == "OBJECT" and self.tracker.active:
                self.state.approach_mode = not self.state.approach_mode
                self.state.object_reached = False
                print(f"Approach {'ON' if self.state.approach_mode else 'OFF'}")
        elif key == ord(" "):
            self.state.gripper_closed = not self.state.gripper_closed
            self.state.is_frozen = self.state.gripper_closed
            self.state.target["gripper"] = cfg.GRIPPER_CLOSE if self.state.gripper_closed else cfg.GRIPPER_OPEN
            if not self.state.gripper_closed:
                self.state.returning_home = False
            print("Gripper closing" if self.state.gripper_closed else "Gripper opening")
        elif key == ord("j"):
            if self.sim is None:
                print("PyBullet sim unavailable.")
                return
            self.state.sim_jog_active = not self.state.sim_jog_active
            if self.state.sim_jog_active:
                self.sim.recreate_sliders_from_ticks(self.state.ticks())
            print(f"Sim jog {'ON' if self.state.sim_jog_active else 'OFF'}")
        elif key == ord("r"):
            if self.sim is not None:
                self.sim.recreate_sliders_from_ticks(self.state.ticks())
                print("Sim sliders synced.")
        elif key == ord("u"):
            self._request_auto_target(cfg.DEFAULT_TARGET_CLASS)
        elif key == ord("t"):
            target = input(f"RF-DETR target class [{cfg.DEFAULT_TARGET_CLASS}]: ").strip()
            self._request_auto_target(target or cfg.DEFAULT_TARGET_CLASS)
        elif key == ord("z"):
            self.state.auto_palm = False
            self.state.target["palm"] = int(clamp(self.state.target["palm"] + 50, cfg.PALM_MIN, cfg.PALM_MAX))
        elif key == ord("x"):
            self.state.auto_palm = False
            self.state.target["palm"] = int(clamp(self.state.target["palm"] - 50, cfg.PALM_MIN, cfg.PALM_MAX))
        elif key == ord("c"):
            self.state.auto_palm = True

    def _request_auto_target(self, target_name: str) -> None:
        bbox = self.detector.select_bbox(self.last_frame_bgr, target_name)
        if bbox is None:
            return
        self.auto_target_name = target_name
        self.state.tracking_mode = "OBJECT"
        self.state.object_reached = False
        self.state.approach_mode = cfg.AUTO_APPROACH_AFTER_RFDETR
        self.tracker.request_bbox(bbox)
