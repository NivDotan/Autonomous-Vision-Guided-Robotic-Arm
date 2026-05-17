from __future__ import annotations

import cv2
import mediapipe as mp
import time

from . import config as cfg
from .control import MotionController, draw_overlay
from .hardware import make_hardware, home_ticks_to_state
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
        self.hardware = make_hardware(use_daemon=cfg.USE_MOTOR_DAEMON)
        self.sim: PyBulletArmSim | None = None
        self.segmenter = SAM2Segmenter()
        self.detector = RFDETRTargetSelector()
        self.tracker = ObjectTracker()
        self.controller = MotionController()
        self.mp_hands = mp.solutions.hands
        self.cap = cv2.VideoCapture(cfg.CAMERA_INDEX)
        self.frame_index = 0
        self.last_frame_bgr = None
        self.auto_target_name = cfg.DEFAULT_TARGET_CLASS
        self.free_mode = False
        from .tracking import TrackingResult
        self._last_tracking = TrackingResult(False)

        from .data_logger import DataLogger
        self.logger = DataLogger(cfg.PROJECT_ROOT)

        # ── Trajectory planning (Tier 1) ─────────────────────────────────────
        from .trajectory.planner import TrajectoryPlanner
        self.traj_planner = TrajectoryPlanner(str(cfg.SIM_CALIBRATION_PATH))
        self.collision_checker = None   # set after sim init in _start_sim()

        # ── RealSense depth camera (Tier 1) ───────────────────────────────────
        self._depth_frame = None
        self._realsense_ok = False
        if cfg.REALSENSE_ENABLED:
            if cfg.MOCK_REALSENSE:
                from .vision.depth_perception import MockRealSenseDepth
                self.realsense = MockRealSenseDepth()
            else:
                from .vision.depth_perception import RealSenseDepth
                self.realsense = RealSenseDepth()
        else:
            self.realsense = None

        # ── 3D grasp planner (Tier 1) ─────────────────────────────────────────
        self.grasp_planner = None  # init after realsense connects

        # ── VL53L1X distance sensor ───────────────────────────────────────────
        self.vl53 = None
        if cfg.VL53_ENABLED:
            from .vl53_sensor import VL53Sensor
            self.vl53 = VL53Sensor(cfg.VL53_PORT, cfg.VL53_BAUD)

    def setup(self) -> None:
        if self.vl53 is not None:
            self.vl53.connect()
        self.hardware.connect()
        self.state.home = self.hardware.load_home(cfg.HOME_POSITION_PATH)
        if self.state.home:
            self.state.set_curr_and_target(home_ticks_to_state(self.state.home))
            from .go_home_util import go_home
            go_home(self.hardware, self.state.home)
        self._start_sim()
        self._start_realsense()

    def _start_sim(self) -> None:
        try:
            self.sim = PyBulletArmSim(cfg.SIM_CALIBRATION_PATH)
            self.sim.connect()
            self.sim.recreate_sliders_from_ticks(self.state.ticks())
            from .trajectory.collision_check import CollisionChecker
            self.collision_checker = CollisionChecker(self.sim)
            print("PyBullet sim ready. Press J for sim jog, R to resync sliders.")
        except Exception as exc:
            self.sim = None
            print(f"PyBullet sim not started: {exc}")

    def _start_realsense(self) -> None:
        if self.realsense is None:
            return
        self._realsense_ok = self.realsense.connect()
        if self._realsense_ok:
            from .vision.grasp_planner import GraspPlanner
            from .vision.scene_3d import CoordTransform
            tf = CoordTransform(cfg.HAND_EYE_CALIB_PATH)
            self.grasp_planner = GraspPlanner(self.realsense, tf)
            print("RealSense depth camera connected. 3D grasping enabled — press G to grasp.")
        else:
            print("RealSense not available — 3D grasping disabled.")

    def run(self) -> None:
        self.setup()
        if not self.cap.isOpened():
            print(f"ERROR: Could not open camera index {cfg.CAMERA_INDEX}. "
                  f"Change CAMERA_INDEX in config.py.")
            self.close()
            return
        print("Controls: S motors, M mode, A approach, Space grab/release, U auto cup, T typed target")
        print("          R reset home, L log start/stop, F free-arm mode, Z/X manual palm, C auto palm, Q quit")
        cv2.namedWindow("Robot Brain")
        cv2.setMouseCallback("Robot Brain", self._on_mouse)

        with self.mp_hands.Hands(min_detection_confidence=0.6) as hands:
            while self.cap.isOpened():
                # Camera capture — RealSense or plain webcam.
                if self._realsense_ok:
                    rs_result = self.realsense.read_aligned()
                    if rs_result is None:
                        continue
                    frame, self._depth_frame = rs_result
                    frame = cv2.flip(frame, 1)
                    self._depth_frame = cv2.flip(self._depth_frame, 1)
                else:
                    ok, frame = self.cap.read()
                    if not ok:
                        break
                    frame = cv2.flip(frame, 1)
                    self._depth_frame = None

                self.last_frame_bgr = frame.copy()
                h, w, _ = frame.shape
                try:
                    results = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                except Exception:
                    results = None

                status = self._update_vision(results, frame, w, h)
                self._update_sim()
                dist = self.vl53.distance_mm if self.vl53 is not None else None
                self.state.vl53_dist_mm = dist
                self._check_vl53(dist)
                if self.state.retreat_mode:
                    status = self._update_retreat()
                elif not self.free_mode:
                    self._update_motion()
                else:
                    # Free mode: read actual positions and echo them straight back as
                    # goal positions — motor error stays at zero so it barely resists
                    # manual movement (daemon owns the port, true torque-off not possible)
                    if self.hardware.connected:
                        ticks = self.hardware.read_ticks()
                        if ticks:
                            self.state.set_curr_and_target(ticks)
                            self.hardware.write_ticks(ticks)
                self.logger.record(self.state, self._last_tracking, w, h, self.free_mode)
                now = time.monotonic()
                if not hasattr(self, '_load_dbg_t') or now - self._load_dbg_t >= 2.0:
                    self._load_dbg_t = now
                    if hasattr(self.hardware, 'read_gripper_load_raw'):
                        print(f"[LOAD DBG] raw={self.hardware.read_gripper_load_raw()}")
                draw_overlay(frame, status, self.state, self.auto_target_name, dist)

                cv2.imshow("Robot Brain", frame)
                key = cv2.waitKey(5) & 0xFF
                if key == ord("q"):
                    self._go_home()
                    break
                self._handle_key(key)
                self.frame_index += 1

        self.close()

    def _check_vl53(self, dist_mm: int | None) -> None:
        self.state.vl53_controls_elbow = False

        # Lock base/shoulder when too close for SAM2 to be reliable
        self.state.arm_locked = (
            dist_mm is not None and
            dist_mm < cfg.VL53_LOCK_DIST_MM and
            self.state.approach_mode
        )

        # Phase 2 — palm adjusting, close gripper once it arrives
        if self.state.pre_grasp_palm:
            if abs(self.state.curr["palm"] - self.state.target["palm"]) <= cfg.SPEED_LIMIT * 2:
                self.state.pre_grasp_palm    = False
                self.state.gripper_closed    = True
                self.state.target["gripper"] = cfg.GRIPPER_CLOSE
                print("Palm ready — closing gripper")
            return

        if dist_mm is None or not self.state.approach_mode or self.state.gripper_closed:
            return
        if self.vl53 is not None and self.vl53.is_stable_and_close(cfg.VL53_GRIP_DIST_MM,
                                                                     cfg.VL53_STABLE_WINDOW_MM,
                                                                     cfg.VL53_MAX_JUMP_MM):
            self.state.approach_mode = False
            self.state.is_frozen     = True
            new_palm = int(clamp(self.state.curr["palm"] + cfg.VL53_PREGRASP_PALM_DELTA,
                                 cfg.PALM_MIN, cfg.PALM_MAX))
            if abs(new_palm - self.state.curr["palm"]) >= 10:
                print(f"VL53: {dist_mm} mm — moving palm {cfg.VL53_PREGRASP_PALM_DELTA:+d} then closing")
                self.state.pre_grasp_palm = True
                self.state.target["palm"] = new_palm
            else:
                print(f"VL53: {dist_mm} mm — palm at limit, closing directly")
                self.state.gripper_closed    = True
                self.state.target["gripper"] = cfg.GRIPPER_CLOSE

    def _go_home(self) -> None:
        if not self.state.home or not self.hardware.connected:
            return
        from .go_home_util import go_home
        self.state.gripper_closed = False
        go_home(self.hardware, self.state.home)

    def close(self) -> None:
        if self.sim is not None:
            self.sim.disconnect()
        self.hardware.disconnect()
        if self.realsense is not None:
            self.realsense.disconnect()
        if self.vl53 is not None:
            self.vl53.disconnect()
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
            self._last_tracking = tracking
            if tracking.success and self._depth_frame is not None and self.grasp_planner is not None:
                from .vision.grasp_planner import GraspPlanner
                mask = GraspPlanner.bbox_to_mask(
                    self.tracker.bbox, frame.shape)
                self.state.grasp_pose = self.grasp_planner.plan(mask, self._depth_frame)
            return self.controller.update_from_object(self.state, tracking, width, height)
        from .tracking import TrackingResult
        self._last_tracking = TrackingResult(False)
        return self.controller.update_from_hand(self.state, results)

    def _handle_grip_state(self) -> str | None:
        if not self.state.gripper_closed:
            # Local retry: waiting for adjustment to settle, then re-close
            if self.state.grip_local_retry:
                settled = all(
                    abs(self.state.curr[n] - self.state.target[n]) <= cfg.SPEED_LIMIT * 2
                    for n in ("shoulder", "elbow")
                )
                if settled:
                    self.state.grip_local_retry      = False
                    self.state.gripper_closed        = True
                    self.state.gripper_closed_frames = 0
                    self.state.target["gripper"]     = cfg.GRIPPER_CLOSE
                return f"ADJUSTING {'UP' if self.state.grip_local_attempt <= cfg.GRIP_UP_MAX_TRIES else 'CLOSER'} ({self.state.grip_local_attempt})"
            return None

        if self.state.returning_home:
            return "OBJECT CAUGHT - RETURNING HOME"

        self.state.gripper_closed_frames += 1

        if (self.state.gripper_closed_frames >= cfg.GRIP_LOAD_MIN_FRAMES and
                self.hardware.gripper_load_detected()):
            print("Object caught. Returning home.")
            self.state.returning_home    = True
            self.state.grip_local_attempt = 0
            self.tracker.reset()
            self.state.approach_mode  = False
            self.state.object_reached = False
            if not self.state.home:
                self.state.home = self.hardware.load_home(cfg.HOME_POSITION_PATH)
            home_ticks = home_ticks_to_state(self.state.home)
            self.state.target["base"]     = home_ticks["base"]
            self.state.target["shoulder"] = home_ticks["shoulder"]
            self.state.target["elbow"]    = home_ticks["elbow"]
            self.state.target["palm"]     = home_ticks["palm"]
            self.state.target["wrist"]    = cfg.GRIPPER_ROT_90_POS
            self.state.target["gripper"]  = cfg.GRIPPER_CLOSE
            return "OBJECT CAUGHT - RETURNING HOME"

        if self.state.gripper_closed_frames > cfg.GRIP_CHECK_FRAMES:
            print(f"[GRIP] frames={self.state.gripper_closed_frames} local_attempt={self.state.grip_local_attempt} max={cfg.GRIP_UP_MAX_TRIES}")
            if self.state.grip_local_attempt <= cfg.GRIP_UP_MAX_TRIES:
                self._start_local_retry()
                return "MISS — ADJUSTING"
            # All local retries exhausted → full retreat
            print("[GRIP] all local retries done — full retreat")
            self.state.grip_local_attempt = 0
            self._start_retreat()
            return "MISS — RETREATING"

        return f"GRIPPING... ({self.state.gripper_closed_frames}/{cfg.GRIP_CHECK_FRAMES})"

    def _start_local_retry(self) -> None:
        attempt = self.state.grip_local_attempt
        self.state.gripper_closed        = False
        self.state.gripper_closed_frames = 0
        self.state.target["gripper"]     = cfg.GRIPPER_OPEN
        self.state.grip_local_retry      = True
        self.state.grip_local_attempt    = attempt + 1

        if attempt < cfg.GRIP_UP_MAX_TRIES:
            self.state.target["shoulder"] = int(clamp(
                self.state.curr["shoulder"] + cfg.GRIP_UP_SHOULDER_DELTA,
                cfg.SH_MIN, cfg.SH_MAX))
            print(f"Grip miss — moving UP (try {attempt + 1}/{cfg.GRIP_UP_MAX_TRIES})")
        else:
            self.state.target["elbow"] = int(clamp(
                self.state.curr["elbow"] + cfg.GRIP_CLOSER_ELBOW_DELTA,
                cfg.EL_MIN, cfg.EL_MAX))
            print("Grip miss — moving CLOSER")

    def _start_retreat(self) -> None:
        attempt = self.state.grip_attempt + 1
        print(f"Grip miss #{attempt} — retreating to try new position")
        self.state.gripper_closed        = False
        self.state.is_frozen             = False
        self.state.approach_mode         = False
        self.state.retreat_mode          = True
        self.state.pre_grasp_palm        = False
        self.state.gripper_closed_frames = 0
        self.state.grip_attempt          = attempt
        self.state.target["gripper"]     = cfg.GRIPPER_OPEN

    def _update_retreat(self) -> str:
        """Step motors back to pre-approach position, then pick next aim cell."""
        pre = self.state.pre_approach_ticks
        if not pre:
            self.state.retreat_mode = False
            return "RETREAT ERROR"

        for name in ("base", "shoulder", "elbow", "palm"):
            self.state.target[name] = pre.get(name, self.state.curr[name])
        self._step_proportional()

        back = all(
            abs(self.state.curr[n] - pre.get(n, self.state.curr[n])) < cfg.RETREAT_TOLERANCE
            for n in ("shoulder", "elbow")
        )
        if back:
            self._pick_next_aim()
        return f"RETREATING (attempt {self.state.grip_attempt}/{len(cfg.RETRY_AIM_SEQUENCE)})"

    def _pick_next_aim(self) -> None:
        seq = cfg.RETRY_AIM_SEQUENCE
        attempt = self.state.grip_attempt
        if attempt >= len(seq):
            print("All positions tried — giving up.")
            self.state.retreat_mode = False
            return
        aim = seq[attempt]
        self.state.current_aim_x  = aim[0]
        self.state.current_aim_y  = aim[1]
        self.state.retreat_mode   = False
        self.state.approach_mode  = True
        self.state.object_reached = False
        print(f"Retry #{attempt + 1}: aiming at X={aim[0]:.2f} Y={aim[1]:.2f} in frame")

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
        if self.state.trajectory_active:
            self._step_trajectory()
        elif self.state.sim_jog_active and cfg.SIM_INSTANT_WHEN_JOG:
            self.state.curr.update({k: int(v) for k, v in self.state.target.items()})
            if self.state.motors_enabled and self.hardware.connected:
                self.hardware.write_ticks(self.state.curr)
        else:
            self._step_proportional()

    def _step_proportional(self) -> None:
        """Original proportional step — unchanged logic."""
        if not self.state.motors_enabled:
            return
        for name in ("base", "shoulder", "elbow", "palm", "wrist"):
            self.state.curr[name] = step_toward(
                self.state.curr[name], self.state.target[name], cfg.SPEED_LIMIT)
        self.state.curr["gripper"] = self.state.target["gripper"]
        if self.hardware.connected:
            self.hardware.write_ticks(self.state.curr)

    def _step_trajectory(self) -> None:
        """Advance one waypoint from the planned trajectory."""
        if self.state.trajectory_index >= len(self.state.trajectory_waypoints):
            self.state.trajectory_active = False
            return
        wp = self.state.trajectory_waypoints[self.state.trajectory_index]
        self.state.set_curr_and_target(wp.ticks)
        self.state.trajectory_index += 1
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
                if self.state.approach_mode:
                    self.state.pre_approach_ticks = dict(self.state.curr)
                    self.state.grip_attempt       = 0
                    self.state.grip_local_attempt = 0
                    self.state.grip_local_retry   = False
                    self.state.current_aim_x  = cfg.APPROACH_AIM_X
                    self.state.current_aim_y  = cfg.APPROACH_AIM_Y
                    self.state.retreat_mode   = False
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
        elif key == ord("l"):
            if self.logger.active:
                self.logger.stop()
            else:
                self.logger.start()
        elif key == ord("f"):
            self.free_mode = not self.free_mode
            if self.free_mode:
                self.state.motors_enabled = False
                self.hardware.set_torque(False, [2, 3, 4])
                if not self.logger.active:
                    self.logger.start()
                print("FREE MODE ON — goal=current each frame (low resistance). Move arm by hand. Logging started.")
            else:
                self.hardware.set_torque(True, [2, 3, 4])
                self.state.motors_enabled = False  # user must re-press S to re-enable
                print("FREE MODE OFF — motors re-engaged. Press S to enable control.")
        elif key == ord("r"):
            print("Resetting to start position...")
            self.state.approach_mode = False
            self.state.object_reached = False
            self.state.is_frozen = False
            self.state.gripper_closed = False
            self.state.returning_home = False
            self.state.pre_grasp_palm = False
            self.tracker.reset()
            self._go_home()
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
        elif key == ord("g"):
            self._request_3d_grasp()

    def _request_3d_grasp(self) -> None:
        """Plan and execute a 3D grasp trajectory from the current grasp pose."""
        if self.state.grasp_pose is None:
            print("No 3D grasp pose available. Track an object with RealSense first.")
            return
        wps = self.traj_planner.plan_grasp(
            self.state.ticks(), self.state.grasp_pose)
        if not wps:
            print("Grasp trajectory planning returned no waypoints.")
            return
        safe, idx = True, None
        if self.collision_checker is not None:
            safe, idx = self.collision_checker.check_trajectory(wps)
        if not safe:
            print(f"Grasp trajectory has collision at waypoint {idx} — aborted.")
            return
        self.state.trajectory_waypoints = wps
        self.state.trajectory_index     = 0
        self.state.trajectory_active    = True
        print(f"Grasp trajectory started: {len(wps)} waypoints, "
              f"pos={self.state.grasp_pose.position_base}")

    def _request_auto_target(self, target_name: str) -> None:
        bbox = self.detector.select_bbox(self.last_frame_bgr, target_name)
        if bbox is None:
            return
        self.auto_target_name = target_name
        self.state.tracking_mode = "OBJECT"
        self.state.object_reached = False
        self.state.approach_mode = cfg.AUTO_APPROACH_AFTER_RFDETR
        self.tracker.request_bbox(bbox)
