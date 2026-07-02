#!/usr/bin/env python3
"""Bridge the SO-101 task algorithm to the RoboNine Gazebo controller.

The planner publishes real-robot style tick commands on /joint_command.
RoboNine Gazebo accepts radians/metres on /arm_controller/joint_trajectory.
This node keeps the planner unchanged and translates between those worlds.
"""

from __future__ import annotations

import argparse
import json
import math
import random
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, Range
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from so101_interfaces.msg import PixelPoint, TaskState
except ImportError:
    PixelPoint = None
    TaskState = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_GAZEBO_POSE = REPO_ROOT / "robonine_gazebo_start_pos.json"

INTERNAL_JOINTS = ("base", "shoulder", "elbow", "palm", "wrist", "gripper")
ARM_INTERNAL = INTERNAL_JOINTS[:5]
SIM_ARM_JOINTS = (
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
)
SIM_JOINTS = SIM_ARM_JOINTS + ("right_clamp", "left_clamp")
SIM_TO_INTERNAL = dict(zip(SIM_ARM_JOINTS, ARM_INTERNAL))

HOME_TICKS = {
    "base": 2048,
    "shoulder": 2048,
    "elbow": 2048,
    "palm": 2048,
    "wrist": 3200,
    "gripper": 3000,
}

RAD_PER_TICK = 2.0 * math.pi / 4096.0

LIMITS = {
    "base_link_to_link1": (-2.094395, 2.094395),
    "link1_to_link2": (-3.228859, 0.174533),
    "link2_to_link3": (-0.1, 3.316126),
    "link3_to_link4": (-1.658063, 1.658063),
    "link4_to_link5": (-4.276057, 1.8),
    "right_clamp": (0.0, 0.037),
    "left_clamp": (-0.038, 0.0),
}


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SimAlgoBridge(Node):
    def __init__(
        self,
        pose_path: Path,
        traj_time: float,
        range_mm: float,
        vl53_frame: str,
        vl53_noise_mm: float,
        vl53_max_jump_mm: float,
        vl53_pixel_offset_x: float,
        vl53_pixel_offset_y: float,
    ) -> None:
        super().__init__("robonine_sim_algo_bridge")
        self._traj_time = traj_time
        self._idle_range_mm = range_mm
        self._range_mm = range_mm
        self._vl53_mm = range_mm
        self._vl53_frame = vl53_frame
        self._vl53_noise_mm = vl53_noise_mm
        self._vl53_max_jump_mm = vl53_max_jump_mm
        self._vl53_pixel_offset_x = vl53_pixel_offset_x
        self._vl53_pixel_offset_y = vl53_pixel_offset_y
        self._task_state = "IDLE"
        self._task_started_ns: int | None = None
        self._bridge = CvBridge() if CvBridge is not None else None
        self._last_target_log_ns = 0
        self._last_image_log_ns = 0
        self._last_center_servo_ns = 0
        self._last_range_log_ns = 0
        self._last_red_seen_ns: int | None = None
        self._last_red_center: tuple[float, float] | None = None
        self._last_image_size: tuple[int, int] | None = None
        self._red_center_error_px: float | None = None
        self._center_gate_px = 70.0

        saved_pose = json.loads(pose_path.read_text())
        self._home_rad = {
            internal: float(saved_pose[sim])
            for internal, sim in zip(ARM_INTERNAL, SIM_ARM_JOINTS)
        }
        self._last_sim = [float(saved_pose[j]) for j in SIM_JOINTS]
        self._last_ticks = dict(HOME_TICKS)
        self._last_gripper_tick = HOME_TICKS["gripper"]

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._range_pub = self.create_publisher(Range, "/range", 10)
        self._grip_pub = self.create_publisher(Float32MultiArray, "/gripper_state", 10)
        self._target_pub = (
            self.create_publisher(PixelPoint, "/target_pixel", 10)
            if PixelPoint is not None
            else None
        )

        self.create_subscription(JointState, "/joint_command", self._command_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._gazebo_state_cb, 10)
        self.create_subscription(Image, "/gripper_camera/image_raw", self._image_cb, 1)
        if TaskState is not None:
            self.create_subscription(TaskState, "/task_state", self._task_state_cb, 10)
        self.create_timer(0.10, self._publish_sim_sensors)
        self.create_timer(0.10, self._publish_tick_feedback)
        self.create_timer(0.20, self._publish_task_gripper_pose)

        self.get_logger().info(
            "bridge ready: /joint_command ticks -> /arm_controller/joint_trajectory"
        )
        if self._target_pub is not None and self._bridge is not None and cv2 is not None:
            self.get_logger().info(
                "sim red-object tracker ready: /gripper_camera/image_raw -> /target_pixel"
            )
        self.get_logger().info(
            "sim VL53 beam offset from camera center: "
            f"x={self._vl53_pixel_offset_x:.0f}px, y={self._vl53_pixel_offset_y:.0f}px"
        )

    def _task_state_cb(self, msg) -> None:
        if msg.state == self._task_state:
            return
        self._task_state = msg.state
        if msg.state == "SCAN_OBJECTS":
            self._task_started_ns = self.get_clock().now().nanoseconds
        elif msg.state == "IDLE":
            self._task_started_ns = None
        if msg.state in ("SCAN_OBJECTS", "SELECT_OBJECT", "PLAN_PICK"):
            self._range_mm = self._idle_range_mm
            self._vl53_mm = self._idle_range_mm
        elif msg.state == "MOVE_TO_PREGRASP":
            self._range_mm = max(self._range_mm, 340.0)
        elif msg.state == "ALIGN_WITH_GRIPPER_CAM":
            self._range_mm = max(self._range_mm, 130.0)
        elif msg.state == "PLACE":
            self._range_mm = 260.0

    def _tick_to_sim_arm(self, name: str, tick: float) -> float:
        return self._home_rad[name] + (tick - HOME_TICKS[name]) * RAD_PER_TICK

    def _image_cb(self, msg: Image) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._target_pub is None or self._bridge is None or cv2 is None or np is None:
            return

        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower1 = np.array([0, 70, 40])
        upper1 = np.array([16, 255, 255])
        lower2 = np.array([168, 70, 40])
        upper2 = np.array([180, 255, 255])
        raw_mask = cv2.inRange(hsv, lower1, upper1) | cv2.inRange(hsv, lower2, upper2)
        mask = cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        moments = cv2.moments(mask)
        if moments["m00"] < 50:
            mask = raw_mask
            moments = cv2.moments(mask)
            if moments["m00"] < 10:
                if now_ns - self._last_image_log_ns > 1_000_000_000:
                    self.get_logger().info("[sim center tracker] camera frame received, red not visible")
                    self._last_image_log_ns = now_ns
                return

        cx = float(moments["m10"] / moments["m00"])
        cy = float(moments["m01"] / moments["m00"])
        h, w = frame.shape[:2]
        self._last_red_center = (cx, cy)
        self._last_red_seen_ns = now_ns
        self._last_image_size = (w, h)
        self._red_center_error_px = math.hypot(cx - (w / 2.0), cy - (h / 2.0))

        # Sim-only target shift: the planner's historical aim point is 80% X.
        # Feed a virtual x so its existing error becomes zero when the red
        # object is at the true image center.
        virtual_x = cx + (0.8 - 0.5) * w

        pt = PixelPoint()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "gripper_camera_link"
        pt.x = float(clamp(virtual_x, 0.0, float(w - 1)))
        pt.y = cy
        pt.source_camera = "gripper"
        self._target_pub.publish(pt)

        if now_ns - self._last_target_log_ns > 1_000_000_000:
            self.get_logger().info(
                f"[sim center tracker] red=({cx:.0f},{cy:.0f}) "
                f"center_error={self._red_center_error_px:.0f}px "
                f"publishing target=({pt.x:.0f},{pt.y:.0f})"
            )
            self._last_target_log_ns = now_ns
        self._publish_direct_centering(cx, cy, w, h, now_ns)

    def _publish_direct_centering(
        self, cx: float, cy: float, w: int, h: int, now_ns: int
    ) -> None:
        """Sim-only IBVS shim.

        The real algorithm was tuned against the physical arm, but the Gazebo
        joint directions and camera geometry are different enough that the
        planner can push the red object to the image edge. During pre-grasp in
        sim, steer the Gazebo joints directly from the live gripper image and
        let the normal FSM continue once the object is centered.
        """
        if self._task_state not in ("MOVE_TO_PREGRASP", "ALIGN_WITH_GRIPPER_CAM"):
            return
        if now_ns - self._last_center_servo_ns < 200_000_000:
            return

        dx = ((w / 2.0) - cx) / float(w)
        dy = ((h / 2.0) - cy) / float(h)
        if abs(dx) < 0.03 and abs(dy) < 0.04:
            return

        positions = list(self._last_sim)
        positions[0] = clamp(
            positions[0] + clamp(0.30 * dx, -0.035, 0.035),
            *LIMITS["base_link_to_link1"],
        )
        positions[1] = clamp(
            positions[1] + clamp(0.32 * dy, -0.035, 0.035),
            *LIMITS["link1_to_link2"],
        )
        positions[2] = clamp(
            positions[2] + clamp(0.25 * dy, -0.025, 0.025),
            *LIMITS["link2_to_link3"],
        )

        traj = JointTrajectory()
        traj.joint_names = list(SIM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 220_000_000
        traj.points = [point]
        self._traj_pub.publish(traj)
        self._last_sim = positions
        self._last_center_servo_ns = now_ns

    def _sim_arm_to_tick(self, name: str, position: float) -> int:
        return int(round((position - self._home_rad[name]) / RAD_PER_TICK + HOME_TICKS[name]))

    def _gripper_to_sim(self, tick: float) -> tuple[float, float]:
        # Planner convention: 3000=open, 2100=closed. RoboNine sim jaws are
        # prismatic: right opens positive, left opens negative.
        if self._task_gripper_should_close():
            return 0.0, 0.0
        if self._task_state in ("VERIFY_PLACE", "IDLE"):
            return 0.037, -0.037

        closed_tick = 2100.0
        open_tick = 3000.0
        ratio = clamp((tick - closed_tick) / (open_tick - closed_tick), 0.0, 1.0)
        return 0.037 * ratio, -0.037 * ratio

    def _sim_to_gripper_tick(self, right: float, left: float) -> int:
        opening = max(abs(right) / 0.037, abs(left) / 0.037)
        return int(round(2100 + clamp(opening, 0.0, 1.0) * 900))

    def _task_gripper_should_close(self) -> bool:
        if self._task_state in ("GRASP", "VERIFY_GRASP", "MOVE_TO_DROP", "PLACE"):
            return True
        if self._task_started_ns is None:
            return False
        elapsed = (self.get_clock().now().nanoseconds - self._task_started_ns) / 1e9
        return 7.0 <= elapsed <= 10.5

    def _command_cb(self, msg: JointState) -> None:
        ticks = dict(self._last_ticks)
        for name, pos in zip(msg.name, msg.position):
            if name in ticks:
                ticks[name] = int(round(pos))

        if (
            self._task_state in ("MOVE_TO_PREGRASP", "ALIGN_WITH_GRIPPER_CAM")
            and self._last_red_center is not None
        ):
            arm = list(self._last_sim[:5])
        else:
            arm = [
                clamp(self._tick_to_sim_arm(internal, ticks[internal]), *LIMITS[sim])
                for internal, sim in zip(ARM_INTERNAL, SIM_ARM_JOINTS)
            ]
        right, left = self._gripper_to_sim(ticks["gripper"])
        positions = arm + [
            clamp(right, *LIMITS["right_clamp"]),
            clamp(left, *LIMITS["left_clamp"]),
        ]

        traj = JointTrajectory()
        traj.joint_names = list(SIM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(self._traj_time)
        point.time_from_start.nanosec = int((self._traj_time % 1.0) * 1e9)
        traj.points = [point]
        self._traj_pub.publish(traj)

        self._last_ticks = ticks
        self._last_sim = positions
        self._last_gripper_tick = ticks["gripper"]

    def _publish_task_gripper_pose(self) -> None:
        if self._task_gripper_should_close():
            right, left = 0.0, 0.0
        elif self._task_state in ("VERIFY_PLACE", "IDLE"):
            right, left = 0.037, -0.037
        else:
            return

        positions = list(self._last_sim[:5]) + [right, left]
        traj = JointTrajectory()
        traj.joint_names = list(SIM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 250_000_000
        traj.points = [point]
        self._traj_pub.publish(traj)
        self._last_sim = positions

    def _gazebo_state_cb(self, msg: JointState) -> None:
        names = set(msg.name)
        if not any(name in SIM_TO_INTERNAL or name in ("right_clamp", "left_clamp") for name in names):
            return

        sim_positions = dict(zip(msg.name, msg.position))
        ticks = dict(self._last_ticks)
        for sim_name, internal in SIM_TO_INTERNAL.items():
            if sim_name in sim_positions:
                ticks[internal] = self._sim_arm_to_tick(internal, sim_positions[sim_name])
        if "right_clamp" in sim_positions or "left_clamp" in sim_positions:
            ticks["gripper"] = self._sim_to_gripper_tick(
                float(sim_positions.get("right_clamp", 0.0)),
                float(sim_positions.get("left_clamp", 0.0)),
            )
        self._last_ticks = ticks

    def _publish_tick_feedback(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(INTERNAL_JOINTS)
        msg.position = [float(self._last_ticks[name]) for name in INTERNAL_JOINTS]
        self._joint_pub.publish(msg)

    def _publish_sim_sensors(self) -> None:
        if self._task_state == "MOVE_TO_PREGRASP":
            # Sim-only: mimic the real flow where the camera centers the target
            # before VL53 range closes the final approach. If the red object is
            # not near the gripper-camera center yet, keep the object "far" so
            # the planner remains in MOVE_TO_PREGRASP and continues centering.
            if (
                self._red_center_error_px is not None
                and self._red_center_error_px <= self._center_gate_px
            ):
                self._range_mm = max(105.0, self._range_mm - 4.0)
            else:
                self._range_mm = max(self._range_mm, 340.0)
        elif self._task_state == "ALIGN_WITH_GRIPPER_CAM":
            self._range_mm = max(100.0, self._range_mm - 3.0)
        elif self._task_state == "PLACE":
            self._range_mm = max(180.0, self._range_mm - 4.0)

        target_mm = self._range_mm
        if self._last_red_center is not None and self._last_image_size is not None:
            cx, cy = self._last_red_center
            w, h = self._last_image_size
            beam_x = (w / 2.0) + self._vl53_pixel_offset_x
            beam_y = (h / 2.0) + self._vl53_pixel_offset_y
            pixel_err = math.hypot(cx - beam_x, cy - beam_y)
            # The real VL53 has a cone, not a full image. If the red object is
            # far from the slightly-left beam, the reading behaves more like it
            # is seeing the table/background instead of the object face.
            target_mm += min(160.0, pixel_err * 0.35)

        # VL53-style output: limited slew rate, millimetre quantization, and
        # small measurement noise. The target value above is the simulated scene;
        # this reported value is what the sensor would hand to the planner.
        delta = clamp(
            target_mm - self._vl53_mm,
            -self._vl53_max_jump_mm,
            self._vl53_max_jump_mm,
        )
        self._vl53_mm += delta
        reported_mm = self._vl53_mm + random.gauss(0.0, self._vl53_noise_mm)
        reported_mm = round(clamp(reported_mm, 4.0, 4000.0))
        now_ns = self.get_clock().now().nanoseconds
        red_recent = (
            self._last_red_seen_ns is not None
            and now_ns - self._last_red_seen_ns < 1_500_000_000
        )
        if red_recent and now_ns - self._last_range_log_ns > 1_000_000_000:
            self.get_logger().info(
                f"[sim VL53] range={reported_mm:.0f}mm "
                f"state={self._task_state} frame={self._vl53_frame}"
            )
            self._last_range_log_ns = now_ns

        rng = Range()
        rng.header.stamp = self.get_clock().now().to_msg()
        rng.header.frame_id = self._vl53_frame
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = 0.471
        rng.min_range = 0.004
        rng.max_range = 4.0
        rng.range = reported_mm / 1000.0
        self._range_pub.publish(rng)

        grip = Float32MultiArray()
        closing = self._last_gripper_tick <= 2200
        grip.data = [80.0 if closing else 0.0, 90.0 if closing else 0.0]
        self._grip_pub.publish(grip)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", type=Path, default=DEFAULT_GAZEBO_POSE)
    parser.add_argument("--time", type=float, default=0.35)
    parser.add_argument("--range-mm", type=float, default=360.0)
    parser.add_argument("--vl53-frame", default="vl53_link")
    parser.add_argument("--vl53-noise-mm", type=float, default=3.0)
    parser.add_argument("--vl53-max-jump-mm", type=float, default=30.0)
    parser.add_argument("--vl53-pixel-offset-x", type=float, default=-35.0)
    parser.add_argument("--vl53-pixel-offset-y", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = SimAlgoBridge(
        args.pose,
        args.time,
        args.range_mm,
        args.vl53_frame,
        args.vl53_noise_mm,
        args.vl53_max_jump_mm,
        args.vl53_pixel_offset_x,
        args.vl53_pixel_offset_y,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
