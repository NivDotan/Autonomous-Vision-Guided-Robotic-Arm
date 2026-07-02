#!/usr/bin/env python3
"""Move the RoboNine SO-ARM101 Gazebo sim to a saved Gazebo joint pose."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HOME = REPO_ROOT / "StartHelloPos_handoff.json"
DEFAULT_CALIBRATION = REPO_ROOT / "robonine_tick_calibration.json"
DEFAULT_GAZEBO_POSE = REPO_ROOT / "robonine_gazebo_start_pos.json"

JOINTS = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
    "right_clamp",
    "left_clamp",
]

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


def ticks_to_rad(ticks: dict[int, int], cfg: dict) -> float:
    motor_id = int(cfg["motor_id"])
    tick_offset = float(cfg["tick_offset"])
    sign = float(cfg.get("sign", 1.0))
    joint_offset = float(cfg.get("joint_offset", 0.0))
    return (
        sign * (ticks.get(motor_id, int(tick_offset)) - tick_offset) * RAD_PER_TICK
        + joint_offset
    )


def gazebo_positions(pose_path: Path) -> list[float]:
    raw = json.loads(pose_path.read_text())
    return [
        clamp(float(raw[joint]), *LIMITS[joint])
        for joint in JOINTS
    ]


def tick_positions(home_path: Path, calibration_path: Path = DEFAULT_CALIBRATION) -> list[float]:
    raw = json.loads(home_path.read_text())
    ticks = {int(k): int(v) for k, v in raw.items()}
    calibration = json.loads(calibration_path.read_text())

    arm = [
        ticks_to_rad(ticks, calibration["base"]),
        ticks_to_rad(ticks, calibration["shoulder"]),
        ticks_to_rad(ticks, calibration["elbow"]),
        ticks_to_rad(ticks, calibration["palm"]),
        ticks_to_rad(ticks, calibration["wrist"]),
    ]

    # The physical gripper is one motor; the sim has two prismatic jaws.
    # Treat the home gripper offset as closed, and command both jaws closed.
    positions = arm + [0.0, 0.0]
    return [
        clamp(pos, *LIMITS[joint])
        for joint, pos in zip(JOINTS, positions)
    ]


class StartMover(Node):
    def __init__(self, positions: list[float], seconds: float) -> None:
        super().__init__("robonine_move_start")
        self._pub = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self._positions = positions
        self._seconds = seconds
        self._sent = False
        self.create_timer(0.5, self._send_once)

    def _send_once(self) -> None:
        if self._sent:
            return
        if self._pub.get_subscription_count() == 0:
            self.get_logger().info("waiting for /arm_controller/joint_trajectory subscriber...")
            return

        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in self._positions]
        point.time_from_start.sec = int(self._seconds)
        point.time_from_start.nanosec = int((self._seconds % 1.0) * 1e9)
        msg.points = [point]

        self._pub.publish(msg)
        self.get_logger().info(
            "sent start_pos: "
            + ", ".join(f"{j}={p:.4f}" for j, p in zip(JOINTS, self._positions))
        )
        self._sent = True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", type=Path, default=DEFAULT_GAZEBO_POSE)
    parser.add_argument("--home", type=Path)
    parser.add_argument("--calibration", type=Path, default=DEFAULT_CALIBRATION)
    parser.add_argument("--time", type=float, default=8.0)
    args = parser.parse_args()

    if args.home is None:
        positions = gazebo_positions(args.pose)
    else:
        positions = tick_positions(args.home, args.calibration)

    rclpy.init()
    node = StartMover(positions, args.time)
    try:
        deadline = node.get_clock().now().nanoseconds + int((args.time + 3.0) * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
