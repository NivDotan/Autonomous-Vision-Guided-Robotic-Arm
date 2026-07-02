#!/usr/bin/env python3
"""Publish a saved pose as /joint_states for RViz."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POSE = REPO_ROOT / "StartHelloPos_handoff.json"
DEFAULT_GAZEBO_POSE = REPO_ROOT / "robonine_gazebo_start_pos.json"
DEFAULT_CALIBRATION = REPO_ROOT / "robonine_tick_calibration.json"

MOTOR_IDS = {
    "base": 1,
    "shoulder": 2,
    "elbow": 3,
    "palm": 4,
    "wrist": 5,
    "gripper": 6,
}

PROJECT_JOINT_NAMES = ["base", "shoulder", "elbow", "palm", "wrist", "gripper"]
ROBONINE_JOINT_NAMES = [
    "base_link_to_link1",
    "link1_to_link2",
    "link2_to_link3",
    "link3_to_link4",
    "link4_to_link5",
    "right_clamp",
    "left_clamp",
]
TICK_CENTER = 2048
RAD_PER_TICK = 2.0 * math.pi / 4096.0


def _ticks(path: Path) -> dict[int, int]:
    raw = json.loads(path.read_text())
    return {int(k): int(v) for k, v in raw.items()}


def _tick_rad(ticks: dict[int, int], logical: str) -> float:
    return (ticks.get(MOTOR_IDS[logical], TICK_CENTER) - TICK_CENTER) * RAD_PER_TICK


def load_project_pose(path: Path) -> tuple[list[str], list[float]]:
    ticks = _ticks(path)
    return PROJECT_JOINT_NAMES, [_tick_rad(ticks, name) for name in PROJECT_JOINT_NAMES]


def load_robonine_pose(path: Path) -> tuple[list[str], list[float]]:
    raw = json.loads(path.read_text())
    if all(joint in raw for joint in ROBONINE_JOINT_NAMES):
        return ROBONINE_JOINT_NAMES, [float(raw[joint]) for joint in ROBONINE_JOINT_NAMES]

    ticks = _ticks(path)
    if DEFAULT_CALIBRATION.exists():
        calibration = json.loads(DEFAULT_CALIBRATION.read_text())
        names = [
            calibration[key]["joint"]
            for key in ("base", "shoulder", "elbow", "palm", "wrist")
        ]
        positions = [
            float(calibration[key].get("sign", 1.0))
            * (ticks.get(int(calibration[key]["motor_id"]), int(calibration[key]["tick_offset"]))
               - float(calibration[key]["tick_offset"]))
            * RAD_PER_TICK
            + float(calibration[key].get("joint_offset", 0.0))
            for key in ("base", "shoulder", "elbow", "palm", "wrist")
        ]
        return names + ["right_clamp", "left_clamp"], positions + [0.0, 0.0]

    arm = [
        _tick_rad(ticks, "base"),
        _tick_rad(ticks, "shoulder"),
        _tick_rad(ticks, "elbow"),
        _tick_rad(ticks, "palm"),
        _tick_rad(ticks, "wrist"),
    ]
    # RViz can display the raw tick pose even when Gazebo later clamps for physics.
    return ROBONINE_JOINT_NAMES, arm + [0.0, 0.0]


class StartPosePublisher(Node):
    def __init__(self, pose_path: Path, model: str) -> None:
        super().__init__("publish_start_pose_joint_states")
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        if model == "project":
            self._joint_names, self._positions = load_project_pose(pose_path)
        else:
            self._joint_names, self._positions = load_robonine_pose(pose_path)
        self.get_logger().info(
            "publishing "
            + str(dict(zip(self._joint_names, [round(p, 4) for p in self._positions])))
        )
        self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self._joint_names
        msg.position = self._positions
        self._pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pose", type=Path, default=DEFAULT_POSE)
    parser.add_argument("--model", choices=["robonine", "project"], default="robonine")
    args = parser.parse_args()

    rclpy.init()
    node = StartPosePublisher(args.pose, args.model)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
