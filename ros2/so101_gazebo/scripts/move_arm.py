#!/usr/bin/env python3
"""
Move the SO-101 arm in Gazebo (or any ros2_control setup using arm_controller).

Publishes a JointTrajectory to /arm_controller/joint_trajectory.

Usage:
  # six joint angles in radians: shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper
  python3 move_arm.py 0.5 -0.3 0.4 0.0 0.0 0.0
  python3 move_arm.py 0.5 -0.3 0.4 0.0 0.0 0.0 --time 3.0

  # named poses
  python3 move_arm.py --pose home
  python3 move_arm.py --pose ready
  python3 move_arm.py --pose wave

Requires ROS2 sourced and the Gazebo sim running:
  ros2 launch so101_gazebo gazebo.launch.py
"""
import argparse
import json
import math
import os
import sys

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# ── Real start position (StartHelloPos_handoff.json) in TICKS, by motor id ────
# 1=base 2=shoulder 3=elbow 4=palm 5=wrist 6=gripper  → official joint order above
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
_HOME_FILE = os.path.join(_REPO, "StartHelloPos_handoff.json")
_TICKS_CENTER = 2048
_RAD_PER_TICK = 2.0 * math.pi / 4096.0   # ≈0.001534


def _start_pose():
    """Read the real start pose (ticks) and convert to radians for the sim.
    NOTE: sim joint zero/sign may differ from the real arm, so this is an
    approximation — tune signs here if a joint goes the wrong way."""
    ticks = {int(k): int(v) for k, v in json.load(open(_HOME_FILE)).items()}
    return [(ticks.get(i, 2048) - _TICKS_CENTER) * _RAD_PER_TICK for i in (1, 2, 3, 4, 5, 6)]


POSES = {
    "ready": [0.0, -0.6,  0.8,  0.0, 0.0, 0.0],
    "wave":  [0.6, -0.3,  0.5,  0.0, 0.0, 0.0],
    "open":  [0.0, -0.6,  0.8,  0.0, 0.0, 0.8],   # gripper open
    "close": [0.0, -0.6,  0.8,  0.0, 0.0, 0.0],   # gripper closed
    "zero":  [0.0,  0.0,  0.0,  0.0, 0.0, 0.0],   # URDF straight-up
}
POSES["start"] = _start_pose()   # your real StartHelloPos, converted to radians
POSES["home"]  = POSES["start"]  # 'home' == your real start position


class ArmMover(Node):
    def __init__(self, positions, seconds):
        super().__init__("so101_move_arm")
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.positions = positions
        self.seconds = seconds
        # wait briefly for the publisher to connect to the controller
        self.create_timer(0.5, self._send_once)
        self._sent = False

    def _send_once(self):
        if self._sent:
            return
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in self.positions]
        pt.time_from_start.sec = int(self.seconds)
        pt.time_from_start.nanosec = int((self.seconds % 1) * 1e9)
        msg.points = [pt]
        self.pub.publish(msg)
        self.get_logger().info(f"sent {dict(zip(JOINTS, self.positions))} over {self.seconds}s")
        self._sent = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("angles", nargs="*", type=float,
                    help="6 joint angles (rad): shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper")
    ap.add_argument("--pose", choices=sorted(POSES), help="use a named pose instead of angles")
    ap.add_argument("--time", type=float, default=2.0, help="seconds to reach the target")
    args = ap.parse_args()

    if args.pose:
        positions = POSES[args.pose]
    elif len(args.angles) == 6:
        positions = args.angles
    else:
        ap.error("give exactly 6 angles, or use --pose NAME")
        return

    rclpy.init()
    node = ArmMover(positions, args.time)
    try:
        # spin long enough to publish + let the message be delivered
        end = node.get_clock().now().nanoseconds + int((args.time + 1.5) * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < end:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
