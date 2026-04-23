"""
ROS2 hardware node — wraps FeetechHardware (or DaemonHardware) as a ROS2 node.

Publishes:  /arm/joint_states (sensor_msgs/JointState) @ 20 Hz
Subscribes: /arm/joint_command (sensor_msgs/JointState) → write_ticks()
"""
import sys
import os

# Allow importing robot_sam2_app without installing it.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..', '..', 'robot_sam2_app'))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_sam2_app.hardware import make_hardware
from robot_sam2_app.config import MOTOR_NAMES, USE_MOTOR_DAEMON


class HardwareNode(Node):
    def __init__(self):
        super().__init__("hardware_node")
        self.declare_parameter("use_daemon", USE_MOTOR_DAEMON)

        use_daemon = self.get_parameter("use_daemon").get_parameter_value().bool_value
        self.hw = make_hardware(use_daemon=use_daemon)
        connected = self.hw.connect()
        if connected:
            self.get_logger().info(f"Hardware connected (daemon={use_daemon})")
        else:
            self.get_logger().warn("Hardware not available — running in simulation-only mode")

        self._state_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
        self._cmd_sub   = self.create_subscription(
            JointState, "/arm/joint_command", self._cmd_cb, 10)
        self.create_timer(0.05, self._publish_state)   # 20 Hz

    def _cmd_cb(self, msg: JointState) -> None:
        ticks = {name: int(pos) for name, pos in zip(msg.name, msg.position)}
        self.hw.write_ticks(ticks)

    def _publish_state(self) -> None:
        ticks = self.hw.read_ticks()
        if ticks is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(MOTOR_NAMES)
        msg.position = [float(ticks.get(n, 0)) for n in MOTOR_NAMES]
        self._state_pub.publish(msg)

    def destroy_node(self):
        self.hw.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
