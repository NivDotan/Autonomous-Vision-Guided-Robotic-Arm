"""
ROS2 simulation node — keeps PyBullet visual mirror in sync with joint states.

Subscribes: /arm/joint_states (sensor_msgs/JointState)
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', '..', '..', '..', '..', '..', 'robot_sam2_app'))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from robot_sam2_app.simulation import PyBulletArmSim
from robot_sam2_app.config import SIM_CALIBRATION_PATH, MOTOR_NAMES


class SimNode(Node):
    def __init__(self):
        super().__init__("sim_node")
        self.sim = PyBulletArmSim(calibration_path=str(SIM_CALIBRATION_PATH))
        connected = self.sim.connect()
        if connected:
            self.get_logger().info("PyBullet sim connected.")
        else:
            self.get_logger().warn("PyBullet unavailable — sim_node will idle.")

        self._state_sub = self.create_subscription(
            JointState, "/arm/joint_states", self._state_cb, 10)
        self.create_timer(1.0 / 30.0, self._step)   # 30 Hz GUI step

    def _state_cb(self, msg: JointState) -> None:
        ticks = {name: int(pos) for name, pos in zip(msg.name, msg.position)}
        if self.sim.connected:
            self.sim.set_visual_from_ticks(ticks)

    def _step(self) -> None:
        if self.sim.connected:
            self.sim.step_gui()


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
