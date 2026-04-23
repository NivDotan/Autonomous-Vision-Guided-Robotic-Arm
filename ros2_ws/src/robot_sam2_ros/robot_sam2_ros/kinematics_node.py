"""
ROS2 kinematics node — provides the MoveToTarget action server.

Action server: /move_to_target (robot_sam2_ros/action/MoveToTarget)
  Goal:     GraspCommand (target position, approach axis, pre-grasp offset)
  Feedback: phase + progress
  Result:   success + final joint angles

Subscribes: /arm/joint_states  — current tick positions from hardware_node
Publishes:  /arm/joint_command — tick commands during trajectory execution
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', '..', '..', '..', '..', '..', 'robot_sam2_app'))

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from sensor_msgs.msg import JointState

from robot_sam2_ros.action import MoveToTarget  # type: ignore
from robot_sam2_app.config import MOTOR_NAMES, SIM_CALIBRATION_PATH
from robot_sam2_app.trajectory.planner import TrajectoryPlanner
from robot_sam2_app.vision.grasp_planner import GraspPose3D


class KinematicsNode(Node):
    def __init__(self):
        super().__init__("kinematics_node")
        self.planner = TrajectoryPlanner(str(SIM_CALIBRATION_PATH))

        self._current_ticks: dict[str, int] = {n: 2048 for n in MOTOR_NAMES}
        self._state_sub = self.create_subscription(
            JointState, "/arm/joint_states", self._state_cb, 10)
        self._cmd_pub = self.create_publisher(
            JointState, "/arm/joint_command", 10)

        self._action_server = ActionServer(
            self,
            MoveToTarget,
            "/move_to_target",
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info("KinematicsNode ready — MoveToTarget action server active.")

    def _state_cb(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            self._current_ticks[name] = int(pos)

    async def _execute_cb(self, goal_handle):
        grasp_cmd = goal_handle.request.grasp
        feedback   = MoveToTarget.Feedback()
        result     = MoveToTarget.Result()

        # Planning phase.
        feedback.phase = "PLANNING"
        feedback.progress = 0.0
        goal_handle.publish_feedback(feedback)

        grasp_pose = GraspPose3D(
            position_base=tuple(grasp_cmd.target_position),
            approach_axis=tuple(grasp_cmd.approach_axis),
            quality=1.0,
        )
        try:
            waypoints = self.planner.plan_grasp(
                self._current_ticks,
                grasp_pose,
                pre_grasp_offset=float(grasp_cmd.pre_grasp_offset_m),
            )
        except Exception as e:
            result.success = False
            result.message = f"Planning failed: {e}"
            goal_handle.abort()
            return result

        total = len(waypoints)
        if total == 0:
            result.success = False
            result.message = "No waypoints generated"
            goal_handle.abort()
            return result

        # Execution phase.
        feedback.phase = "MOVING"
        feedback.total_waypoints = total
        rate = self.create_rate(200)  # 200 Hz to match daemon

        for i, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Cancelled"
                return result

            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name     = list(MOTOR_NAMES)
            cmd.position = [float(wp.ticks.get(n, 2048)) for n in MOTOR_NAMES]
            self._cmd_pub.publish(cmd)

            feedback.waypoint_index = i
            feedback.progress = float(i) / total
            goal_handle.publish_feedback(feedback)
            rate.sleep()

        feedback.phase = "DONE"
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.success = True
        result.message = "Trajectory completed"
        result.final_joint_rad = [0.0] * 6  # fill from pykinematics if available
        return result


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
