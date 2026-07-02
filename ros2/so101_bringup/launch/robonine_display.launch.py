"""RViz display for the RoboNine SO-ARM101 mesh model, without joint sliders."""

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("so_arm_101_description")
    xacro_file = PathJoinSubstitution([pkg_share, "urdf", "so_101.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg_share, "config", "display.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file, " sim_backend:=gazebo"]),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
    ])
