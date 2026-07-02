"""
Phase 1 — RViz visualization for the SO-101.

Loads the URDF and starts:
  - robot_state_publisher  (publishes /robot_description + TF from joint states)
  - joint_state_publisher_gui  (sliders -> /joint_states), toggle with gui:=false
  - rviz2

No hardware touched.

The urdf_file argument selects which xacro to load:
  • Default (legalaspro/so101-ros-physical-ai installed):
      so101_arm.urdf.xacro  ← official SO-101 with real meshes
  • Fallback (our placeholder, if no official package):
      so101_placeholder.urdf.xacro

Usage:
  ros2 launch so101_bringup display.launch.py
  ros2 launch so101_bringup display.launch.py gui:=false
  # explicit file override:
  ros2 launch so101_bringup display.launch.py \\
      urdf_file:=$(ros2 pkg prefix so101_description)/share/so101_description/urdf/so101_arm.urdf.xacro
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")

    description_pkg = FindPackageShare("so101_description")

    # Default to the committed wrapper. It uses the placeholder model unless the
    # official SO-101 URDF is installed and explicitly requested.
    default_xacro = PathJoinSubstitution([description_pkg, "urdf", "so101.urdf.xacro"])

    urdf_file   = LaunchConfiguration("urdf_file")
    variant     = LaunchConfiguration("variant")
    rviz_config = PathJoinSubstitution([description_pkg, "rviz", "display.rviz"])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", urdf_file]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "urdf_file", default_value=default_xacro,
            description="Path to the robot xacro/URDF file.",
        ),
        DeclareLaunchArgument(
            "variant", default_value="follower",
            description="'follower' (default) or 'leader'",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start joint_state_publisher_gui sliders",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            # Use the package rviz config if it exists; rviz2 opens with defaults otherwise.
            arguments=["-d", rviz_config],
        ),
    ])
