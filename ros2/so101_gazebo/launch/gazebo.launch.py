"""
Phase 7 — Gazebo Harmonic simulation for the SO-101.

Starts:
  - Gazebo Harmonic with so101_table.world
  - robot_state_publisher (URDF → TF, same placeholder as Phase 1)
  - ros_gz_bridge: Gazebo clock → /clock, Gazebo joint states → /joint_states
  - joint_state_publisher_gui (optional, for manual joint control)

Prerequisites:
  sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

Note:
  The SO-101 robot model is NOT spawned automatically in this phase.
  TODO(gazebo): once official SO-ARM101 SDF/meshes are available, spawn the
  model with a gz_spawn_entity call and wire up ros2_control. See
  so101_description/urdf/FETCH_OFFICIAL_URDF.md for how to get the meshes.

Usage:
  ros2 launch so101_gazebo gazebo.launch.py
  ros2 launch so101_gazebo gazebo.launch.py gui:=false   # headless
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gz_world = PathJoinSubstitution(
        [FindPackageShare('so101_gazebo'), 'worlds', 'so101_table.world']
    )
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('so101_description'), 'urdf', 'so101_arm.urdf.xacro']
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', gz_world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Start joint_state_publisher_gui'),

        gz_sim,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Bridge Gazebo clock → ROS /clock
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_clock_bridge',
            output='screen',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        ),

        # Optional slider GUI for manual joint driving
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
    ])
