"""
Phase 2 — SO-101 driver launch.

Starts so101_driver/driver_node with params from driver_params.yaml.
No hardware is touched in dry_run mode (default).

Usage:
  ros2 launch so101_driver driver.launch.py
  ros2 launch so101_driver driver.launch.py dry_run:=false backend:=feetech
  ros2 launch so101_driver driver.launch.py dry_run:=false backend:=daemon
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('so101_driver'), 'config', 'driver_params.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true',
                              description='true = no hardware connect'),
        DeclareLaunchArgument('backend', default_value='daemon',
                              description='"daemon" or "feetech"'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Serial port for feetech backend'),
        DeclareLaunchArgument('use_official_names', default_value='false',
                              description='Publish /joint_states with official SO-101 joint names'),

        Node(
            package='so101_driver',
            executable='driver_node',
            name='so101_driver',
            output='screen',
            parameters=[
                params_file,
                {
                    'dry_run':            LaunchConfiguration('dry_run'),
                    'backend':            LaunchConfiguration('backend'),
                    'serial_port':        LaunchConfiguration('serial_port'),
                    'use_official_names': LaunchConfiguration('use_official_names'),
                },
            ],
        ),
    ])
