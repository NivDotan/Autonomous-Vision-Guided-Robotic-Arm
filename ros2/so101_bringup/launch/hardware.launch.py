"""
Phase 2 — RViz + driver.

dry_run:=true  (default) — slider GUI publishes /joint_states for visualization.
dry_run:=false           — driver publishes real arm positions; GUI is OFF
                           (it would conflict by overwriting /joint_states).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    dry_run     = LaunchConfiguration('dry_run')
    backend     = LaunchConfiguration('backend')
    serial_port = LaunchConfiguration('serial_port')

    description_pkg = FindPackageShare('so101_description')
    xacro_file  = PathJoinSubstitution([description_pkg, 'urdf', 'so101_arm.urdf.xacro'])
    rviz_config = PathJoinSubstitution([description_pkg, 'rviz', 'display.rviz'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file, ' variant:=follower']),
        value_type=str,
    )

    driver_launch = PathJoinSubstitution(
        [FindPackageShare('so101_driver'), 'launch', 'driver.launch.py']
    )

    return LaunchDescription([
        DeclareLaunchArgument('dry_run',     default_value='true'),
        DeclareLaunchArgument('backend',     default_value='daemon'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),

        # Phase 1: visualization
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),
        # Slider GUI only in dry_run — with real hardware the driver publishes /joint_states
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(dry_run),
        ),

        # Phase 2: driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch),
            launch_arguments={
                'dry_run':     dry_run,
                'backend':     backend,
                'serial_port': serial_port,
            }.items(),
        ),
    ])
