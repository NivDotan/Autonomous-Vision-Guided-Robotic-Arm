"""Phase 7 — Gazebo simulation (delegates to so101_gazebo)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('so101_gazebo'), 'launch', 'gazebo.launch.py']
                )
            ),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
        ),
    ])
