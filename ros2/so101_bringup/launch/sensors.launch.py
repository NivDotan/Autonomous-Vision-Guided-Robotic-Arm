"""Phase 3 — cameras + distance sensor (no AI models)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cameras_launch = PathJoinSubstitution(
        [FindPackageShare('so101_perception'), 'launch', 'cameras.launch.py']
    )
    return LaunchDescription([
        DeclareLaunchArgument('dry_run_sensor', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cameras_launch),
            launch_arguments={'dry_run_sensor': LaunchConfiguration('dry_run_sensor')}.items(),
        ),
    ])
