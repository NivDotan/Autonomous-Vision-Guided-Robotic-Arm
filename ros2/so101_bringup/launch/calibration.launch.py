"""Phase 5 — calibration node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cal_launch = PathJoinSubstitution(
        [FindPackageShare('so101_calibration'), 'launch', 'calibration.launch.py']
    )
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cal_launch),
            launch_arguments={
                'calibration_file': LaunchConfiguration('calibration_file'),
            }.items(),
        ),
    ])
