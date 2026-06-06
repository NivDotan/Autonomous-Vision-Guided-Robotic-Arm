"""Phase 5 — calibration node launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('so101_calibration'), 'config', 'calibration.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Path to calibration JSON file'),
        Node(
            package='so101_calibration',
            executable='calibration_node',
            name='calibration_node',
            output='screen',
            parameters=[params, {'calibration_file': LaunchConfiguration('calibration_file')}],
        ),
    ])
