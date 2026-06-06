"""Phase 3 — cameras and distance sensor only (no AI)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('so101_perception'), 'config', 'perception_params.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('dry_run_sensor', default_value='true',
                              description='dry_run for distance_sensor_node'),
        Node(package='so101_perception', executable='base_camera_node',
             name='base_camera', output='screen', parameters=[params]),
        Node(package='so101_perception', executable='gripper_camera_node',
             name='gripper_camera', output='screen', parameters=[params]),
        Node(package='so101_perception', executable='distance_sensor_node',
             name='distance_sensor', output='screen',
             parameters=[params, {'dry_run': LaunchConfiguration('dry_run_sensor')}]),
    ])
