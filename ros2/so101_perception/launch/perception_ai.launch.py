"""Phase 4 — full perception stack: cameras + sensor + AI detection + SAM2 tracking."""
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
        DeclareLaunchArgument('dry_run_sensor', default_value='true'),
        DeclareLaunchArgument('sam2_checkpoint', default_value='',
                              description='Path to SAM2 .pt checkpoint file'),

        Node(package='so101_perception', executable='base_camera_node',
             name='base_camera', output='screen', parameters=[params]),
        Node(package='so101_perception', executable='gripper_camera_node',
             name='gripper_camera', output='screen', parameters=[params]),
        Node(package='so101_perception', executable='distance_sensor_node',
             name='distance_sensor', output='screen',
             parameters=[params, {'dry_run': LaunchConfiguration('dry_run_sensor')}]),
        Node(package='so101_perception', executable='object_detection_node',
             name='object_detection', output='screen', parameters=[params]),
        Node(package='so101_perception', executable='sam_node',
             name='sam_node', output='screen',
             parameters=[params,
                         {'sam2_checkpoint': LaunchConfiguration('sam2_checkpoint')}]),
        Node(package='so101_perception', executable='drop_zone_detector_node',
             name='drop_zone_detector', output='screen', parameters=[params]),
    ])
