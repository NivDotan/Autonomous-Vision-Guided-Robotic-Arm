"""Phase 4 — full perception: cameras + sensor + Grounding DINO + SAM2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ai_launch = PathJoinSubstitution(
        [FindPackageShare('so101_perception'), 'launch', 'perception_ai.launch.py']
    )
    return LaunchDescription([
        DeclareLaunchArgument('dry_run_sensor',  default_value='true'),
        DeclareLaunchArgument('sam2_checkpoint', default_value='',
                              description='Path to SAM2 .pt file'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ai_launch),
            launch_arguments={
                'dry_run_sensor':  LaunchConfiguration('dry_run_sensor'),
                'sam2_checkpoint': LaunchConfiguration('sam2_checkpoint'),
            }.items(),
        ),
    ])
