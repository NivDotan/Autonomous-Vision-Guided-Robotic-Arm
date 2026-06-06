"""Phase 6 — task planner launch (planner node only, peers started separately)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution(
        [FindPackageShare('so101_task_planner'), 'config', 'planner_params.yaml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        Node(
            package='so101_task_planner',
            executable='task_planner_node',
            name='task_planner',
            output='screen',
            parameters=[params, {'dry_run': LaunchConfiguration('dry_run')}],
        ),
    ])
