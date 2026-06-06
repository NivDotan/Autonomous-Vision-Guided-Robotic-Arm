"""
Full stack launch — runs ALL phases (1–6) in one command.

  Phase 1: robot_state_publisher + RViz (so101_description)
  Phase 2: so101_driver
  Phase 3+4: so101_perception (cameras + sensor + AI)
  Phase 5: so101_calibration
  Phase 6: so101_task_planner

Phase 7 (Gazebo) is a separate simulation demo; do not run alongside real hardware.

Usage — dry run (no hardware, just nodes + RViz):
  ros2 launch so101_bringup full_stack.launch.py

Usage — real hardware (official SO-101 URDF via legalaspro):
  ros2 launch so101_bringup full_stack.launch.py \\
      dry_run:=false \\
      backend:=feetech \\
      serial_port:=/dev/ttyACM0 \\
      sensor_port:=/dev/ttyACM1 \\
      sam2_checkpoint:=/path/to/sam2.1_hiera_tiny.pt \\
      calibration_file:=/path/to/calibration.json \\
      use_official_names:=true

Start a task (in another terminal after launch):
  ros2 service call /start_task so101_interfaces/srv/StartTask \\
      '{pick_query: "the red cup", place_query: "the open box"}'

Monitor state:
  ros2 topic echo /task_state
  ros2 topic echo /joint_states
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _inc(pkg: str, rel: str, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', rel])
        ),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    gui              = LaunchConfiguration('gui')
    dry_run          = LaunchConfiguration('dry_run')
    backend          = LaunchConfiguration('backend')
    port             = LaunchConfiguration('serial_port')
    sensor_port      = LaunchConfiguration('sensor_port')
    ckpt             = LaunchConfiguration('sam2_checkpoint')
    cal_file         = LaunchConfiguration('calibration_file')
    use_official     = LaunchConfiguration('use_official_names')
    urdf_file        = LaunchConfiguration('urdf_file')

    description_pkg  = FindPackageShare('so101_description')
    default_xacro    = PathJoinSubstitution([description_pkg, 'urdf', 'so101_arm.urdf.xacro'])
    rviz_config      = PathJoinSubstitution([description_pkg, 'rviz', 'display.rviz'])

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file,
                 ' variant:=follower']),
        value_type=str,
    )

    return LaunchDescription([
        # ── launch arguments ─────────────────────────────────────────────────
        DeclareLaunchArgument('urdf_file',         default_value=default_xacro,
                              description='Path to xacro/URDF file. Default: so101_arm.urdf.xacro'),
        DeclareLaunchArgument('gui',               default_value='true',
                              description='Show RViz and joint slider GUI'),
        DeclareLaunchArgument('dry_run',           default_value='true',
                              description='true=no hardware; false=real arm'),
        DeclareLaunchArgument('backend',           default_value='daemon',
                              description='"daemon" or "feetech"'),
        DeclareLaunchArgument('serial_port',       default_value='/dev/ttyACM0',
                              description='Arm serial port (feetech backend)'),
        DeclareLaunchArgument('sensor_port',       default_value='/dev/ttyACM1',
                              description='VL53 ESP32 serial port'),
        DeclareLaunchArgument('sam2_checkpoint',   default_value='',
                              description='Path to SAM2 .pt checkpoint'),
        DeclareLaunchArgument('calibration_file',  default_value='',
                              description='Path to calibration JSON'),
        DeclareLaunchArgument('use_official_names', default_value='false',
                              description='Publish /joint_states with official joint names'),

        # ── Phase 1: robot visualisation ─────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(gui),
        ),

        # ── Phase 2: driver ───────────────────────────────────────────────────
        _inc('so101_driver', 'driver.launch.py',
             dry_run=dry_run, backend=backend, serial_port=port),

        # ── Phase 3+4: perception ─────────────────────────────────────────────
        _inc('so101_perception', 'perception_ai.launch.py',
             dry_run_sensor=dry_run, sam2_checkpoint=ckpt),

        # ── Phase 5: calibration ──────────────────────────────────────────────
        _inc('so101_calibration', 'calibration.launch.py',
             calibration_file=cal_file),

        # ── Phase 6: task planner ─────────────────────────────────────────────
        _inc('so101_task_planner', 'task_planner.launch.py',
             dry_run=dry_run),
    ])
