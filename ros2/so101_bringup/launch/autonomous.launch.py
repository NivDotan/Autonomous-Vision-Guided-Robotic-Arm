"""
Phase 6 — full autonomous pick-and-place stack.

Starts all nodes needed for one-shot autonomous operation:
  Phase 2: so101_driver
  Phase 3+4: so101_perception (cameras + sensor + AI)
  Phase 5: so101_calibration
  Phase 6: so101_task_planner

Usage:
  ros2 launch so101_bringup autonomous.launch.py \\
      dry_run:=false backend:=feetech serial_port:=/dev/ttyACM0 \\
      sam2_checkpoint:=/path/to/sam2.1_hiera_tiny.pt \\
      calibration_file:=/path/to/calibration.json

To start a task:
  ros2 service call /start_task so101_interfaces/srv/StartTask \\
      '{pick_query: "the red cup", place_query: "the open box"}'

To abort:
  ros2 service call /abort_task std_srvs/srv/Trigger '{}'
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _inc(pkg: str, rel: str, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), 'launch', rel])
        ),
        launch_arguments=args.items(),
    )


def generate_launch_description():
    dry_run     = LaunchConfiguration('dry_run')
    backend     = LaunchConfiguration('backend')
    port        = LaunchConfiguration('serial_port')
    ckpt        = LaunchConfiguration('sam2_checkpoint')
    cal_file    = LaunchConfiguration('calibration_file')
    sensor_port = LaunchConfiguration('sensor_port')

    return LaunchDescription([
        DeclareLaunchArgument('dry_run',           default_value='true'),
        DeclareLaunchArgument('backend',           default_value='daemon'),
        DeclareLaunchArgument('serial_port',       default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('sensor_port',       default_value='/dev/ttyACM1',
                              description='Serial port for VL53 ESP32 (was COM3)'),
        DeclareLaunchArgument('sam2_checkpoint',   default_value=''),
        DeclareLaunchArgument('calibration_file',  default_value=''),

        # Phase 2 — driver
        _inc('so101_driver', 'driver.launch.py',
             dry_run=dry_run, backend=backend, serial_port=port),

        # Phase 3+4 — perception
        _inc('so101_perception', 'perception_ai.launch.py',
             dry_run_sensor=dry_run, sam2_checkpoint=ckpt),

        # Phase 5 — calibration
        _inc('so101_calibration', 'calibration.launch.py',
             calibration_file=cal_file),

        # Phase 6 — task planner
        _inc('so101_task_planner', 'task_planner.launch.py',
             dry_run=dry_run),
    ])
