"""
distance_sensor_node — VL53L1X ToF distance sensor via ESP32 serial.

Wraps robot_sam2_app_v2/.../vl53_sensor.py's VL53Sensor directly.

Publishes:
  /range   sensor_msgs/Range   raw distance in metres (INFRARED type)

Parameters:
  serial_port      string  "/dev/ttyACM1"   was COM3 on Windows
                                            TODO(hardware): confirm /dev path
  baudrate         int     115200
  publish_rate     float   10.0 Hz
  frame_id         string  "vl53_link"
  dry_run          bool    true
  stable_window_mm int     15    VL53_STABLE_WINDOW_MM (for task planner logic)
  max_jump_mm      int     30    VL53_MAX_JUMP_MM

Notes:
  The ESP32 sends ASCII lines: "Distance: NNN mm"
  VL53L1X range: 4 mm to 4000 mm, ~27° FOV.
  Requires --symlink-install to find v2 source; set dry_run:=false with hardware.
"""
from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

def _add_v2_to_path() -> bool:
    for parent in Path(__file__).resolve().parents:
        v2 = parent / 'robot_sam2_app_v2'
        if v2.is_dir():
            sys.path.insert(0, str(v2))
            return True
    return False

_V2_FOUND = _add_v2_to_path()


class DistanceSensorNode(Node):
    def __init__(self) -> None:
        super().__init__('distance_sensor')
        self.declare_parameter('serial_port',      '/dev/ttyACM1')
        self.declare_parameter('baudrate',         115200)
        self.declare_parameter('publish_rate',     10.0)
        self.declare_parameter('frame_id',         'vl53_link')
        self.declare_parameter('dry_run',          True)
        self.declare_parameter('stable_window_mm', 15)
        self.declare_parameter('max_jump_mm',      30)

        self._fid      = self.get_parameter('frame_id').value
        self._dry_run  = self.get_parameter('dry_run').value
        self._sensor   = None
        fps            = self.get_parameter('publish_rate').value

        if not self._dry_run and _V2_FOUND:
            self._connect()
        else:
            self.get_logger().info('Distance sensor: dry-run or v2 not found — no serial opened.')

        self._pub = self.create_publisher(Range, '/range', 10)
        self.create_timer(1.0 / fps, self._timer_cb)
        self.get_logger().info('distance_sensor_node ready.')

    def _connect(self) -> None:
        try:
            from robot_sam2_app.vl53_sensor import VL53Sensor
            port = self.get_parameter('serial_port').value
            baud = self.get_parameter('baudrate').value
            self._sensor = VL53Sensor(port, baud)
            if self._sensor.connect():
                self.get_logger().info(f'VL53 connected on {port} @ {baud}')
            else:
                self._sensor = None
                self.get_logger().warning(
                    f'VL53 not available on {port}. '
                    'TODO(hardware): confirm /dev path (ls /dev/ttyACM*).'
                )
        except Exception as exc:
            self.get_logger().error(f'VL53 init error: {exc}')
            self._sensor = None

    def _timer_cb(self) -> None:
        if self._sensor is None or not self._sensor.connected:
            return
        dist_mm = self._sensor.distance_mm
        if dist_mm is None:
            return
        msg = Range()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._fid
        msg.radiation_type  = Range.INFRARED
        msg.field_of_view   = 0.471   # ~27° in radians
        msg.min_range       = 0.004   # 4 mm
        msg.max_range       = 4.0     # 4000 mm
        msg.range           = dist_mm / 1000.0
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        if self._sensor is not None and self._sensor.connected:
            self._sensor.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DistanceSensorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
