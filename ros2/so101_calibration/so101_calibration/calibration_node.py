"""
calibration_node — pixel-to-robot coordinate service.

Wraps robot_system/calibration/base_camera_to_robot.py's CameraRobotCalibration
(homography DLT, pure numpy, no OpenCV required).

Services:
  /pixel_to_robot_xy   so101_interfaces/PixelToRobotXY
    Converts a camera pixel (u, v) → robot base-frame (x, y, z=table_z) in metres.

Parameters:
  calibration_file  string  ""   path to a saved calibration JSON
                                 (output of CameraRobotCalibration.save())
                                 empty = uncalibrated; service returns success=false
  table_z           float   0.02  table height in robot frame (metres)
                                  TODO(hardware): measure actual table z

Usage:
  1. Run collect_calibration_points.py (helper script) to gather pixel↔robot pairs.
  2. The script saves a JSON; pass it as calibration_file param.
  3. Alternatively: call the calibration API programmatically from your pipeline.

Fallback:
  If calibration JSON is absent or not yet fitted, the service returns
  success=false with an informative message so the task planner can skip
  calibration-dependent steps or prompt the operator.
"""
from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node

try:
    from so101_interfaces.srv import PixelToRobotXY
    _IFACES = True
except ImportError:
    _IFACES = False


def _add_robot_system_to_path() -> bool:
    """Walk up from this file to find robot_system/calibration."""
    for parent in Path(__file__).resolve().parents:
        rs = parent / 'robot_system'
        if rs.is_dir():
            sys.path.insert(0, str(parent))
            return True
    return False

_RS_FOUND = _add_robot_system_to_path()


class CalibrationNode(Node):
    def __init__(self) -> None:
        super().__init__('calibration_node')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('table_z',          0.02)

        self._cal        = None
        table_z          = self.get_parameter('table_z').value
        cal_file         = self.get_parameter('calibration_file').value

        if not _RS_FOUND:
            self.get_logger().warning('robot_system not found in workspace tree.')
        else:
            self._load_calibration(cal_file, table_z)

        if not _IFACES:
            self.get_logger().error('so101_interfaces not found — build it first.')
        else:
            self.create_service(PixelToRobotXY, '/pixel_to_robot_xy', self._srv_cb)

        self.get_logger().info('calibration_node ready.')

    def _load_calibration(self, cal_file: str, table_z: float) -> None:
        try:
            from robot_system.calibration.base_camera_to_robot import CameraRobotCalibration
            if cal_file and Path(cal_file).exists():
                self._cal = CameraRobotCalibration.load(cal_file)
                self.get_logger().info(
                    f'Calibration loaded from {cal_file} '
                    f'({self._cal.n_points} points, table_z={self._cal.table_z} m)'
                )
            else:
                self._cal = CameraRobotCalibration(table_z=table_z)
                if cal_file:
                    self.get_logger().warning(
                        f'Calibration file {cal_file} not found — starting uncalibrated. '
                        'Run collect_calibration_points.py to create one.'
                    )
                else:
                    self.get_logger().info(
                        'No calibration_file set — service will return success=false. '
                        'TODO(hardware): collect calibration points and set the param.'
                    )
        except Exception as exc:
            self.get_logger().error(f'Calibration init error: {exc}')

    def _srv_cb(self, req, resp):
        if not _IFACES:
            resp.success = False
            resp.message = 'so101_interfaces not built'
            return resp
        if self._cal is None:
            resp.success = False
            resp.message = 'Calibration not initialized'
            return resp
        if not self._cal.is_fitted():
            resp.success = False
            resp.message = (
                'Calibration not fitted — collect ≥4 pixel↔robot point pairs and call fit().'
            )
            return resp
        try:
            x, y, z = self._cal.pixel_to_robot(req.pixel_x, req.pixel_y)
            resp.success = True
            resp.message = f'pixel ({req.pixel_x:.0f}, {req.pixel_y:.0f}) → robot ({x:.4f}, {y:.4f}, {z:.4f}) m'
            resp.robot_x = float(x)
            resp.robot_y = float(y)
            resp.robot_z = float(z)
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
        return resp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
