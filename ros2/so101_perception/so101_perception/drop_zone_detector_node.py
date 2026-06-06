"""
drop_zone_detector_node — determines where to place the object.

Two modes selected via /set_drop_zone service:
  manual  — fixed pixel from service request (deterministic, no AI)
  ai      — runs Grounding DINO on base camera to find the described location

Subscribes:
  /base_camera/image_raw   sensor_msgs/Image  (cached for AI mode)

Services:
  /set_drop_zone   so101_interfaces/SetDropZone

Publishes:
  /drop_zone_pixel   so101_interfaces/PixelPoint  (latched, re-published at 1 Hz)

Parameters:
  default_mode   string  "manual"   "manual" or "ai"
  model          string  "IDEA-Research/grounding-dino-tiny"
  device         string  "cuda"
"""
from __future__ import annotations

import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE = True
except ImportError:
    _CV_BRIDGE = False

try:
    from so101_interfaces.msg import PixelPoint
    from so101_interfaces.srv import SetDropZone
    _IFACES = True
except ImportError:
    _IFACES = False

def _add_v2_to_path() -> bool:
    for parent in Path(__file__).resolve().parents:
        v2 = parent / 'robot_sam2_app_v2'
        if v2.is_dir():
            sys.path.insert(0, str(v2))
            return True
    return False

_V2_FOUND = _add_v2_to_path()


class DropZoneDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('drop_zone_detector')
        self.declare_parameter('default_mode', 'manual')
        self.declare_parameter('model',        'IDEA-Research/grounding-dino-tiny')
        self.declare_parameter('device',       'cuda')

        self._bridge       = CvBridge() if _CV_BRIDGE else None
        self._latest_frame = None
        self._detector     = None
        self._drop_x: float | None = None
        self._drop_y: float | None = None
        self._mode         = self.get_parameter('default_mode').value

        if _V2_FOUND:
            try:
                from robot_sam2_app.vision.vqa_detector import VQADetector
                self._detector = VQADetector(
                    self.get_parameter('model').value,
                    self.get_parameter('device').value,
                )
            except Exception as exc:
                self.get_logger().warning(f'Grounding DINO unavailable: {exc}')

        self.create_subscription(Image, '/base_camera/image_raw', self._img_cb, 1)

        if _IFACES:
            self._pub = self.create_publisher(PixelPoint, '/drop_zone_pixel', 10)
            self.create_service(SetDropZone, '/set_drop_zone', self._set_srv)
            self.create_timer(1.0, self._latch_pub)   # re-publish at 1 Hz
        else:
            self.get_logger().error('so101_interfaces not found — build it first.')

        self.get_logger().info(f'drop_zone_detector ready (mode={self._mode}).')

    def _img_cb(self, msg: Image) -> None:
        if self._bridge is None:
            return
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _set_srv(self, req, resp):
        if not _IFACES:
            resp.success = False
            resp.message = 'so101_interfaces not built'
            return resp
        self._mode = req.mode
        if req.mode == 'manual':
            self._drop_x = req.pixel_x
            self._drop_y = req.pixel_y
            resp.success = True
            resp.message = f'Drop zone set (manual): ({req.pixel_x}, {req.pixel_y})'
        elif req.mode == 'ai':
            if self._detector is None:
                resp.success = False
                resp.message = 'AI detector not available'
                return resp
            if self._latest_frame is None:
                resp.success = False
                resp.message = 'No camera frame — is base_camera_node running?'
                return resp
            bbox = self._detector.detect_bbox(self._latest_frame, req.query)
            if bbox is None:
                resp.success = False
                resp.message = f"Nothing found for '{req.query}'"
                return resp
            x0, y0, x1, y1 = bbox
            self._drop_x  = (x0 + x1) / 2.0
            self._drop_y  = (y0 + y1) / 2.0
            resp.success  = True
            resp.message  = f"Drop zone found (AI) for '{req.query}': center ({self._drop_x:.0f}, {self._drop_y:.0f})"
        else:
            resp.success = False
            resp.message = f"Unknown mode '{req.mode}' — use 'manual' or 'ai'"
        self.get_logger().info(resp.message)
        return resp

    def _latch_pub(self) -> None:
        if not _IFACES or self._drop_x is None:
            return
        pt              = PixelPoint()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.x            = self._drop_x
        pt.y            = self._drop_y
        pt.source_camera = 'base'
        self._pub.publish(pt)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DropZoneDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
