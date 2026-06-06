"""
object_detection_node — Grounding DINO zero-shot object detector.

Wraps robot_sam2_app_v2/.../vision/vqa_detector.py's VQADetector.
Model is lazy-loaded on first service call (~500 MB VRAM).

Subscribes:
  /base_camera/image_raw   sensor_msgs/Image   (caches latest frame)

Services:
  /detect_object   so101_interfaces/DetectObject
    Request:  query string  (e.g. "the red cup on the left")
    Response: bbox + confidence, or success=false if nothing found

Publishes:
  /detected_objects   so101_interfaces/DetectedObjectArray
    (published each time a detection service call succeeds)

Parameters:
  model              string  "IDEA-Research/grounding-dino-tiny"
  device             string  "cuda"   (or "cpu" if VRAM is tight)
  confidence_thresh  float   0.75     mirrors VQA threshold in vqa_detector.py
  eager_load         bool    false    true = load model at startup
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
    _CV_BRIDGE = True
except ImportError:
    _CV_BRIDGE = False

try:
    from so101_interfaces.msg import DetectedObject, DetectedObjectArray
    from so101_interfaces.srv import DetectObject
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


class ObjectDetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('object_detection')
        self.declare_parameter('model',             'IDEA-Research/grounding-dino-tiny')
        self.declare_parameter('device',            'cuda')
        self.declare_parameter('confidence_thresh', 0.75)
        self.declare_parameter('eager_load',        False)

        self._bridge       = CvBridge() if _CV_BRIDGE else None
        self._latest_frame = None   # numpy BGR, updated by camera subscription
        self._detector     = None   # VQADetector, lazy-loaded

        if not _IFACES:
            self.get_logger().error(
                'so101_interfaces not found — build it first: '
                'colcon build --packages-select so101_interfaces'
            )
        if not _V2_FOUND:
            self.get_logger().warning('robot_sam2_app_v2 not found — detection will fail.')

        model  = self.get_parameter('model').value
        device = self.get_parameter('device').value

        if _V2_FOUND:
            try:
                from robot_sam2_app.vision.vqa_detector import VQADetector
                self._detector = VQADetector(model, device)
                if self.get_parameter('eager_load').value:
                    self._detector.load()
                    self.get_logger().info(f'Grounding DINO loaded ({model})')
                else:
                    self.get_logger().info(f'Grounding DINO ready (lazy, model={model})')
            except Exception as exc:
                self.get_logger().error(f'VQADetector init failed: {exc}')

        self.create_subscription(Image, '/base_camera/image_raw', self._img_cb, 1)

        if _IFACES:
            self._pub = self.create_publisher(DetectedObjectArray, '/detected_objects', 10)
            self.create_service(DetectObject, '/detect_object', self._detect_srv)

        self.get_logger().info('object_detection_node ready.')

    def _img_cb(self, msg: Image) -> None:
        if self._bridge is None:
            return
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def _detect_srv(self, req, resp):
        if not _IFACES:
            resp.success = False
            resp.message = 'so101_interfaces not built'
            return resp
        if self._detector is None:
            resp.success = False
            resp.message = 'Detector not initialized (check v2 path and model)'
            return resp
        if self._latest_frame is None:
            resp.success = False
            resp.message = 'No camera frame received yet — is base_camera_node running?'
            return resp

        bbox = self._detector.detect_bbox(self._latest_frame, req.query)
        if bbox is None:
            resp.success = False
            resp.message = f"Nothing found for query '{req.query}'"
            return resp

        x0, y0, x1, y1 = bbox
        h, w = self._latest_frame.shape[:2]

        resp.success      = True
        resp.message      = f"Detected '{req.query}'"
        resp.x_min        = float(x0)
        resp.y_min        = float(y0)
        resp.x_max        = float(x1)
        resp.y_max        = float(y1)
        resp.confidence   = 0.0   # VQADetector doesn't return score externally; threshold already applied
        resp.image_width  = w
        resp.image_height = h

        # Also publish to topic
        arr = DetectedObjectArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.query        = req.query
        obj              = DetectedObject()
        obj.label        = req.query
        obj.x_min        = resp.x_min
        obj.y_min        = resp.y_min
        obj.x_max        = resp.x_max
        obj.y_max        = resp.y_max
        obj.image_width  = w
        obj.image_height = h
        arr.objects      = [obj]
        self._pub.publish(arr)
        return resp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
