"""
sam_node — SAM2 segmentation + CSRT tracking.

Wraps robot_sam2_app_v2/.../vision/sam2_segmenter.py and tracking.py.

Subscribes:
  /gripper_camera/image_raw   sensor_msgs/Image  (main arm camera, index 1)

Services:
  /initialize_tracking   so101_interfaces/InitializeTracking
    Given a bbox, runs SAM2 to refine it, then starts CSRT tracking.

Publishes:
  /target_pixel   so101_interfaces/PixelPoint   (center of tracked bbox, each frame)

Parameters:
  sam2_checkpoint   string  ""        path to SAM2 .pt file
                                      TODO(hardware): set to actual checkpoint path
  sam2_model_cfg    string  "configs/sam2.1/sam2.1_hiera_t.yaml"
  device            string  "cuda"
  frame_id          string  "gripper_camera_link"

Notes:
  If sam2_checkpoint is empty or missing, SAM2 init is skipped and tracking
  uses a fixed bbox passed to InitializeTracking (no mask refinement).
  Requires --symlink-install and GPU for full operation.
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
    from so101_interfaces.srv import InitializeTracking
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


class SamNode(Node):
    def __init__(self) -> None:
        super().__init__('sam_node')
        self.declare_parameter('sam2_checkpoint', '')
        self.declare_parameter('sam2_model_cfg',  'configs/sam2.1/sam2.1_hiera_t.yaml')
        self.declare_parameter('device',          'cuda')
        self.declare_parameter('frame_id',        'gripper_camera_link')

        self._fid          = self.get_parameter('frame_id').value
        self._bridge       = CvBridge() if _CV_BRIDGE else None
        self._latest_frame = None
        self._tracker      = None   # ObjectTracker from v2
        self._tracking     = False

        if not _V2_FOUND:
            self.get_logger().warning('robot_sam2_app_v2 not found — SAM2 unavailable.')
        else:
            self._init_tracker()

        self.create_subscription(Image, '/gripper_camera/image_raw', self._img_cb, 1)

        if _IFACES:
            self._pub = self.create_publisher(PixelPoint, '/target_pixel', 10)
            self.create_service(InitializeTracking, '/initialize_tracking', self._init_srv)
        else:
            self.get_logger().error('so101_interfaces not found — build it first.')

        self.get_logger().info('sam_node ready.')

    def _init_tracker(self) -> None:
        try:
            from robot_sam2_app.tracking import ObjectTracker
            ckpt = self.get_parameter('sam2_checkpoint').value
            cfg  = self.get_parameter('sam2_model_cfg').value
            dev  = self.get_parameter('device').value
            self._tracker = ObjectTracker(ckpt, cfg, dev)
            if ckpt:
                self.get_logger().info(f'ObjectTracker ready (SAM2 checkpoint={ckpt})')
            else:
                self.get_logger().warning(
                    'sam2_checkpoint is empty — SAM2 refinement disabled. '
                    'TODO(hardware): set sam2_checkpoint param to your .pt file path.'
                )
        except Exception as exc:
            self.get_logger().error(f'ObjectTracker init failed: {exc}')

    def _img_cb(self, msg: Image) -> None:
        if self._bridge is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return
        self._latest_frame = frame

        if not self._tracking or self._tracker is None or not _IFACES:
            return
        try:
            result = self._tracker.process(frame)
            if result is not None and result.success:
                pt           = PixelPoint()
                pt.header.stamp    = self.get_clock().now().to_msg()
                pt.header.frame_id = self._fid
                pt.x               = float(result.center_x)
                pt.y               = float(result.center_y)
                pt.source_camera   = 'gripper'
                self._pub.publish(pt)
        except Exception as exc:
            self.get_logger().debug(f'Tracker process error: {exc}')

    def _init_srv(self, req, resp):
        if not _IFACES:
            resp.success = False
            resp.message = 'so101_interfaces not built'
            return resp
        if self._tracker is None:
            resp.success = False
            resp.message = 'Tracker not initialized'
            return resp
        if self._latest_frame is None:
            resp.success = False
            resp.message = 'No camera frame received yet'
            return resp
        try:
            bbox = (req.x_min, req.y_min, req.x_max, req.y_max)
            self._tracker.request_bbox(bbox)
            self._tracking = True
            resp.success   = True
            resp.message   = f'Tracking initialized at bbox={bbox}'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = str(exc)
        return resp


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
