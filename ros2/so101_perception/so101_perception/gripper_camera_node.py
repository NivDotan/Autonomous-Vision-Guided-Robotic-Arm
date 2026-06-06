"""
gripper_camera_node — main arm camera used for SAM2 tracking (was index 1 in v2 app).

Publishes:
  /gripper_camera/image_raw   sensor_msgs/Image

Parameters:
  device_index  int    1       cv2.VideoCapture index
  width         int    640
  height        int    480
  fps           float  30.0
  frame_id      string "gripper_camera_link"
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
try:
    from cv_bridge import CvBridge
    _CV_BRIDGE = True
except ImportError:
    _CV_BRIDGE = False


class GripperCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('gripper_camera')
        self.declare_parameter('device_index', 1)
        self.declare_parameter('width',        640)
        self.declare_parameter('height',       480)
        self.declare_parameter('fps',          30.0)
        self.declare_parameter('frame_id',     'gripper_camera_link')

        idx       = self.get_parameter('device_index').value
        w         = self.get_parameter('width').value
        h         = self.get_parameter('height').value
        fps       = self.get_parameter('fps').value
        self._fid = self.get_parameter('frame_id').value

        self._bridge = CvBridge() if _CV_BRIDGE else None
        self._cap    = cv2.VideoCapture(idx)
        if not self._cap.isOpened():
            self.get_logger().warning(f'Camera index={idx} not available — standing by.')
        else:
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self._cap.set(cv2.CAP_PROP_FPS,          fps)
            self.get_logger().info(f'Gripper camera opened (index={idx}, {w}x{h} @ {fps}fps)')

        self._pub = self.create_publisher(Image, '/gripper_camera/image_raw', 10)
        self.create_timer(1.0 / fps, self._timer_cb)

    def _timer_cb(self) -> None:
        if not self._cap.isOpened() or self._bridge is None:
            return
        ret, frame = self._cap.read()
        if not ret:
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._fid
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
