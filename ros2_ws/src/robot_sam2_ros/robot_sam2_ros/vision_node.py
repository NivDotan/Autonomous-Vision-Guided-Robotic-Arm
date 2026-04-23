"""
ROS2 vision node — runs SAM2 segmentation + CSRT tracking.

Subscribes: /camera/color/image_raw (sensor_msgs/Image)
Publishes:  /tracking/result (robot_sam2_ros/msg/ObjectPose)
            /camera/debug/image (sensor_msgs/Image) — annotated frame

Provides service: /vision/click_target (std_srvs/SetBool placeholder)
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', '..', '..', '..', '..', '..', 'robot_sam2_app'))

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
try:
    from cv_bridge import CvBridge
    _BRIDGE_OK = True
except ImportError:
    _BRIDGE_OK = False

from robot_sam2_ros.msg import ObjectPose  # type: ignore
from robot_sam2_app.tracking import ObjectTracker
from robot_sam2_app.vision.sam2_segmenter import SAM2Segmenter


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.get_logger().info("Initialising SAM2 segmenter...")
        self.segmenter = SAM2Segmenter()
        self.tracker   = ObjectTracker()
        self.frame_idx = 0
        self.bridge    = CvBridge() if _BRIDGE_OK else None

        self._img_sub = self.create_subscription(
            Image, "/camera/color/image_raw", self._image_cb, 10)
        self._pose_pub  = self.create_publisher(ObjectPose, "/tracking/result", 10)
        self._debug_pub = self.create_publisher(Image, "/camera/debug/image", 10)

        self.get_logger().info("VisionNode ready.")

    def _image_cb(self, msg: Image) -> None:
        if self.bridge is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        result = self.tracker.process(frame, self.segmenter, self.frame_idx,
                                       approach_mode=False)
        self.frame_idx += 1

        pose_msg = ObjectPose()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.tracking_active = result.success
        if result.success:
            pose_msg.center_x  = float(result.center_x) / frame.shape[1]
            pose_msg.center_y  = float(result.center_y) / frame.shape[0]
            pose_msg.area      = float(result.area)
        self._pose_pub.publish(pose_msg)

        # Publish annotated debug image.
        debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        debug_msg.header = msg.header
        self._debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
