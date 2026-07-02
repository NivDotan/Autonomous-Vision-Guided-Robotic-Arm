#!/usr/bin/env python3
"""
Run Grounding DINO on a ROS2 image TOPIC (works for the Gazebo sim camera or any
real camera publishing sensor_msgs/Image). Draws the detected box live.

Unlike tools/test_grounding_dino.py (which reads a v4l2 device), this reads a ROS
topic — use it for the simulated gripper camera.

Usage (with the sim running):
  python3 tools/detect_on_topic.py --query "red cylinder"
  python3 tools/detect_on_topic.py --topic /gripper_camera/image_raw --query "the box"
  python3 tools/detect_on_topic.py --device cpu --thresh 0.25

Keys (in the video window):
  T → type a new query in the terminal
  +/- → raise/lower threshold
  Q → quit
"""
import argparse
import os
import sys

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "robot_sam2_app_v2"))
from robot_sam2_app.vision.vqa_detector import VQADetector

try:
    from cv_bridge import CvBridge
except ImportError:
    sys.exit("cv_bridge missing: sudo apt install ros-jazzy-cv-bridge")


class TopicDetector(Node):
    def __init__(self, topic):
        super().__init__("detect_on_topic")
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, topic, self._cb, 1)

    def _cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")


def detect(det, frame, query, thresh):
    import torch
    from PIL import Image as PImage
    det._ensure_loaded()
    try:
        image = PImage.fromarray(frame[..., ::-1])
        inputs = det._processor(images=image, text=query.rstrip(".") + ".",
                                return_tensors="pt").to(det._device)
        with torch.no_grad():
            out = det._model(**inputs)
        res = det._processor.post_process_grounded_object_detection(
            out, inputs.input_ids, target_sizes=[image.size[::-1]])
        boxes, scores = res[0]["boxes"], res[0]["scores"]
        mask = scores > thresh
        boxes, scores = boxes[mask], scores[mask]
        if len(boxes) == 0:
            return None
        i = int(scores.argmax())
        x0, y0, x1, y1 = (int(v) for v in boxes[i].tolist())
        return x0, y0, x1, y1, float(scores[i])
    except Exception as e:
        print("[detect]", e)
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/gripper_camera/image_raw")
    ap.add_argument("--query", default="red cylinder")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--model", default="IDEA-Research/grounding-dino-tiny")
    ap.add_argument("--thresh", type=float, default=0.30)
    ap.add_argument("--every", type=int, default=2)
    args = ap.parse_args()

    print(f"Loading {args.model} on {args.device} ...")
    det = VQADetector(args.model, args.device)
    det.load()
    print(f"Ready. Subscribing to {args.topic}. Query='{args.query}'. T=new query, Q=quit")

    rclpy.init()
    node = TopicDetector(args.topic)
    query, thresh, every = args.query, args.thresh, args.every
    last_box, i = None, 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is None:
                continue
            frame = node.frame.copy()
            if i % every == 0:
                last_box = detect(det, frame, query, thresh)
            i += 1
            if last_box:
                x0, y0, x1, y1, s = last_box
                cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
                cv2.putText(frame, f"{query} {s:.2f}", (x0, max(20, y0 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(frame, f"'{query}' - no detection", (15, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"thr={thresh:.2f}  T=query +/-=thr Q=quit", (15, 465),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 0), 2)
            cv2.imshow("Grounding DINO on topic", frame)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("t"):
                q = input(f"New query [{query}]: ").strip()
                if q:
                    query, last_box = q, None
            elif k in (ord("+"), ord("=")):
                thresh = min(0.95, thresh + 0.05)
            elif k in (ord("-"), ord("_")):
                thresh = max(0.05, thresh - 0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
