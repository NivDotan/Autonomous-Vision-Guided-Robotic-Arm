#!/usr/bin/env python3
"""
Autonomous search in Gazebo: sweep the arm through "look" poses, run Grounding
DINO on the gripper camera at each, and stop when the target is found.

This is the sim equivalent of the real robot's _scan_for_target sweep.

Usage (with the sim running):
  python3 sim_search.py --query "red cylinder"
  python3 sim_search.py --query "red object" --device cpu

Keys in the window: Q = quit.
"""
import argparse, os, sys, time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(_REPO, "robot_sam2_app_v2"))
from robot_sam2_app.vision.vqa_detector import VQADetector

try:
    from cv_bridge import CvBridge
except ImportError:
    sys.exit("cv_bridge missing: sudo apt install ros-jazzy-cv-bridge")

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# "Look down at the table" base config; only shoulder_pan changes during the sweep.
# TODO(tune): adjust lift/elbow/wrist so the gripper camera points at the table.
LIFT, ELBOW, WRISTF = -0.6, 1.2, 0.8
PAN_SWEEP = [-1.0, -0.6, -0.2, 0.2, 0.6, 1.0]   # radians, left → right


class Searcher(Node):
    def __init__(self, topic):
        super().__init__("sim_search")
        self.bridge = CvBridge()
        self.frame = None
        self.pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self.create_subscription(Image, topic, self._cb, 1)

    def _cb(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def look(self, pan, seconds=1.5):
        m = JointTrajectory(); m.joint_names = JOINTS
        p = JointTrajectoryPoint()
        p.positions = [pan, LIFT, ELBOW, WRISTF, 0.0, 0.0]
        p.time_from_start.sec = int(seconds)
        m.points = [p]; self.pub.publish(m)


def detect(det, frame, query, thresh):
    import torch
    from PIL import Image as PImage
    det._ensure_loaded()
    image = PImage.fromarray(frame[..., ::-1])
    inp = det._processor(images=image, text=query.rstrip(".") + ".", return_tensors="pt").to(det._device)
    with torch.no_grad():
        out = det._model(**inp)
    res = det._processor.post_process_grounded_object_detection(
        out, inp.input_ids, target_sizes=[image.size[::-1]])
    b, s = res[0]["boxes"], res[0]["scores"]
    m = s > thresh; b, s = b[m], s[m]
    if len(b) == 0:
        return None
    i = int(s.argmax())
    x0, y0, x1, y1 = (int(v) for v in b[i].tolist())
    return x0, y0, x1, y1, float(s[i])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--query", default="red cylinder")
    ap.add_argument("--topic", default="/gripper_camera/image_raw")
    ap.add_argument("--device", default="cuda")
    ap.add_argument("--thresh", type=float, default=0.30)
    ap.add_argument("--dwell", type=float, default=2.5, help="seconds to settle + look at each pose")
    args = ap.parse_args()

    print("Loading Grounding DINO ...")
    det = VQADetector("IDEA-Research/grounding-dino-tiny", args.device); det.load()

    rclpy.init()
    node = Searcher(args.topic)

    def show(tag, box=None):
        if node.frame is None:
            return
        f = node.frame.copy()
        if box:
            x0, y0, x1, y1, sc = box
            cv2.rectangle(f, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(f, f"{args.query} {sc:.2f}", (x0, max(20, y0-8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(f, tag, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 255), 2)
        cv2.imshow("sim search (gripper cam)", f); cv2.waitKey(1)

    found = None
    try:
        for pan in PAN_SWEEP:
            node.look(pan)
            print(f"[SEARCH] looking at pan={pan:+.1f}")
            t0 = time.time()
            while time.time() - t0 < args.dwell:        # settle + stream frames
                rclpy.spin_once(node, timeout_sec=0.05)
                show(f"scanning pan={pan:+.1f}")
            if node.frame is None:
                continue
            box = detect(det, node.frame, args.query, args.thresh)
            show(f"pan={pan:+.1f}", box)
            if box:
                print(f"[SEARCH] FOUND '{args.query}' at pan={pan:+.1f}  score={box[4]:.2f}")
                found = (pan, box)
                break
        if found:
            print(f"[SEARCH] done — target at shoulder_pan={found[0]:+.1f}. Holding view.")
            # keep showing the final view until Q
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                show(f"FOUND at pan={found[0]:+.1f}", found[1])
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        else:
            print(f"[SEARCH] '{args.query}' not found across the sweep. "
                  f"Lower --thresh or retune LIFT/ELBOW/WRISTF.")
    finally:
        node.destroy_node(); rclpy.shutdown(); cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
