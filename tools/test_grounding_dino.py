#!/usr/bin/env python3
"""
Live Grounding DINO test — see the detector working on the camera feed.

Opens a camera, loads the same VQADetector the app uses, and continuously draws
the detected bounding box for whatever text query you type. No robot/daemon needed.

Usage:
    python3 tools/test_grounding_dino.py                 # gripper camera from config
    python3 tools/test_grounding_dino.py --query "the red cup"
    python3 tools/test_grounding_dino.py --camera /dev/video4
    python3 tools/test_grounding_dino.py --device cpu    # if no/low GPU
    python3 tools/test_grounding_dino.py --every 3        # detect every 3rd frame

Keys (in the video window):
    T  → type a new query in the terminal
    +/- → raise / lower the confidence threshold
    Q  → quit
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import cv2

# Make robot_sam2_app importable when run from the repo root or tools/.
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "robot_sam2_app_v2"))

from robot_sam2_app import config as cfg
from robot_sam2_app.vision.vqa_detector import VQADetector


def open_camera(device) -> cv2.VideoCapture:
    """Open with the V4L2 backend + MJPG (same as the app)."""
    is_linux_path = isinstance(device, str) and device.startswith("/dev/")
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2) if is_linux_path else cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"ERROR: could not open camera {device}")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc("M", "J", "P", "G"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    print(f"Camera {device} opened: {int(cap.get(3))}x{int(cap.get(4))}")
    return cap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default=str(cfg.CAMERA_INDEX),
                    help="device path/index (default: gripper camera from config)")
    ap.add_argument("--query", default=cfg.DEFAULT_TARGET_CLASS,
                    help="initial text query")
    ap.add_argument("--device", default=cfg.VQA_DEVICE, help="cuda or cpu")
    ap.add_argument("--model", default=cfg.VQA_MODEL)
    ap.add_argument("--every", type=int, default=1,
                    help="run detection every Nth frame (raise to speed up display)")
    ap.add_argument("--thresh", type=float, default=0.30,
                    help="confidence threshold (Grounding DINO scores run low)")
    args = ap.parse_args()

    cam = args.camera
    if cam.isdigit():
        cam = int(cam)

    print(f"Loading {args.model} on {args.device} (first load takes a few seconds) ...")
    det = VQADetector(args.model, args.device)
    det.load()
    print(f"Ready. Query = '{args.query}'.  Keys: T=new query, +/- threshold, Q=quit")

    cap = open_camera(cam)
    query = args.query
    thresh = args.thresh
    every = max(1, args.every)

    last_box = None
    last_ms = 0.0
    frame_i = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            print("camera read failed"); break
        frame = cv2.flip(frame, 1)   # match the app's mirrored view

        if frame_i % every == 0 and query:
            t0 = time.time()
            last_box = _detect(det, frame, query, thresh)
            last_ms = (time.time() - t0) * 1000.0
        frame_i += 1

        # ── overlay ──────────────────────────────────────────────────────────
        if last_box is not None:
            x0, y0, x1, y1, score = last_box
            cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 255, 0), 2)
            cv2.putText(frame, f"{query}  {score:.2f}", (x0, max(20, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        else:
            cv2.putText(frame, f"'{query}' — no detection", (15, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(frame, f"thr={thresh:.2f}  {last_ms:.0f}ms  (T=query +/-=thr Q=quit)",
                    (15, 465), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 0), 2)
        cv2.imshow("Grounding DINO test", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("t"):
            new_q = input(f"New query [{query}]: ").strip()
            if new_q:
                query = new_q
                last_box = None
        elif key in (ord("+"), ord("=")):
            thresh = min(0.95, thresh + 0.05)
        elif key in (ord("-"), ord("_")):
            thresh = max(0.05, thresh - 0.05)

    cap.release()
    cv2.destroyAllWindows()


def _detect(det: VQADetector, frame, query: str, thresh: float):
    """Run detection and return (x0,y0,x1,y1,score) of the best box, or None.

    Reimplements VQADetector.detect_bbox's pipeline here so we can (a) use an
    adjustable threshold and (b) recover the score for display.
    """
    import torch
    from PIL import Image

    det._ensure_loaded()
    try:
        image = Image.fromarray(frame[..., ::-1])  # BGR -> RGB
        text = query.rstrip(".") + "."
        inputs = det._processor(images=image, text=text, return_tensors="pt").to(det._device)
        with torch.no_grad():
            outputs = det._model(**inputs)
        results = det._processor.post_process_grounded_object_detection(
            outputs, inputs.input_ids, target_sizes=[image.size[::-1]],
        )
        boxes, scores = results[0]["boxes"], results[0]["scores"]
        mask = scores > thresh
        boxes, scores = boxes[mask], scores[mask]
        if len(boxes) == 0:
            return None
        best = int(scores.argmax())
        x0, y0, x1, y1 = (int(v) for v in boxes[best].tolist())
        return x0, y0, x1, y1, float(scores[best])
    except Exception as exc:
        print(f"[detect] error: {exc}")
        return None


if __name__ == "__main__":
    main()
