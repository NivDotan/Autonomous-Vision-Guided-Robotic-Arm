"""
Tracking accuracy benchmark.

Compares the CSRT tracker's reported centroid against ArUco marker ground truth.
Computes per-frame pixel error and reports statistics.

Usage:
    python benchmarks/tracking_accuracy.py --video test_clip.mp4 --marker-id 0
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


def detect_aruco_centroid(frame: np.ndarray, marker_id: int) -> tuple[int, int] | None:
    """Returns pixel centroid (cx, cy) of the specified ArUco marker."""
    try:
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector   = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        corners, ids, _ = detector.detectMarkers(frame)
        if ids is None:
            return None
        for i, mid in enumerate(ids.flatten()):
            if mid == marker_id:
                pts = corners[i][0]
                cx  = int(pts[:, 0].mean())
                cy  = int(pts[:, 1].mean())
                return cx, cy
    except Exception:
        pass
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--video",     type=str, required=True)
    parser.add_argument("--marker-id", type=int, default=0)
    parser.add_argument("--output",    type=str, default="results")
    args = parser.parse_args()

    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent / "robot_sam2_app"))
    from robot_sam2_app.tracking import ObjectTracker
    from robot_sam2_app.vision.sam2_segmenter import SAM2Segmenter

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"Cannot open video: {args.video}")
        return

    tracker   = ObjectTracker()
    segmenter = SAM2Segmenter()

    errors = []
    frame_idx = 0
    initialised = False
    print("Processing frames... (press Ctrl+C to stop early)")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            gt = detect_aruco_centroid(frame, args.marker_id)

            if not initialised and gt is not None:
                # Seed tracker with ArUco centroid.
                tracker.request_click(gt[0], gt[1])
                initialised = True

            if initialised:
                result = tracker.process(frame, segmenter, frame_idx, False)
                if result.success and gt is not None:
                    err = np.sqrt((result.center_x - gt[0]) ** 2 + (result.center_y - gt[1]) ** 2)
                    errors.append({
                        "frame": frame_idx,
                        "tracked_cx": result.center_x,
                        "tracked_cy": result.center_y,
                        "gt_cx": gt[0],
                        "gt_cy": gt[1],
                        "error_px": float(err),
                    })

            frame_idx += 1
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()

    if not errors:
        print("No tracked frames with ground truth — cannot compute accuracy.")
        return

    errs = np.array([e["error_px"] for e in errors])
    print(f"\n=== Tracking accuracy ({len(errors)} frames) ===")
    print(f"  Mean error:  {errs.mean():.2f} px")
    print(f"  Median:      {np.median(errs):.2f} px")
    print(f"  P95:         {np.percentile(errs, 95):.2f} px")
    print(f"  Max:         {errs.max():.2f} px")

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)
    out_path = output_dir / f"tracking_accuracy_{int(time.time())}.json"
    with open(out_path, "w") as f:
        json.dump({"errors": errors, "summary": {
            "mean_px": float(errs.mean()),
            "median_px": float(np.median(errs)),
            "p95_px": float(np.percentile(errs, 95)),
            "max_px": float(errs.max()),
        }}, f, indent=2)
    print(f"Saved to {out_path}")


if __name__ == "__main__":
    main()
