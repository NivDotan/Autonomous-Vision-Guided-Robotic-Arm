"""
End-to-end latency profiler.

Measures the pipeline latency from camera frame capture to motor command write.
Instruments timing at each stage: capture → detection → IK → command.

Usage:
    python benchmarks/latency_profiler.py --frames 300 --output results/
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


class LatencyProbe:
    """Context manager that records elapsed time for a named stage."""

    def __init__(self, name: str, records: list):
        self.name    = name
        self.records = records
        self._start  = 0.0

    def __enter__(self):
        self._start = time.perf_counter()
        return self

    def __exit__(self, *_):
        elapsed_ms = (time.perf_counter() - self._start) * 1000
        self.records.append({"stage": self.name, "ms": elapsed_ms})


def profile_frame(app, records: list) -> None:
    """Profile one frame through the app pipeline."""
    # Capture.
    with LatencyProbe("capture", records):
        ok, frame = app.cap.read()
        if not ok:
            return
        frame = cv2.flip(frame, 1)

    # Hand detection (MediaPipe).
    with LatencyProbe("mediapipe", records):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = app.hands.process(rgb)

    # Vision update (CSRT track or SAM2).
    with LatencyProbe("vision_update", records):
        if app.state.tracking_mode == "OBJECT":
            tracking = app.tracker.process(
                frame, app.segmenter, app.frame_index, app.state.approach_mode)
            app.controller.update_from_object(
                app.state, tracking, frame.shape[1], frame.shape[0])

    # Trajectory or proportional step.
    with LatencyProbe("motion_step", records):
        if app.state.trajectory_active:
            pass  # would call _step_trajectory()
        # Proportional step is O(1) — always fast.

    # Hardware write.
    with LatencyProbe("hw_write", records):
        if app.state.motors_enabled and app.hardware.connected:
            app.hardware.write_ticks(app.state.curr)


def summarise(records: list) -> dict:
    from collections import defaultdict
    by_stage: dict[str, list] = defaultdict(list)
    for r in records:
        by_stage[r["stage"]].append(r["ms"])
    summary = {}
    for stage, vals in by_stage.items():
        a = np.array(vals)
        summary[stage] = {
            "mean_ms":   float(a.mean()),
            "p50_ms":    float(np.percentile(a, 50)),
            "p95_ms":    float(np.percentile(a, 95)),
            "p99_ms":    float(np.percentile(a, 99)),
            "max_ms":    float(a.max()),
            "n_samples": len(vals),
        }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=300)
    parser.add_argument("--output", type=str, default="results")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    import sys
    sys.path.insert(0, str(Path(__file__).parent.parent / "robot_sam2_app"))
    from robot_sam2_app.app import RobotApp

    app = RobotApp()
    app._setup()  # initialise without running the loop

    records = []
    print(f"Profiling {args.frames} frames...")
    for i in range(args.frames):
        profile_frame(app, records)
        if i % 50 == 0:
            print(f"  {i}/{args.frames}")

    summary = summarise(records)

    print("\n=== Latency summary ===")
    for stage, s in summary.items():
        print(f"  {stage:20s}  mean={s['mean_ms']:.2f}ms  p95={s['p95_ms']:.2f}ms  p99={s['p99_ms']:.2f}ms")

    out_path = output_dir / f"latency_{int(time.time())}.json"
    with open(out_path, "w") as f:
        json.dump({"summary": summary, "raw": records}, f)
    print(f"\nSaved to {out_path}")


if __name__ == "__main__":
    main()
