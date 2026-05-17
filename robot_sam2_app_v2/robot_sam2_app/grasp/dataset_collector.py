"""
Automatic grasp dataset collector.

Records successful and failed grasp attempts with:
  - RGB frame at grasp time
  - Depth frame (if available)
  - 3D grasp pose
  - Joint angles
  - Outcome (success / fail)

Saves to JSONL + images for later use with GraspNet fine-tuning.
"""
from __future__ import annotations

import json
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import cv2
import numpy as np


@dataclass
class GraspRecord:
    timestamp: float
    outcome: str              # "success" | "fail" | "unknown"
    position_3d: list[float]  # [x, y, z] metres, robot base frame
    approach_axis: list[float]
    joint_ticks: dict[str, int]
    grasp_quality: float
    image_path: str           # relative path to saved RGB image
    depth_path: str           # relative path to saved depth .npy (empty if no depth)


class GraspDatasetCollector:
    """
    Records grasp attempts automatically.

    Usage:
        collector = GraspDatasetCollector("dataset_raw/grasps")
        # Before grasp:
        record_id = collector.start_attempt(grasp_pose, joint_ticks, frame, depth_m)
        # After grasp:
        collector.finish_attempt(record_id, success=True)
    """

    def __init__(self, output_dir: str | Path = "dataset_raw/grasps"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._jsonl_path = self.output_dir / "records.jsonl"
        self._pending: dict[int, dict] = {}
        self._counter = 0

    def start_attempt(
        self,
        grasp_pose,           # GraspPose3D
        joint_ticks: dict[str, int],
        frame_bgr: np.ndarray,
        depth_m: Optional[np.ndarray] = None,
        quality: float = 0.0,
    ) -> int:
        """
        Save the pre-grasp snapshot and return a record ID.
        The caller must call finish_attempt() with the same ID after the grasp.
        """
        rec_id = self._counter
        self._counter += 1
        ts = time.time()

        # Save RGB image.
        img_name = f"grasp_{rec_id:06d}_rgb.jpg"
        img_path = self.output_dir / img_name
        cv2.imwrite(str(img_path), frame_bgr)

        # Save depth if provided.
        depth_name = ""
        if depth_m is not None:
            depth_name = f"grasp_{rec_id:06d}_depth.npy"
            np.save(str(self.output_dir / depth_name), depth_m)

        self._pending[rec_id] = {
            "timestamp":     ts,
            "outcome":       "unknown",
            "position_3d":   list(grasp_pose.position_base),
            "approach_axis": list(grasp_pose.approach_axis),
            "joint_ticks":   dict(joint_ticks),
            "grasp_quality": float(quality),
            "image_path":    img_name,
            "depth_path":    depth_name,
        }
        return rec_id

    def finish_attempt(self, record_id: int, success: bool) -> None:
        """Mark the attempt as succeeded or failed and write to JSONL."""
        if record_id not in self._pending:
            return
        rec = self._pending.pop(record_id)
        rec["outcome"] = "success" if success else "fail"
        with open(self._jsonl_path, "a") as f:
            f.write(json.dumps(rec) + "\n")

    def load_records(self) -> list[dict]:
        """Load all completed records from the JSONL file."""
        records = []
        if self._jsonl_path.exists():
            with open(self._jsonl_path) as f:
                for line in f:
                    line = line.strip()
                    if line:
                        records.append(json.loads(line))
        return records

    def summary(self) -> dict:
        """Return a summary of success/fail counts."""
        records = self.load_records()
        n_success = sum(1 for r in records if r["outcome"] == "success")
        n_fail    = sum(1 for r in records if r["outcome"] == "fail")
        return {
            "total":   len(records),
            "success": n_success,
            "fail":    n_fail,
            "rate":    n_success / max(1, n_success + n_fail),
        }
