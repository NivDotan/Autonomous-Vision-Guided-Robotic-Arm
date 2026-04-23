from __future__ import annotations

from contextlib import nullcontext

import cv2
import numpy as np
import torch

from ..config import SAM2_CHECKPOINT, SAM2_MODEL_CFG


class SAM2Segmenter:
    """Point/box prompted SAM2 segmentation for one frame."""

    def __init__(self, checkpoint: str = SAM2_CHECKPOINT, model_cfg: str = SAM2_MODEL_CFG):
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"SAM2 device: {self.device}")
        model = build_sam2(model_cfg, checkpoint, device=self.device)
        self.predictor = SAM2ImagePredictor(model)

    def _contexts(self):
        autocast = torch.autocast("cuda", dtype=torch.bfloat16) if self.device == "cuda" else nullcontext()
        return torch.inference_mode(), autocast

    def segment_bbox(self, frame_bgr, point_xy: tuple[int, int], box_xyxy=None):
        image_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        point = np.array([[point_xy[0], point_xy[1]]], dtype=np.float32)
        label = np.array([1], dtype=np.int32)
        box = np.array(box_xyxy, dtype=np.float32) if box_xyxy is not None else None

        infer_ctx, autocast_ctx = self._contexts()
        with infer_ctx, autocast_ctx:
            self.predictor.set_image(image_rgb)
            masks, _, _ = self.predictor.predict(
                point_coords=point,
                point_labels=label,
                box=box,
                multimask_output=False,
            )

        ys, xs = np.where(masks[0])
        if len(xs) == 0:
            return None
        x0, x1, y0, y1 = xs.min(), xs.max(), ys.min(), ys.max()
        return int(x0), int(y0), int(x1 - x0), int(y1 - y0)

