from __future__ import annotations

import cv2
import numpy as np

from ..config import RFDETR_CONFIDENCE, RFDETR_MODEL_SIZE
from ..utils import clamp, normalize_class_name


class RFDETRTargetSelector:
    """Lazy RF-DETR detector used only when the user requests a target."""

    def __init__(self, model_size: str = RFDETR_MODEL_SIZE, confidence: float = RFDETR_CONFIDENCE):
        self.model_size = model_size
        self.confidence = confidence
        self.model = None

    def _ensure_model(self) -> bool:
        if self.model is not None:
            return True
        try:
            from rfdetr import RFDETRLarge, RFDETRMedium, RFDETRNano, RFDETRSmall
        except Exception as exc:
            print(f"RF-DETR unavailable: {exc}")
            print("Install it with: python -m pip install rfdetr")
            return False

        classes = {
            "nano": RFDETRNano,
            "small": RFDETRSmall,
            "medium": RFDETRMedium,
            "large": RFDETRLarge,
        }
        cls = classes.get(self.model_size.lower(), RFDETRNano)
        print(f"Loading RF-DETR {self.model_size}...")
        self.model = cls()
        print("RF-DETR ready.")
        return True

    @staticmethod
    def _matches(label, target) -> bool:
        label_norm = normalize_class_name(label)
        target_norm = normalize_class_name(target)
        return target_norm == label_norm or target_norm in label_norm or label_norm in target_norm

    def select_bbox(self, frame_bgr, target_name: str):
        if frame_bgr is None or not self._ensure_model():
            return None

        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        detections = self.model.predict(rgb, threshold=self.confidence)
        boxes = getattr(detections, "xyxy", None)
        if boxes is None or len(boxes) == 0:
            print(f"RF-DETR found no objects above confidence {self.confidence}.")
            return None

        data = getattr(detections, "data", None)
        names = data.get("class_name") if isinstance(data, dict) else None
        if names is None:
            names = [str(cid) for cid in getattr(detections, "class_id", [])]
        confidence = getattr(detections, "confidence", None)
        if confidence is None:
            confidence = np.ones(len(boxes), dtype=np.float32)

        best_index = None
        best_score = -1.0
        seen = []
        for index, (box, label, score) in enumerate(zip(boxes, names, confidence)):
            label = str(label)
            seen.append(label)
            if self._matches(label, target_name) and float(score) > best_score:
                best_index = index
                best_score = float(score)

        if best_index is None:
            labels = ", ".join(sorted(set(seen))) if seen else "none"
            print(f"RF-DETR did not find '{target_name}'. Seen: {labels}")
            return None

        h, w, _ = frame_bgr.shape
        x0, y0, x1, y1 = map(float, boxes[best_index])
        bbox = (
            int(clamp(round(x0), 0, w - 1)),
            int(clamp(round(y0), 0, h - 1)),
            int(clamp(round(x1), 1, w)),
            int(clamp(round(y1), 1, h)),
        )
        print(f"RF-DETR selected {names[best_index]} for '{target_name}' at {bbox}")
        return bbox

