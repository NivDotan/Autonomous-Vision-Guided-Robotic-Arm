from __future__ import annotations

import torch
from PIL import Image


class VQADetector:
    """Grounding DINO → (x0, y0, x1, y1) bbox from natural-language description.

    Lazy-loaded on first detect_bbox() call (~500 MB VRAM, float32).
    Text prompt is auto-terminated with '.' as required by Grounding DINO.
    """

    def __init__(self, model_name: str, device: str = "cuda"):
        self._model_name = model_name
        self._device = device
        self._model = None
        self._processor = None

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        print(f"[VQA] Loading {self._model_name} …")
        self._processor = AutoProcessor.from_pretrained(self._model_name)
        self._model = AutoModelForZeroShotObjectDetection.from_pretrained(
            self._model_name).to(self._device).eval()
        print(f"[VQA] {self._model_name} ready")

    def detect_bbox(self, frame_bgr, query: str) -> tuple[int, int, int, int] | None:
        """Return (x0, y0, x1, y1) for the described object, or None."""
        if frame_bgr is None:
            return None
        self._ensure_loaded()
        try:
            image = Image.fromarray(frame_bgr[..., ::-1])  # BGR → RGB
            text = query.rstrip(".") + "."  # Grounding DINO requires period-terminated text
            inputs = self._processor(
                images=image, text=text, return_tensors="pt"
            ).to(self._device)
            with torch.no_grad():
                outputs = self._model(**inputs)
            results = self._processor.post_process_grounded_object_detection(
                outputs,
                inputs.input_ids,
                target_sizes=[image.size[::-1]],
            )
            boxes  = results[0]["boxes"]
            scores = results[0]["scores"]
            mask   = scores > 0.3
            boxes  = boxes[mask]
            scores = scores[mask]
            if len(boxes) == 0:
                print(f"[VQA] '{query}' — nothing found")
                return None
            best = int(scores.argmax())
            x0, y0, x1, y1 = (int(v) for v in boxes[best].tolist())
            print(f"[VQA] '{query}' → ({x0}, {y0}, {x1}, {y1})  score={scores[best]:.2f}")
            return x0, y0, x1, y1
        except Exception as exc:
            print(f"[VQA] error: {exc}")
            return None
