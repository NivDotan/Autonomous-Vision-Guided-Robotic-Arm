from __future__ import annotations

import torch
from PIL import Image


class VQADetector:
    """Florence-2 REFERRING_EXPRESSION_COMPREHENSION → (x0, y0, x1, y1) bbox.

    Lazy-loaded on first call to detect_bbox(). Uses ~0.8 GB VRAM (float16).
    """

    def __init__(self, model_name: str, device: str = "cuda"):
        self._model_name = model_name
        self._device = device
        self._model = None
        self._processor = None

    def _ensure_loaded(self) -> None:
        if self._model is not None:
            return
        from transformers import AutoModelForCausalLM, AutoProcessor
        print(f"[VQA] Loading {self._model_name} …")
        self._processor = AutoProcessor.from_pretrained(
            self._model_name, trust_remote_code=True)
        self._model = AutoModelForCausalLM.from_pretrained(
            self._model_name, torch_dtype=torch.float16,
            trust_remote_code=True).to(self._device).eval()
        print(f"[VQA] {self._model_name} ready")

    def detect_bbox(self, frame_bgr, query: str) -> tuple[int, int, int, int] | None:
        """Return (x0, y0, x1, y1) for the described object, or None."""
        if frame_bgr is None:
            return None
        self._ensure_loaded()
        try:
            image = Image.fromarray(frame_bgr[..., ::-1])  # BGR → RGB
            task = "<REFERRING_EXPRESSION_COMPREHENSION>"
            inputs = self._processor(
                text=task + query, images=image, return_tensors="pt",
            ).to(self._device, torch.float16)
            with torch.no_grad():
                ids = self._model.generate(
                    **inputs, max_new_tokens=64, do_sample=False, num_beams=3)
            text_out = self._processor.batch_decode(ids, skip_special_tokens=False)[0]
            parsed = self._processor.post_process_generation(
                text_out, task=task, image_size=(image.width, image.height))
            bboxes = parsed.get(task, {}).get("bboxes", [])
            if not bboxes:
                print(f"[VQA] '{query}' — nothing found")
                return None
            x0, y0, x1, y1 = (int(v) for v in bboxes[0])
            print(f"[VQA] '{query}' → ({x0}, {y0}, {x1}, {y1})")
            return x0, y0, x1, y1
        except Exception as exc:
            print(f"[VQA] error: {exc}")
            return None
