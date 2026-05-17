"""
Step 14 — Structured logger and debug image writer.

Two responsibilities:
  1. StructuredLogger  — timestamped, phase-tagged log records to stdout and/or file
  2. DebugImageWriter  — saves annotated frames to an output directory

Usage:
    from utils.logger import StructuredLogger, DebugImageWriter
    import numpy as np

    log = StructuredLogger(name="grasp", log_file="outputs/run.log")
    log.info("CENTERING", "centroid_norm=(-0.12, 0.03)")
    log.warn("APPROACHING", "elbow at limit")

    writer = DebugImageWriter(out_dir="outputs/debug_frames")
    frame  = np.zeros((480, 640, 3), dtype=np.uint8)
    writer.save(frame, tag="centering", step=42)
"""

from __future__ import annotations

import datetime
import sys
from pathlib import Path
from typing import Optional

import numpy as np


# ------------------------------------------------------------------
# Structured logger
# ------------------------------------------------------------------

class StructuredLogger:
    """
    Minimal structured logger.

    Each record: ``[HH:MM:SS.mmm] [LEVEL] [phase] message``

    Args:
        name:     logger name (included in file output header)
        log_file: optional path — appended if it exists, created otherwise
        verbose:  if False, DEBUG records are suppressed
    """

    LEVELS = {"DEBUG": 0, "INFO": 1, "WARN": 2, "ERROR": 3}

    def __init__(
        self,
        name:     str = "robot",
        log_file: Optional[str | Path] = None,
        verbose:  bool = True,
    ) -> None:
        self.name    = name
        self.verbose = verbose
        self._fh     = None

        if log_file:
            p = Path(log_file)
            p.parent.mkdir(parents=True, exist_ok=True)
            self._fh = p.open("a", encoding="utf-8")
            self._fh.write(
                f"\n{'='*60}\n"
                f"Session start: {datetime.datetime.now().isoformat()}\n"
                f"Logger: {name}\n"
                f"{'='*60}\n"
            )

    def debug(self, phase: str, message: str) -> None:
        if self.verbose:
            self._write("DEBUG", phase, message)

    def info(self, phase: str, message: str) -> None:
        self._write("INFO", phase, message)

    def warn(self, phase: str, message: str) -> None:
        self._write("WARN", phase, message)

    def error(self, phase: str, message: str) -> None:
        self._write("ERROR", phase, message)

    def close(self) -> None:
        if self._fh:
            self._fh.close()
            self._fh = None

    def __enter__(self) -> "StructuredLogger":
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # ------------------------------------------------------------------

    def _write(self, level: str, phase: str, message: str) -> None:
        now = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{now}] [{level:5s}] [{phase}] {message}"
        print(line, file=sys.stdout)
        if self._fh:
            self._fh.write(line + "\n")
            self._fh.flush()


# ------------------------------------------------------------------
# Debug image writer
# ------------------------------------------------------------------

class DebugImageWriter:
    """
    Save annotated frames to disk for offline review.

    Files are named ``<step:06d>_<tag>.jpg`` so they sort chronologically.

    Args:
        out_dir:  directory for saved frames (created if absent)
        enabled:  set False to disable all writes (zero overhead in prod)
        quality:  JPEG quality 1-100
    """

    def __init__(
        self,
        out_dir: str | Path = "outputs/debug_frames",
        enabled: bool = True,
        quality: int  = 85,
    ) -> None:
        self.enabled = enabled
        self.quality = quality
        self._dir    = Path(out_dir)
        if enabled:
            self._dir.mkdir(parents=True, exist_ok=True)
        self._step = 0

    def save(
        self,
        frame: np.ndarray,
        tag:   str = "frame",
        step:  Optional[int] = None,
    ) -> Optional[Path]:
        """
        Write frame to disk.

        Args:
            frame: H×W×3 uint8 BGR image
            tag:   short label appended to filename
            step:  explicit step index; auto-increments if None

        Returns:
            Path written, or None if disabled or cv2 unavailable.
        """
        if not self.enabled:
            return None

        idx = step if step is not None else self._step
        self._step += 1

        path = self._dir / f"{idx:06d}_{tag}.jpg"

        try:
            import cv2  # type: ignore[import-untyped]
            cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        except ImportError:
            _write_ppm(frame, path.with_suffix(".ppm"))
            path = path.with_suffix(".ppm")

        return path

    def annotate_and_save(
        self,
        frame:      np.ndarray,
        lines:      list[str],
        tag:        str = "frame",
        step:       Optional[int] = None,
        color:      tuple[int, int, int] = (0, 255, 0),
    ) -> Optional[Path]:
        """
        Overlay text lines on frame, then save.

        Falls back gracefully if cv2 is unavailable (saves without text).
        """
        out = frame.copy()
        try:
            import cv2  # type: ignore[import-untyped]
            for i, line in enumerate(lines):
                cv2.putText(
                    out, line, (10, 25 + i * 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA,
                )
        except ImportError:
            pass   # save plain frame if cv2 missing

        return self.save(out, tag=tag, step=step)

    def close(self) -> None:
        pass   # nothing to close; kept for symmetry with StructuredLogger

    def __enter__(self) -> "DebugImageWriter":
        return self

    def __exit__(self, *_) -> None:
        self.close()


# ------------------------------------------------------------------
# Minimal PPM writer (no dependencies)
# ------------------------------------------------------------------

def _write_ppm(frame: np.ndarray, path: Path) -> None:
    """Write an H×W×3 uint8 RGB array as a binary PPM file."""
    H, W = frame.shape[:2]
    # cv2 uses BGR; flip to RGB for PPM
    rgb = frame[:, :, ::-1] if frame.shape[2] == 3 else frame
    header = f"P6\n{W} {H}\n255\n".encode()
    path.write_bytes(header + rgb.tobytes())
