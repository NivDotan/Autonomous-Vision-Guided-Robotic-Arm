"""
Intel RealSense D435 wrapper providing aligned color+depth frames.
Degrades gracefully if pyrealsense2 is not installed or camera is absent.
"""
from __future__ import annotations

import numpy as np

try:
    import pyrealsense2 as rs
    _RS_AVAILABLE = True
except ImportError:
    rs = None
    _RS_AVAILABLE = False


class RealSenseDepth:
    """
    Wraps pyrealsense2 to deliver time-aligned (color, depth_m) frame pairs.

    color_bgr  : uint8 ndarray (H, W, 3) — OpenCV BGR
    depth_m    : float32 ndarray (H, W) — metric depth in metres
    """

    def __init__(self, width: int = 640, height: int = 480, fps: int = 30):
        self.width  = width
        self.height = height
        self.fps    = fps

        self._pipeline  = None
        self._align     = None
        self.intrinsics = None   # rs.intrinsics after connect()

    def connect(self) -> bool:
        """
        Returns True if a RealSense device is found and streaming started.
        Returns False silently if unavailable (app continues without depth).
        """
        if not _RS_AVAILABLE:
            return False
        try:
            self._pipeline = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.color, self.width, self.height,
                              rs.format.bgr8, self.fps)
            cfg.enable_stream(rs.stream.depth, self.width, self.height,
                              rs.format.z16, self.fps)
            profile = self._pipeline.start(cfg)

            # Store camera intrinsics for deprojection.
            color_stream = profile.get_stream(rs.stream.color)
            self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

            # Align depth to color frame.
            self._align = rs.align(rs.stream.color)
            return True
        except Exception as e:
            print(f"[RealSenseDepth] Could not start camera: {e}")
            self._pipeline = None
            return False

    def read_aligned(self) -> tuple[np.ndarray, np.ndarray] | None:
        """
        Returns (color_bgr uint8 HxWx3, depth_m float32 HxW), or None on error.
        Depth values of 0 indicate invalid / out-of-range pixels.
        """
        if self._pipeline is None or self._align is None:
            return None
        try:
            frames = self._pipeline.wait_for_frames(timeout_ms=200)
            aligned = self._align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                return None

            color_bgr = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            # Convert raw uint16 depth units → metres using the device scale.
            depth_scale = (
                self._pipeline.get_active_profile()
                .get_device()
                .first_depth_sensor()
                .get_depth_scale()
            )
            depth_m = depth_raw * depth_scale
            return color_bgr, depth_m
        except Exception as e:
            print(f"[RealSenseDepth] read_aligned error: {e}")
            return None

    def deproject_pixel(
        self,
        u: int,
        v: int,
        depth_m: float,
    ) -> tuple[float, float, float]:
        """
        Back-project a pixel (u, v) at metric depth_m to a 3D camera-frame point.
        Uses rs.rs2_deproject_pixel_to_point when intrinsics are available;
        falls back to simple pinhole approximation otherwise.
        """
        if _RS_AVAILABLE and self.intrinsics is not None:
            point = rs.rs2_deproject_pixel_to_point(
                self.intrinsics, [float(u), float(v)], depth_m)
            return float(point[0]), float(point[1]), float(point[2])
        # Pinhole fallback (assumes fx ≈ fy ≈ 600, cx ≈ W/2, cy ≈ H/2).
        fx = fy = 600.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        return float(x), float(y), float(depth_m)

    def disconnect(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
            self._align = None


class MockRealSenseDepth(RealSenseDepth):
    """
    Drop-in mock for testing without a physical camera.
    Enabled when cfg.MOCK_REALSENSE = True.
    Returns a static gradient depth map (0.3 m → 0.8 m) with a black color frame.
    """

    def connect(self) -> bool:
        ramp = np.linspace(0.3, 0.8, self.width * self.height, dtype=np.float32)
        self._mock_depth = ramp.reshape(self.height, self.width)
        self._mock_color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return True

    def read_aligned(self) -> tuple[np.ndarray, np.ndarray] | None:
        return self._mock_color.copy(), self._mock_depth.copy()

    def deproject_pixel(self, u: int, v: int, depth_m: float) -> tuple[float, float, float]:
        fx = fy = 600.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        x = (u - cx) * depth_m / fx
        y = (v - cy) * depth_m / fy
        return float(x), float(y), float(depth_m)

    def disconnect(self) -> None:
        pass
