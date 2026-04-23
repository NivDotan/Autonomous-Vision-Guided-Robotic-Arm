"""
6-DOF grasp pose estimation from SAM2 mask + aligned depth frame.

Pipeline:
  1. Extract depth pixels inside the SAM2 segmentation mask
  2. Filter outlier depths (< 0.05 m or > 2.0 m)
  3. Deproject each valid pixel → 3D point cloud (camera frame)
  4. Compute centroid as grasp position
  5. PCA on point cloud → principal axis = grasp approach direction
  6. Transform centroid + axis to robot base frame via CoordTransform
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

from .scene_3d import CoordTransform
from .depth_perception import RealSenseDepth


@dataclass
class GraspPose3D:
    position_base: tuple[float, float, float]   # (x, y, z) metres, robot base frame
    approach_axis: tuple[float, float, float]   # unit vector, base frame
    quality: float                               # 0..1, higher = more depth points


class GraspPlanner:
    """
    Converts a segmentation mask + depth frame into a 3D grasp pose.

    Parameters
    ----------
    depth_cam      : RealSenseDepth (or mock) providing deproject_pixel().
    coord_transform: CoordTransform from camera to robot base frame.
    min_depth_m    : ignore pixels closer than this (usually noise/reflection).
    max_depth_m    : ignore pixels further than this (out of robot reach).
    min_points     : minimum valid 3D points needed to produce a pose.
    """

    def __init__(
        self,
        depth_cam: RealSenseDepth,
        coord_transform: CoordTransform,
        min_depth_m: float = 0.05,
        max_depth_m: float = 2.0,
        min_points: int = 10,
    ):
        self._cam   = depth_cam
        self._tf    = coord_transform
        self._d_min = min_depth_m
        self._d_max = max_depth_m
        self._min_pts = min_points

    def plan(
        self,
        mask_hw: np.ndarray,   # bool HxW, True = object pixels (from SAM2/CSRT)
        depth_m: np.ndarray,   # float32 HxW, metric depth (from RealSenseDepth)
    ) -> GraspPose3D | None:
        """
        Returns a GraspPose3D or None if there are not enough valid depth points.
        """
        if mask_hw is None or depth_m is None:
            return None

        # Step 1: extract valid depth pixels inside the mask.
        ys, xs = np.where(mask_hw)
        if len(ys) == 0:
            return None

        depths = depth_m[ys, xs]
        valid  = (depths > self._d_min) & (depths < self._d_max)
        if valid.sum() < self._min_pts:
            return None

        xs_v = xs[valid].astype(int)
        ys_v = ys[valid].astype(int)
        ds_v = depths[valid].astype(float)

        # Step 2: deproject each pixel to 3D (camera frame).
        # Vectorised for performance — batch-compute using pinhole if intrinsics
        # are not available, or fall back to per-pixel deproject.
        if hasattr(self._cam, 'intrinsics') and self._cam.intrinsics is not None:
            try:
                import pyrealsense2 as rs
                points_cam = np.array([
                    rs.rs2_deproject_pixel_to_point(
                        self._cam.intrinsics, [float(x), float(y)], float(d))
                    for x, y, d in zip(xs_v, ys_v, ds_v)
                ], dtype=float)
            except Exception:
                points_cam = self._batch_deproject_pinhole(xs_v, ys_v, ds_v)
        else:
            points_cam = self._batch_deproject_pinhole(xs_v, ys_v, ds_v)

        # Step 3: centroid.
        centroid_cam = points_cam.mean(axis=0)

        # Step 4: PCA — principal axis = direction of largest variance.
        centered = points_cam - centroid_cam
        if len(centered) >= 3:
            _, _, Vt = np.linalg.svd(centered, full_matrices=False)
            approach_cam = Vt[0]  # first principal component
        else:
            approach_cam = np.array([0.0, 0.0, -1.0])  # default: downward in cam frame

        # Normalise.
        approach_cam = approach_cam / (np.linalg.norm(approach_cam) + 1e-9)

        # Step 5: transform to robot base frame.
        pos_base    = self._tf.camera_to_base(centroid_cam)
        axis_base   = tuple((self._tf._R @ approach_cam).tolist())

        # Quality metric: fraction of mask pixels that had valid depth, capped at 1.
        quality = min(1.0, valid.sum() / max(1, len(ys)) * 2)

        return GraspPose3D(
            position_base=pos_base,
            approach_axis=axis_base,
            quality=float(quality),
        )

    def _batch_deproject_pinhole(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        ds: np.ndarray,
    ) -> np.ndarray:
        """Simple pinhole deprojection without RealSense intrinsics."""
        fx = fy = 600.0  # approximate — replace with actual intrinsics if available
        cx = self._cam.width / 2.0
        cy = self._cam.height / 2.0
        x = (xs - cx) * ds / fx
        y = (ys - cy) * ds / fy
        return np.stack([x, y, ds], axis=1)

    @staticmethod
    def bbox_to_mask(
        bbox_xywh: tuple[int, int, int, int],
        frame_shape: tuple[int, int],
    ) -> np.ndarray:
        """
        Convert a CSRT tracker bounding box (x, y, w, h) to a boolean mask.
        Used when a full SAM2 mask is not available per-frame.
        """
        h_frame, w_frame = frame_shape[:2]
        mask = np.zeros((h_frame, w_frame), dtype=bool)
        x, y, w, h = [int(v) for v in bbox_xywh]
        x = max(0, x)
        y = max(0, y)
        x2 = min(w_frame, x + w)
        y2 = min(h_frame, y + h)
        mask[y:y2, x:x2] = True
        return mask
