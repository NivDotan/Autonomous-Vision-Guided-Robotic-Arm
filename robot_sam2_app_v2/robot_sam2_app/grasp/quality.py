"""
Grasp quality metrics.

Implements the ε-metric (epsilon quality) from grasp wrench space analysis:
  ε = radius of the largest ball fitting inside the convex hull of the
      wrench primitives (Grasp Quality Measures, Roa & Suárez 2015).

For real sensor feedback (force/torque), also computes a force-closure score.
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass


@dataclass
class GraspQuality:
    epsilon: float       # wrench-space epsilon metric (0 = no closure, >0 = good)
    volume: float        # GWS convex hull volume (larger = more robust)
    force_closure: bool  # True if epsilon > 0
    sensor_load: float   # gripper motor load at time of measurement (normalised)


class GraspQualityAnalyzer:
    """
    Computes grasp quality metrics from contact geometry and sensor feedback.

    Parameters
    ----------
    friction_coeff : Coulomb friction coefficient (typically 0.5–0.8 for rubber).
    n_cone_edges   : Number of edges to discretise each friction cone.
    """

    def __init__(self, friction_coeff: float = 0.6, n_cone_edges: int = 8):
        self.mu = friction_coeff
        self.n  = n_cone_edges

    def from_contacts(
        self,
        contact_points: list[tuple[float, float, float]],
        contact_normals: list[tuple[float, float, float]],
        object_centroid: tuple[float, float, float] = (0, 0, 0),
        sensor_load: float = 0.0,
    ) -> GraspQuality:
        """
        Computes ε-metric from contact points and inward normals.

        contact_points  : list of (x, y, z) contact positions (metres)
        contact_normals : list of (nx, ny, nz) inward normal unit vectors
        object_centroid : object CoM for moment arm calculation
        sensor_load     : normalised gripper load [0..1] from hardware
        """
        wrench_primitives = self._build_wrench_primitives(
            contact_points, contact_normals, object_centroid)

        if len(wrench_primitives) < 4:
            return GraspQuality(0.0, 0.0, False, sensor_load)

        eps, vol = self._epsilon_metric(np.array(wrench_primitives))
        return GraspQuality(eps, vol, eps > 1e-6, sensor_load)

    def from_sensor_load(self, raw_load: int, max_load: int = 1000) -> GraspQuality:
        """
        Lightweight quality estimate from gripper load sensor alone.
        Used when contact geometry is not available.
        """
        normalised = min(1.0, abs(raw_load) / max(1, max_load))
        # Heuristic: loads between 20–80% of max indicate a stable grasp.
        in_sweet_spot = 0.2 <= normalised <= 0.8
        eps = normalised * 0.5 if in_sweet_spot else 0.0
        return GraspQuality(eps, 0.0, in_sweet_spot, normalised)

    # ── Internal ──────────────────────────────────────────────────────────────

    def _build_wrench_primitives(
        self,
        points: list,
        normals: list,
        centroid: tuple,
    ) -> list:
        """Build 6D wrench primitives [f; τ] for each friction cone edge."""
        c = np.array(centroid)
        primitives = []
        for p, n in zip(points, normals):
            p_arr = np.array(p)
            n_arr = np.array(n, dtype=float)
            n_arr /= np.linalg.norm(n_arr) + 1e-9

            # Build orthonormal basis for friction cone.
            t1, t2 = self._tangent_vectors(n_arr)

            for k in range(self.n):
                angle = 2 * np.pi * k / self.n
                # Linearised friction cone edge.
                f = n_arr + self.mu * (np.cos(angle) * t1 + np.sin(angle) * t2)
                f /= np.linalg.norm(f) + 1e-9
                tau = np.cross(p_arr - c, f)
                primitives.append(np.concatenate([f, tau]))
        return primitives

    @staticmethod
    def _tangent_vectors(n: np.ndarray):
        """Return two unit vectors orthogonal to n."""
        if abs(n[0]) < 0.9:
            t1 = np.cross(n, [1, 0, 0])
        else:
            t1 = np.cross(n, [0, 1, 0])
        t1 /= np.linalg.norm(t1) + 1e-9
        t2 = np.cross(n, t1)
        t2 /= np.linalg.norm(t2) + 1e-9
        return t1, t2

    @staticmethod
    def _epsilon_metric(W: np.ndarray) -> tuple[float, float]:
        """
        Approximate ε-metric as the minimum distance from the origin to
        the convex hull of wrench primitives W (n×6).
        Uses a simple LP approximation via min-norm point.
        """
        try:
            from scipy.spatial import ConvexHull
            from scipy.optimize import linprog

            # Check if origin is inside the convex hull.
            n, d = W.shape
            # Minimise ||Wλ||² s.t. Σλᵢ = 1, λᵢ >= 0.
            # Equivalent to finding the minimum-norm point in the convex hull.
            # Use a simple iterative approach (Frank-Wolfe).
            lam = np.ones(n) / n
            for _ in range(100):
                w = W.T @ lam
                dists = ((W - w) ** 2).sum(axis=1)
                i_min = np.argmin(dists)
                e_i = np.zeros(n)
                e_i[i_min] = 1.0
                step = 2.0 / (_ + 2)
                lam = (1 - step) * lam + step * e_i

            min_norm_pt = W.T @ lam
            eps = float(np.linalg.norm(min_norm_pt))

            # Volume (approximate as determinant of covariance).
            vol = float(np.linalg.det(np.cov(W.T) + 1e-9 * np.eye(d)))
            return eps, abs(vol)
        except ImportError:
            # scipy not available — return simple norm proxy.
            eps = float(np.linalg.norm(W.mean(axis=0)))
            return eps, 0.0
