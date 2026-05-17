"""
Unscented Kalman Filter for 3D pose tracking.

State vector: [x, y, z, vx, vy, vz]  (metres + metres/s, robot base frame)
Measurement:  [x, y, z]               (from GraspPlanner.plan())

Used to smooth 3D grasp pose estimates from the depth camera across frames,
reducing the effect of noisy depth readings and occasional segmentation failures.
"""
from __future__ import annotations

import numpy as np


class UKF3DPoseTracker:
    """
    UKF for tracking a 3D object position (x, y, z) and velocity.

    Uses the standard sigma-point approach (Julier & Uhlmann, 1997) with
    the Merwe scaled sigma-point scheme.
    """

    def __init__(
        self,
        dt: float = 1.0 / 30.0,
        alpha: float = 1e-3,
        beta: float = 2.0,
        kappa: float = 0.0,
        process_std: float = 0.02,     # m/s² position noise
        measurement_std: float = 0.015, # metres depth measurement noise
    ):
        self._dt = dt
        self._n  = 6   # state dimension: [x, y, z, vx, vy, vz]

        # UKF parameters.
        lam = alpha ** 2 * (self._n + kappa) - self._n
        self._lam = lam
        self._alpha = alpha
        self._beta  = beta

        # Sigma-point weights.
        n = self._n
        self._Wm = np.full(2 * n + 1, 1.0 / (2 * (n + lam)))
        self._Wm[0] = lam / (n + lam)
        self._Wc = self._Wm.copy()
        self._Wc[0] = lam / (n + lam) + (1 - alpha ** 2 + beta)

        # Covariances.
        q = process_std ** 2
        self._Q = np.diag([q, q, q, q * 4, q * 4, q * 4])
        r = measurement_std ** 2
        self._R = np.diag([r, r, r])

        # State and covariance.
        self._x = np.zeros(6)
        self._P = np.eye(6) * 1.0
        self._initialised = False

    # State transition: constant velocity.
    def _f(self, x: np.ndarray) -> np.ndarray:
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = self._dt
        return F @ x

    # Measurement function: observe position only.
    @staticmethod
    def _h(x: np.ndarray) -> np.ndarray:
        return x[:3]

    def reset(self, pos: tuple[float, float, float]) -> None:
        self._x[:3] = pos
        self._x[3:] = 0.0
        self._P = np.eye(6) * 0.1
        self._initialised = True

    def predict(self) -> tuple[float, float, float]:
        if not self._initialised:
            return 0.0, 0.0, 0.0
        sigmas = self._sigma_points()
        sigmas_f = np.array([self._f(s) for s in sigmas])
        self._x = np.dot(self._Wm, sigmas_f)
        diff = sigmas_f - self._x
        self._P = sum(self._Wc[i] * np.outer(diff[i], diff[i])
                      for i in range(2 * self._n + 1)) + self._Q
        return tuple(self._x[:3].tolist())

    def update(
        self,
        pos: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        """Incorporate a 3D position measurement and return filtered estimate."""
        if not self._initialised:
            self.reset(pos)
            return pos

        self.predict()

        sigmas = self._sigma_points()
        sigmas_h = np.array([self._h(s) for s in sigmas])
        z_pred = np.dot(self._Wm, sigmas_h)

        diff_x = sigmas - self._x
        diff_z = sigmas_h - z_pred
        S = sum(self._Wc[i] * np.outer(diff_z[i], diff_z[i])
                for i in range(2 * self._n + 1)) + self._R
        Pxz = sum(self._Wc[i] * np.outer(diff_x[i], diff_z[i])
                  for i in range(2 * self._n + 1))

        K = Pxz @ np.linalg.inv(S)
        z = np.array(pos)
        self._x = self._x + K @ (z - z_pred)
        self._P = self._P - K @ S @ K.T

        return tuple(self._x[:3].tolist())

    def _sigma_points(self) -> np.ndarray:
        n = self._n
        lam = self._lam
        c = np.sqrt(n + lam)
        try:
            L = np.linalg.cholesky((n + lam) * self._P)
        except np.linalg.LinAlgError:
            L = np.linalg.cholesky((n + lam) * (self._P + 1e-6 * np.eye(n)))
        sigmas = np.zeros((2 * n + 1, n))
        sigmas[0] = self._x
        for i in range(n):
            sigmas[i + 1]     = self._x + L[:, i]
            sigmas[i + 1 + n] = self._x - L[:, i]
        return sigmas

    @property
    def position(self) -> tuple[float, float, float]:
        return tuple(self._x[:3].tolist())

    @property
    def velocity(self) -> tuple[float, float, float]:
        return tuple(self._x[3:].tolist())

    @property
    def initialised(self) -> bool:
        return self._initialised
