"""
Extended Kalman Filter for 2D object tracking.

State vector: [x, y, vx, vy]  (pixels + pixels/frame)
Measurement:  [x, y]           (object centroid from CSRT)

Reduces jitter in the tracked centroid and provides velocity prediction
for frames where CSRT fails (occlusion, fast motion).
"""
from __future__ import annotations

import numpy as np


class KalmanObjectTracker:
    """
    Linear Kalman Filter for 2D centroid tracking.

    The filter state is [cx, cy, vx, vy].
    Measurements are centroid pixel positions from the CSRT tracker.
    """

    def __init__(
        self,
        dt: float = 1.0 / 30.0,
        process_noise: float = 5.0,     # Q diagonal scaling
        measurement_noise: float = 10.0, # R diagonal scaling
    ):
        # State: [cx, cy, vx, vy]
        self._x = np.zeros(4)
        self._P = np.eye(4) * 100.0  # large initial uncertainty

        # State transition matrix (constant velocity model).
        self._F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # Measurement matrix (observe cx, cy only).
        self._H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Process noise covariance.
        q = process_noise
        self._Q = np.diag([q, q, q * 2, q * 2])

        # Measurement noise covariance.
        r = measurement_noise
        self._R = np.diag([r, r])

        self._initialised = False

    def reset(self, cx: float, cy: float) -> None:
        self._x = np.array([cx, cy, 0.0, 0.0])
        self._P = np.eye(4) * 100.0
        self._initialised = True

    def predict(self) -> tuple[float, float]:
        """Predict state forward one timestep without a measurement."""
        if not self._initialised:
            return 0.0, 0.0
        self._x = self._F @ self._x
        self._P = self._F @ self._P @ self._F.T + self._Q
        return float(self._x[0]), float(self._x[1])

    def update(self, cx: float, cy: float) -> tuple[float, float]:
        """
        Incorporate a measurement (cx, cy) and return the filtered estimate.
        Initialises the filter on the first call.
        """
        if not self._initialised:
            self.reset(cx, cy)
            return cx, cy

        # Predict.
        self._x = self._F @ self._x
        self._P = self._F @ self._P @ self._F.T + self._Q

        # Update.
        z = np.array([cx, cy])
        y = z - self._H @ self._x
        S = self._H @ self._P @ self._H.T + self._R
        K = self._P @ self._H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ self._H) @ self._P

        return float(self._x[0]), float(self._x[1])

    @property
    def velocity(self) -> tuple[float, float]:
        """Estimated pixel velocity (vx, vy) in pixels/frame."""
        return float(self._x[2]), float(self._x[3])

    @property
    def initialised(self) -> bool:
        return self._initialised
