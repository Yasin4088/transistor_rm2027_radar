from __future__ import annotations

from typing import Iterable, Tuple

import numpy as np

from .types import Box, Point


def _as_column(values: Iterable[float]) -> np.ndarray:
    return np.asarray(tuple(values), dtype=np.float64).reshape(-1, 1)


class LinearKalman:
    """Small NumPy Kalman filter with explicit predict/correct ordering."""

    def __init__(self, state_size: int, measurement_size: int):
        self.x = np.zeros((state_size, 1), dtype=np.float64)
        self.P = np.eye(state_size, dtype=np.float64) * 10.0
        self.H = np.zeros((measurement_size, state_size), dtype=np.float64)
        self.R = np.eye(measurement_size, dtype=np.float64)
        self.initialized = False

    def correct(self, measurement: Iterable[float]) -> None:
        z = _as_column(measurement)
        innovation = z - self.H @ self.x
        covariance = self.H @ self.P @ self.H.T + self.R
        gain = self.P @ self.H.T @ np.linalg.pinv(covariance)
        self.x = self.x + gain @ innovation
        identity = np.eye(self.P.shape[0], dtype=np.float64)
        # Joseph form keeps covariance positive semi-definite under rounding.
        correction = identity - gain @ self.H
        self.P = correction @ self.P @ correction.T + gain @ self.R @ gain.T

    def mahalanobis(self, measurement: Iterable[float]) -> float:
        z = _as_column(measurement)
        innovation = z - self.H @ self.x
        covariance = self.H @ self.P @ self.H.T + self.R
        return float((innovation.T @ np.linalg.pinv(covariance) @ innovation)[0, 0])


class BoxKalman(LinearKalman):
    """Constant-velocity filter for [cx, cy, width, height]."""

    def __init__(self, box: Box, timestamp: float):
        super().__init__(8, 4)
        self.H[:, :4] = np.eye(4, dtype=np.float64)
        self.R = np.diag([16.0, 16.0, 25.0, 25.0])
        self.last_timestamp = float(timestamp)
        self.reset(box, timestamp)

    @staticmethod
    def box_to_measurement(box: Box) -> Tuple[float, float, float, float]:
        x1, y1, x2, y2 = map(float, box)
        return (
            (x1 + x2) * 0.5,
            (y1 + y2) * 0.5,
            max(x2 - x1, 2.0),
            max(y2 - y1, 2.0),
        )

    @staticmethod
    def measurement_to_box(values: Iterable[float]) -> Box:
        cx, cy, width, height = map(float, values)
        width, height = max(width, 2.0), max(height, 2.0)
        return (
            cx - width * 0.5,
            cy - height * 0.5,
            cx + width * 0.5,
            cy + height * 0.5,
        )

    def reset(self, box: Box, timestamp: float) -> None:
        self.x.fill(0.0)
        self.x[:4] = _as_column(self.box_to_measurement(box))
        self.P = np.eye(8, dtype=np.float64) * 10.0
        self.last_timestamp = float(timestamp)
        self.initialized = True

    def predict(self, timestamp: float) -> Box:
        dt = min(max(float(timestamp) - self.last_timestamp, 1.0 / 120.0), 0.5)
        transition = np.eye(8, dtype=np.float64)
        transition[:4, 4:] = np.eye(4, dtype=np.float64) * dt
        acceleration = np.array([4.0, 4.0, 2.0, 2.0], dtype=np.float64)
        process = np.zeros((8, 8), dtype=np.float64)
        process[:4, :4] = np.diag(acceleration * dt ** 4 / 4.0)
        process[:4, 4:] = np.diag(acceleration * dt ** 3 / 2.0)
        process[4:, :4] = process[:4, 4:]
        process[4:, 4:] = np.diag(acceleration * dt ** 2)
        self.x = transition @ self.x
        self.P = transition @ self.P @ transition.T + process
        self.x[2, 0] = max(self.x[2, 0], 2.0)
        self.x[3, 0] = max(self.x[3, 0], 2.0)
        self.last_timestamp = float(timestamp)
        return self.box

    def update(self, box: Box) -> Box:
        self.correct(self.box_to_measurement(box))
        self.x[2, 0] = max(self.x[2, 0], 2.0)
        self.x[3, 0] = max(self.x[3, 0], 2.0)
        return self.box

    @property
    def box(self) -> Box:
        return self.measurement_to_box(self.x[:4, 0])


class WorldKalman(LinearKalman):
    """Timestamp-aware constant-velocity filter in referee-map centimetres."""

    def __init__(self):
        super().__init__(4, 2)
        self.H[:, :2] = np.eye(2, dtype=np.float64)
        self.R = np.eye(2, dtype=np.float64) * 36.0
        self.last_timestamp: float | None = None

    def reset(self, point: Point, timestamp: float) -> None:
        self.x.fill(0.0)
        self.x[:2] = _as_column(point)
        self.P = np.eye(4, dtype=np.float64) * 100.0
        self.last_timestamp = float(timestamp)
        self.initialized = True

    def predict(self, timestamp: float) -> Point | None:
        if not self.initialized or self.last_timestamp is None:
            return None
        dt = min(max(float(timestamp) - self.last_timestamp, 1.0 / 120.0), 0.5)
        transition = np.array(
            [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
            dtype=np.float64,
        )
        q = 40.0
        process = q * np.array(
            [
                [dt ** 4 / 4, 0, dt ** 3 / 2, 0],
                [0, dt ** 4 / 4, 0, dt ** 3 / 2],
                [dt ** 3 / 2, 0, dt ** 2, 0],
                [0, dt ** 3 / 2, 0, dt ** 2],
            ],
            dtype=np.float64,
        )
        self.x = transition @ self.x
        self.P = transition @ self.P @ transition.T + process
        self.last_timestamp = float(timestamp)
        return self.position

    def update(self, point: Point, timestamp: float, reset_distance: float = 500.0) -> Point:
        if not self.initialized:
            self.reset(point, timestamp)
            return self.position
        if self.last_timestamp != float(timestamp):
            self.predict(timestamp)
        if np.linalg.norm(np.asarray(point) - np.asarray(self.position)) > reset_distance:
            self.reset(point, timestamp)
        else:
            self.correct(point)
        return self.position

    @property
    def position(self) -> Point:
        return float(self.x[0, 0]), float(self.x[1, 0])

    @property
    def velocity(self) -> Point:
        return float(self.x[2, 0]), float(self.x[3, 0])
