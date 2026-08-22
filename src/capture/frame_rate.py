from __future__ import annotations

import threading
import time
from collections import deque


class RecentFrameRate:
    """Thread-safe delivered-frame rate over a short rolling window."""

    def __init__(self, window_seconds: float = 2.0):
        self.window_seconds = max(float(window_seconds), 0.1)
        self._timestamps = deque()
        self._lock = threading.Lock()
        self._has_rate_sample = False

    def mark(self, timestamp: float | None = None) -> None:
        now = float(time.monotonic() if timestamp is None else timestamp)
        with self._lock:
            self._timestamps.append(now)
            if len(self._timestamps) >= 2:
                self._has_rate_sample = True
            self._prune(now)

    def snapshot(self, timestamp: float | None = None) -> float | None:
        now = float(time.monotonic() if timestamp is None else timestamp)
        with self._lock:
            self._prune(now)
            if len(self._timestamps) < 2:
                return 0.0 if self._has_rate_sample else None
            elapsed = self._timestamps[-1] - self._timestamps[0]
            if elapsed <= 0.0:
                return None
            return (len(self._timestamps) - 1) / elapsed

    def _prune(self, now: float) -> None:
        cutoff = now - self.window_seconds
        while self._timestamps and self._timestamps[0] < cutoff:
            self._timestamps.popleft()
