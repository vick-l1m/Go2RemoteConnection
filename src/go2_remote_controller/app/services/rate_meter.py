"""
rate_meter.py
Sliding-window message-rate meter for the perception status endpoint.

Version: 1.0
Author: Victor Lim
"""

import threading
import time
from collections import deque
from typing import Optional


class RateMeter:
    """Call tick() per message; hz() is the rate over the last ``window_s`` seconds
    and age_s() the time since the last tick (None before the first)."""

    def __init__(self, window_s: float = 2.0):
        self.window_s = float(window_s)
        self._stamps: deque = deque()
        self._lock = threading.Lock()

    def tick(self, t: Optional[float] = None) -> None:
        now = time.monotonic() if t is None else t
        with self._lock:
            self._stamps.append(now)
            self._prune(now)

    def _prune(self, now: float) -> None:
        cutoff = now - self.window_s
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.popleft()

    def hz(self, now: Optional[float] = None) -> float:
        now = time.monotonic() if now is None else now
        with self._lock:
            self._prune(now)
            n = len(self._stamps)
            if n < 2:
                # One message inside the window is "alive" but not a rate yet.
                return 0.0
            span = self._stamps[-1] - self._stamps[0]
            return (n - 1) / span if span > 0 else 0.0

    def age_s(self, now: Optional[float] = None) -> Optional[float]:
        now = time.monotonic() if now is None else now
        with self._lock:
            if not self._stamps:
                return None
            return now - self._stamps[-1]

    def alive(self, max_age_s: float = 2.0, now: Optional[float] = None) -> bool:
        age = self.age_s(now)
        return age is not None and age <= max_age_s
