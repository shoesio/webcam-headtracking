"""Conservative one-sample confirmation of unexpected rotation jumps.

Thresholds classify measurement changes, not allowable head positions. Sustained
movement is accepted on the following sample, with no dead zone or pose limit.
"""
from collections import deque
import numpy as np


class RotationGuard:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.timestamp = None
        self.pending = np.zeros(3, dtype=bool)
        self.speeds = deque(maxlen=15)
        self.rejected = 0

    def update(self, angles, timestamp):
        angles = np.asarray(angles, dtype=float)
        if self.previous is None or timestamp - self.timestamp > 0.35:
            self.previous = angles.copy()
            self.timestamp = timestamp
            self.pending[:] = False
            self.speeds.clear()
            return angles.copy()
        dt = timestamp - self.timestamp
        if dt <= 0:
            return self.previous.copy()
        # Work in physical degrees before camera gain magnifies noise.
        delta = angles - self.previous
        threshold = np.full(3, 2.5)
        if len(self.speeds) >= 5:
            speeds = np.asarray(self.speeds)
            typical = np.median(speeds, axis=0)
            spread = np.median(np.abs(speeds - typical), axis=0)
            threshold = np.maximum(threshold, (typical + 6 * 1.4826 * spread) * dt)
        unusual = np.abs(delta) > threshold
        reject = unusual & ~self.pending
        accepted = np.where(reject, self.previous, angles)
        self.pending = reject
        self.rejected += int(np.count_nonzero(reject))
        # Do not teach the noise estimator that a held outlier was real motion.
        if not np.any(unusual):
            self.speeds.append(np.abs(delta) / dt)
        self.previous = accepted.copy()
        self.timestamp = timestamp
        return accepted
