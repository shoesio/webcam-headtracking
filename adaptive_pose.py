"""Velocity-adaptive measurement filtering; no dead zones or pose limits."""
import math
import numpy as np


class AdaptivePoseFilter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.filtered = None
        self.velocity = np.zeros(6, dtype=np.float64)
        self.timestamp = None

    def update(self, pose, timestamp, amount):
        pose = np.asarray(pose, dtype=np.float64)
        if self.timestamp is None or timestamp - self.timestamp > 0.35:
            self.previous = pose.copy()
            self.filtered = pose.copy()
            self.velocity[:] = 0.0
            self.timestamp = timestamp
            return pose.copy()
        dt = timestamp - self.timestamp
        if dt <= 0:
            return self.filtered.copy()
        derivative = (pose - self.previous) / dt
        velocity_alpha = 1.0 - math.exp(-dt * 2.0 * math.pi * 1.0)
        self.velocity += (derivative - self.velocity) * velocity_alpha
        self.previous = pose.copy()
        self.timestamp = timestamp
        if amount <= 0.001:
            self.filtered = pose.copy()
        else:
            # Strong filtering at rest, faster following during intentional motion.
            # Units are degrees for rotation, MediaPipe translation units for XYZ.
            base_cutoff = 0.6 + 3.0 * (1.0 - amount) ** 2
            beta = np.array([0.045, 0.045, 0.045, 0.06, 0.06, 0.06])
            cutoff = base_cutoff + beta * np.abs(self.velocity)
            alpha = 1.0 - np.exp(-dt * 2.0 * math.pi * cutoff)
            self.filtered += (pose - self.filtered) * alpha
        return self.filtered.copy()
