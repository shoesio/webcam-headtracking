"""Velocity-adaptive measurement filtering; no dead zones or pose limits."""
import math
from collections import deque
import numpy as np


class AdaptivePoseFilter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.filtered = None
        self.velocity = np.zeros(6, dtype=np.float64)
        self.timestamp = None
        self.samples = deque(maxlen=3)

    def update(self, pose, timestamp, amount):
        pose = np.asarray(pose, dtype=np.float64)
        if self.timestamp is None:
            self.samples.append(pose.copy())
            self.previous = pose.copy()
            self.filtered = pose.copy()
            self.velocity[:] = 0.0
            self.timestamp = timestamp
            return pose.copy()
        dt = timestamp - self.timestamp
        if dt <= 0:
            return self.filtered.copy()
        if dt > 0.35:
            # Reacquisition must not snap directly to a noisy new measurement.
            # Preserve the rendered estimate, clear old velocity and rebuild the
            # short sample history around it. Explicit recenter still calls reset.
            self.samples.clear()
            self.samples.extend(self.filtered.copy() for _ in range(3))
            self.previous = self.filtered.copy()
            self.velocity[:] = 0.0
            dt = 1.0 / 30.0
        self.samples.append(pose.copy())
        raw_pose = pose
        if amount > 0.001:
            # A causal three-sample median rejects isolated landmark failures.
            # Sustained movements pass after approximately one camera frame.
            pose = np.median(np.asarray(self.samples), axis=0)
        derivative = (pose - self.previous) / dt
        velocity_alpha = 1.0 - math.exp(-dt * 2.0 * math.pi * 1.0)
        self.velocity += (derivative - self.velocity) * velocity_alpha
        self.previous = pose.copy()
        self.timestamp = timestamp
        if amount <= 0.001:
            self.filtered = raw_pose.copy()
        else:
            # Strong filtering at rest, faster following during intentional motion.
            # Units are degrees for rotation, MediaPipe translation units for XYZ.
            base_cutoff = 0.45 + 3.0 * (1.0 - amount) ** 2
            beta = np.array([0.035, 0.035, 0.035, 0.045, 0.045, 0.045])
            cutoff = base_cutoff + beta * np.abs(self.velocity)
            alpha = 1.0 - np.exp(-dt * 2.0 * math.pi * cutoff)
            self.filtered += (pose - self.filtered) * alpha
        return self.filtered.copy()
