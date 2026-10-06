"""Fixed measurement smoothing; no movement-dependent acceleration or limits."""
import math
from collections import deque
import numpy as np


class FixedPoseFilter:
    def __init__(self):
        self.reset()

    def reset(self):
        self.filtered = None
        self.timestamp = None
        self.samples = deque(maxlen=3)

    def update(self, pose, timestamp, amount):
        pose = np.asarray(pose, dtype=np.float64)
        if self.timestamp is None:
            self.samples.append(pose.copy())
            self.filtered = pose.copy()
            self.timestamp = timestamp
            return pose.copy()
        dt = timestamp - self.timestamp
        if dt <= 0:
            return self.filtered.copy()
        if dt > 0.35:
            # Reacquisition must not snap directly to a noisy new measurement.
            # Preserve the rendered estimate and rebuild the
            # short sample history around it. Explicit recenter still calls reset.
            self.samples.clear()
            self.samples.extend(self.filtered.copy() for _ in range(3))
            dt = 1.0 / 30.0
        self.samples.append(pose.copy())
        raw_pose = pose
        if amount > 0.001:
            # A causal three-sample median rejects isolated landmark failures.
            # Sustained movements pass after approximately one camera frame.
            pose = np.median(np.asarray(self.samples), axis=0)
        self.timestamp = timestamp
        if amount <= 0.001:
            self.filtered = raw_pose.copy()
        else:
            # The same response for small movements, fast turns and noise.
            # HA amounts 0.25 / 0.5 / 0.75 / 1 correspond to 50 / 140 /
            # 290 / 500 ms time constants. Zero bypasses filtering entirely.
            tau = 0.020 + 0.480 * amount * amount
            alpha = 1.0 - math.exp(-dt / tau)
            self.filtered += (pose - self.filtered) * alpha
        return self.filtered.copy()
