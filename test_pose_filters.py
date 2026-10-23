"""Synthetic regression checks; no webcam, X11, UDP or game input."""
import unittest
import numpy as np
from adaptive_pose import FixedPoseFilter
from rotation_guard import RotationGuard


class PoseFilterTests(unittest.TestCase):
    def test_zero_smoothing_is_direct(self):
        f = FixedPoseFilter()
        f.update(np.zeros(6), 1., 0)
        pose = np.array([90., -40., 25., 3., -2., 1.])
        np.testing.assert_array_equal(f.update(pose, 1.033, 0), pose)

    def test_tiny_sustained_motion_is_not_suppressed(self):
        f = FixedPoseFilter()
        f.update(np.zeros(6), 1., .5)
        for i in range(1, 90):
            value = f.update(np.full(6, .0001), 1. + i / 30., .5)
        np.testing.assert_allclose(value, .0001, rtol=.001)

    def test_larger_smoothing_reacts_more_slowly(self):
        outputs = []
        for amount in (.25, .5, 1., 5.):
            f = FixedPoseFilter()
            f.update(np.zeros(6), 1., amount)
            for i in range(1, 5):
                pose = f.update(np.ones(6), 1. + i / 30., amount)
            outputs.append(pose[0])
        self.assertTrue(all(a > b for a, b in zip(outputs, outputs[1:])))

    def test_head_positions_are_not_capped(self):
        f = FixedPoseFilter()
        f.update(np.zeros(6), 1., .25)
        pose = np.array([250., 150., 180., 20., 30., -40.])
        for i in range(1, 120):
            output = f.update(pose, 1. + i / 30., .25)
        np.testing.assert_allclose(output, pose, rtol=.001)

    def test_guard_confirms_sustained_large_rotation(self):
        guard = RotationGuard()
        np.testing.assert_array_equal(guard.update([0, 0, 0], 1.), [0, 0, 0])
        np.testing.assert_array_equal(guard.update([60, 0, 0], 1.033), [0, 0, 0])
        np.testing.assert_array_equal(guard.update([60, 0, 0], 1.066), [60, 0, 0])


if __name__ == '__main__':
    unittest.main()
