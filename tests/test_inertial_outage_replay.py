"""Analytic checks for offline inertial feasibility; no flight authorization."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from inertial_outage_replay import estimate_bias, propagate  # noqa: E402


class InertialReplayTests(unittest.TestCase):
    def test_irregular_clock_integrates_constant_and_linear_acceleration(self):
        times = np.array([0., .03, .10, .14, .23, .30])
        acceleration = np.column_stack([2. + 3. * times, np.zeros(len(times)), np.zeros(len(times))])
        position, velocity = propagate(times, acceleration, [4., 0., 0.], [1., 0., 0.], [0., 0., 0.])
        np.testing.assert_allclose(position[:, 0], 4. + times + times**2 + .5 * times**3)
        np.testing.assert_allclose(velocity[:, 0], 1. + 2. * times + 1.5 * times**2)

    def test_constant_bias_is_learned_from_prior_velocities_and_removed(self):
        times = np.linspace(0., 4., 81)
        acceleration = np.tile([.01, -.02, .03], (81, 1))
        bias = estimate_bias(times, acceleration, [1., 0., 0.], [1., 0., 0.])
        np.testing.assert_allclose(bias, [.01, -.02, .03])
        position, _ = propagate(times, acceleration, [0., 0., 0.], [1., 0., 0.], bias)
        np.testing.assert_allclose(position[-1], [4., 0., 0.], atol=1e-12)

    def test_missing_nonfinite_repeated_regressed_and_gapped_samples_reject(self):
        for times in ([0.], [0., 0.], [.1, 0.], [0., .2], [0., float("nan")]):
            with self.subTest(times=times), self.assertRaises(ValueError):
                propagate(times, np.zeros((len(times), 3)), [0.] * 3, [0.] * 3, [0.] * 3)
        with self.assertRaises(ValueError):
            propagate([0., .1], [[0., 0., 0.], [float("nan"), 0., 0.]], [0.] * 3, [0.] * 3, [0.] * 3)
        with self.assertRaises(ValueError):
            estimate_bias([0., .1], np.zeros((2, 3)), [0.] * 3, [0.] * 3)
