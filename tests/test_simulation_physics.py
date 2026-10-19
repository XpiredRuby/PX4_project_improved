"""Physics-truth tests: no predicted trajectory may substitute for Gazebo truth."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from simulation_physics import assess_simulation_physics, quaternion_tilt_deg  # noqa: E402


class SimulatorPhysicsTests(unittest.TestCase):
    def dataset(self):
        phase = ["TAKEOFF", "TRAJECTORY", "TRAJECTORY", "TRAJECTORY", "PX4_LAND"]
        velocity = np.zeros((len(phase), 3))
        velocity[1:4, 0] = [0.2, 0.8, 0.2]
        acceleration = np.zeros_like(velocity)
        acceleration[1:4, 0] = [0.2, 0.5, -0.2]
        tilt = np.array([0., 2., 4., 2., 0.])
        return phase, velocity, acceleration, tilt

    def assess(self, data, **limits):
        return assess_simulation_physics(
            *data, maximum_speed_m_s=limits.get("speed", 1.),
            maximum_tilt_deg=limits.get("tilt", 10.))

    def test_measured_quaternion_tilt_ignores_yaw_and_normalizes(self):
        pitch = math.radians(30.)
        yaw = math.radians(120.)
        q = np.array([
            [1, 0, 0, 0],
            [math.cos(yaw/2), 0, 0, math.sin(yaw/2)],
            [math.cos(pitch/2), 0, math.sin(pitch/2), 0],
            [2 * math.cos(pitch/2), 0, 2 * math.sin(pitch/2), 0],
            [0, 0, 0, 0],
        ], dtype=float)
        values = quaternion_tilt_deg(q)
        np.testing.assert_allclose(values[:2], 0., atol=1e-6)
        np.testing.assert_allclose(values[2:4], 30., atol=1e-5)
        self.assertTrue(np.isnan(values[-1]))
        with self.assertRaisesRegex(ValueError, "four components"):
            quaternion_tilt_deg(np.ones((3, 3)))

    def test_truth_inside_envelope_passes_and_reports_physical_acceleration(self):
        report = self.assess(self.dataset())
        self.assertTrue(all(c["passed"] for c in report["checks"]))
        metrics = report["observations"]
        self.assertAlmostEqual(metrics["simulator_truth_speed_m_s"]["max_abs"], .8)
        self.assertAlmostEqual(metrics["simulator_truth_acceleration_m_s2"]["max_abs"], .5)
        self.assertAlmostEqual(metrics["simulator_truth_tilt_deg"]["max_abs"], 4.)
        self.assertIn("no physical flight authority", report["scope"])

    def test_speed_tilt_or_missing_simulation_truth_refuses(self):
        for corruption in ("speed", "tilt", "velocity_missing", "tilt_missing",
                           "acceleration_missing", "no_trajectory"):
            phase, velocity, acceleration, tilt = self.dataset()
            if corruption == "speed":
                velocity[2, 0] = 2.
            elif corruption == "tilt":
                tilt[2] = 15.
            elif corruption == "velocity_missing":
                velocity[2, 0] = float("nan")
            elif corruption == "tilt_missing":
                tilt[2] = float("nan")
            elif corruption == "acceleration_missing":
                acceleration[2, 0] = float("nan")
            else:
                phase = ["TAKEOFF"] * len(phase)
            with self.subTest(corruption=corruption):
                checks = self.assess((phase, velocity, acceleration, tilt))["checks"]
                self.assertFalse(all(c["passed"] for c in checks))

    def test_only_trajectory_truth_is_scored_not_bootstrap_and_landing(self):
        phase, velocity, acceleration, tilt = self.dataset()
        velocity[0, 0] = 50.
        tilt[-1] = 70.
        self.assertTrue(all(c["passed"] for c in self.assess(
            (phase, velocity, acceleration, tilt))["checks"]))

    def test_shape_and_configuration_refused(self):
        phase, velocity, acceleration, tilt = self.dataset()
        with self.assertRaisesRegex(ValueError, "dimensions"):
            self.assess((phase, velocity[:-1], acceleration, tilt))
        for bound in (0., -1., float("nan"), float("inf")):
            with self.subTest(bound=bound), self.assertRaisesRegex(ValueError, "envelope"):
                self.assess((phase, velocity, acceleration, tilt), speed=bound)
        with self.assertRaisesRegex(ValueError, "envelope"):
            self.assess((phase, velocity, acceleration, tilt), tilt=90.)


if __name__ == "__main__":
    unittest.main()
