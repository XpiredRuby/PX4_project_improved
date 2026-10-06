import math
import unittest

from simple_flight import LoopTiming, MissionConfig, MissionLogic, StraightTrajectory, VehicleState


class SimpleFlightTests(unittest.TestCase):
    def setUp(self):
        self.config = MissionConfig()
        self.state = VehicleState(z=0.0, yaw=0.2, landed_state=1)

    def test_confirmed_values(self):
        self.assertEqual(self.config.control_hz, 10.0)
        self.assertEqual(self.config.period_s, 0.1)
        self.assertEqual(self.config.run_length_m, 200.0)
        self.assertEqual(self.config.cruise_speed_m_s, 5.0)
        self.assertEqual(self.config.slow_descent_height_m, 5.0)
        self.assertEqual(self.config.slow_descent_m_s, 0.1)

    def test_timing_band_is_plus_or_minus_ten_percent(self):
        timing = LoopTiming(0.1, 0.10)
        self.assertTrue(timing.add(0.09))
        self.assertTrue(timing.add(0.11))
        self.assertFalse(timing.add(0.089))
        self.assertFalse(timing.add(0.111))

    def test_trajectory_is_exactly_200_m_and_never_exceeds_5_m_s(self):
        trajectory = StraightTrajectory(self.config)
        for _ in range(3000):
            distance, speed = trajectory.step(0.1)
            self.assertLessEqual(speed, 5.0)
            if distance == 200.0:
                break
        self.assertEqual(distance, 200.0)

    def test_heading_rotates_the_200_m_endpoint(self):
        logic = MissionLogic(self.config, self.state, heading_deg=90.0)
        self.assertAlmostEqual(logic.north, 0.0, places=7)
        self.assertAlmostEqual(logic.east, 1.0, places=7)

    def test_descent_slows_at_five_metres_agl(self):
        logic = MissionLogic(self.config, self.state, heading_deg=0.0)
        logic.phase = "LAND_FAST"
        above = VehicleState(x=200.0, z=-5.01, landed_state=2)
        at_threshold = VehicleState(x=200.0, z=-5.0, landed_state=2)
        self.assertEqual(logic.command(above, 0.1).vz, 0.5)
        self.assertEqual(logic.command(at_threshold, 0.1).vz, 0.1)
        self.assertEqual(logic.phase, "LAND_SLOW")

    def test_landing_uses_known_initial_surface_height(self):
        initial = VehicleState(x=10.0, y=20.0, z=3.0, yaw=math.pi / 2, landed_state=1)
        logic = MissionLogic(self.config, initial, heading_deg=0.0)
        self.assertEqual(logic.altitude_agl(VehicleState(z=-2.0)), 5.0)


if __name__ == "__main__":
    unittest.main()
