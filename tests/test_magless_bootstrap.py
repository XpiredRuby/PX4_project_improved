#!/usr/bin/env python3
# ruff: noqa: E402
"""Deterministic tests for the GPS/IMU-only yaw bootstrap."""

import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

from magless_bootstrap import (
    EARTH_RADIUS_M,
    LaunchReference,
    bootstrap_thrust,
    home_from_geodetic_sample,
    quaternion_from_euler,
)
from mission_config import MissionConfig


class MaglessBootstrapTests(unittest.TestCase):
    def test_quaternion_is_normalized_and_preserves_level_yaw(self):
        yaw = math.radians(37.0)
        quaternion = quaternion_from_euler(0.0, 0.0, yaw)

        self.assertAlmostEqual(
            math.sqrt(sum(value * value for value in quaternion)),
            1.0,
        )
        self.assertAlmostEqual(quaternion[1], 0.0)
        self.assertAlmostEqual(quaternion[2], 0.0)
        self.assertAlmostEqual(
            2.0 * math.atan2(quaternion[3], quaternion[0]),
            yaw,
        )

    def test_saved_launch_fix_maps_back_into_local_ned(self):
        launch = LaunchReference(
            latitude_deg=30.0,
            longitude_deg=-96.0,
            altitude_m=100.0,
            horizontal_accuracy_m=0.5,
            vertical_accuracy_m=1.0,
            sample_count=20,
        )
        north_m = 1.25
        east_m = -0.80
        up_m = 2.0
        current_latitude = launch.latitude_deg + math.degrees(
            north_m / EARTH_RADIUS_M
        )
        current_longitude = launch.longitude_deg + math.degrees(
            east_m
            / (EARTH_RADIUS_M * math.cos(math.radians(launch.latitude_deg)))
        )
        current_local = (4.0, -3.0, -1.5)
        current_geodetic = (
            current_latitude,
            current_longitude,
            launch.altitude_m + up_m,
        )

        home = home_from_geodetic_sample(
            current_local,
            current_geodetic,
            launch,
        )

        self.assertAlmostEqual(home[0], current_local[0] - north_m, places=5)
        self.assertAlmostEqual(home[1], current_local[1] - east_m, places=5)
        self.assertAlmostEqual(home[2], current_local[2] + up_m, places=7)

    def test_bootstrap_thrust_uses_px4_ceiling_and_configured_floor(self):
        config = MissionConfig()
        climbing = bootstrap_thrust(
            z=0.0,
            vz=0.0,
            target_z=-2.0,
            hover_thrust=0.60,
            px4_max_thrust=1.0,
            config=config,
        )
        descending = bootstrap_thrust(
            z=-4.0,
            vz=1.0,
            target_z=-2.0,
            hover_thrust=0.60,
            px4_max_thrust=1.0,
            config=config,
        )

        self.assertEqual(climbing, config.bootstrap_max_thrust)
        self.assertGreaterEqual(descending, config.bootstrap_min_thrust)
        self.assertLessEqual(descending, config.bootstrap_max_thrust)


if __name__ == "__main__":
    unittest.main(verbosity=2)
