#!/usr/bin/env python3
# ruff: noqa: E402
"""Deterministic tests for the GPS/IMU-only yaw bootstrap."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "controller"))

from magless_bootstrap import (
    EARTH_RADIUS_M,
    LaunchReference,
    _raw_navigation_reasons,
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

    def test_bootstrap_thrust_rejects_impossible_px4_ceiling(self):
        config = MissionConfig()
        with self.assertRaisesRegex(RuntimeError, "ceiling"):
            bootstrap_thrust(
                z=0.0,
                vz=0.0,
                target_z=-2.0,
                hover_thrust=0.60,
                px4_max_thrust=config.bootstrap_min_thrust + 0.01,
                config=config,
            )

    def test_bootstrap_thrust_rejects_every_nonfinite_input(self):
        valid = dict(z=0.0, vz=0.0, target_z=-2.0,
                     hover_thrust=0.6, px4_max_thrust=1.0)
        for field in valid:
            for bad in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field, bad=bad):
                    values = dict(valid, **{field: bad})
                    with self.assertRaisesRegex(RuntimeError, "Non-finite"):
                        bootstrap_thrust(**values, config=MissionConfig())

    def test_raw_navigation_rejects_nonfinite_bootstrap_state(self):
        controller, snapshot = self._healthy_raw_navigation()
        for field in ("z", "vz", "roll", "pitch", "yaw"):
            for bad in (math.nan, math.inf, -math.inf):
                with self.subTest(field=field, bad=bad):
                    broken = dict(snapshot, **{field: bad})
                    self.assertIn(f"bootstrap state {field} is non-finite",
                                  _raw_navigation_reasons(controller, broken))

    def test_raw_navigation_accepts_complete_finite_snapshot(self):
        controller, snapshot = self._healthy_raw_navigation()
        self.assertEqual(_raw_navigation_reasons(controller, snapshot), [])

    def test_raw_navigation_rejects_nonfinite_and_invalid_inputs(self):
        controller, snapshot = self._healthy_raw_navigation()
        snapshot.update(
            position_age_s=math.nan,
            attitude_age_s=math.inf,
            gps_age_s=math.nan,
            gps_source_age_s=math.inf,
            gps_source_regressed=True,
            gps_fix_type=2,
            gps_satellites_visible=0,
            gps_hdop=math.nan,
            gps_vdop=math.inf,
            gps_horizontal_accuracy_m=math.nan,
            gps_vertical_accuracy_m=math.inf,
            estimator_age_s=math.nan,
            estimator_flags=0,
            estimator_pos_vert_ratio=math.nan,
        )

        reasons = _raw_navigation_reasons(controller, snapshot)

        expected = {
            "vertical position telemetry stale",
            "attitude telemetry stale",
            "GPS telemetry stale",
            "GPS measurement timestamp stopped advancing",
            "GPS measurement timestamp moved backwards",
            "GPS has no 3D fix",
            "GPS satellite count is below the configured minimum",
            "GPS HDOP exceeds the configured maximum",
            "GPS VDOP exceeds the configured maximum",
            "GPS horizontal accuracy is insufficient",
            "GPS vertical accuracy is insufficient",
            "estimator telemetry stale",
            "estimator flags 0x0 missing bootstrap mask 0x25",
            "vertical position innovation is unhealthy",
        }
        self.assertEqual(set(reasons), expected)

    @staticmethod
    def _healthy_raw_navigation():
        config = MissionConfig()
        controller = SimpleNamespace(
            config=config,
            max_position_age_s=0.5,
            max_attitude_age_s=0.5,
            max_heartbeat_age_s=1.5,
        )
        snapshot = {
            "z": 0.0,
            "vz": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
            "heartbeat_age_s": 0.0,
            "armed": False,
            "heartbeat_main_mode": 6,
            "position_age_s": 0.0,
            "attitude_age_s": 0.0,
            "gps_age_s": 0.0,
            "gps_source_age_s": 0.0,
            "gps_source_regressed": False,
            "gps_fix_type": config.gps_min_fix_type,
            "gps_satellites_visible": config.gps_min_satellites,
            "gps_hdop": 0.5,
            "gps_vdop": 0.8,
            "gps_horizontal_accuracy_m": 0.5,
            "gps_vertical_accuracy_m": 0.8,
            "estimator_age_s": 0.0,
            "estimator_flags": 1 | 4 | 32,
            "estimator_pos_vert_ratio": 0.5,
        }
        return controller, snapshot


if __name__ == "__main__":
    unittest.main(verbosity=2)
