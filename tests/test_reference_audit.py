"""Independent analytic fixtures and corruption tests, not generator snapshots."""
import csv
import io
import json
from pathlib import Path
import unittest

from analysis.reference_audit import audit_reference

ROOT = Path(__file__).resolve().parents[1]


class ReferenceAuditTests(unittest.TestCase):
    def setUp(self):
        self.model = json.loads((ROOT / "config/planner/x500_sitl_vehicle_model.json").read_text())
        self.environment = json.loads((ROOT / "config/planner/empty_world_environment.json").read_text())
        self.rows = []
        for i in range(201):
            t = i / 20
            # Independent constant-acceleration, moving NED reference.
            self.rows.append(dict(time=t, x=.01*t*t, y=0, z=-10, yaw=.02*t*t,
                                  vx=.02*t, vy=0, vz=0, yaw_rate=.04*t,
                                  ax=.02, ay=0, az=0, yaw_acceleration=.04,
                                  jx=0, jy=0, jz=0, yaw_jerk=0))

    def audit(self):
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fieldnames=self.rows[0])
        writer.writeheader()
        writer.writerows(self.rows)
        return audit_reference(stream.getvalue(), self.model, self.environment)

    def test_analytic_motion(self):
        result = self.audit()
        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["sampled_path_length_m"], 1)
        self.assertLess(result["metrics"]["position_integral_residual_m"], 1e-14)
        self.assertFalse(result["model_identified"])

    def test_position_corruption_detected(self):
        self.rows[100]["x"] += .01
        self.assertFalse(self.audit()["checks"]["position_integral"])

    def test_constant_acceleration_bias_detected(self):
        for row in self.rows:
            row['ax'] += .1
        result = self.audit()
        self.assertTrue(result['checks']['position_integral'])
        self.assertFalse(result['checks']['velocity_integral'])

    def test_velocity_corruption_detected(self):
        self.rows[100]["vx"] += .01
        self.assertFalse(self.audit()["checks"]["position_integral"])

    def test_yaw_corruption_detected(self):
        self.rows[100]["yaw"] += .01
        self.assertFalse(self.audit()["checks"]["yaw_integral"])

    def test_overspeed_detected_even_when_kinematically_consistent(self):
        for row in self.rows:
            row.update(x=4*row["time"], vx=4, ax=0)
        result = self.audit()
        self.assertTrue(result["checks"]["position_integral"])
        self.assertFalse(result["checks"]["speed"])

    def test_ned_thrust_sign(self):
        for row in self.rows:
            t = row["time"]
            row.update(z=-10-.01*t*t, vz=-.02*t, az=-.02)
        self.assertGreater(self.audit()["metrics"]["thrust_max_n"], self.model["mass_kg"]*9.80665)

    def test_missing_duplicate_nonfinite_and_sparse_data(self):
        for field, value in (("time", 0), ("x", float("nan")), ("time", 20)):
            with self.subTest(field=field, value=value):
                old = self.rows[100][field]
                self.rows[100][field] = value
                with self.assertRaises(ValueError):
                    self.audit()
                self.rows[100][field] = old
        del self.rows[0]["jx"]
        with self.assertRaises(ValueError):
            self.audit()

    def test_stationary_turn_accounting(self):
        for row in self.rows:
            row.update(x=0, vx=0, ax=0)
        result = self.audit()
        self.assertAlmostEqual(result["sampled_stationary_time_s"], 10)
        self.assertAlmostEqual(result["sampled_stationary_turn_time_s"], 10)


class TimingDisclosureTests(unittest.TestCase):
    def test_irregular_clock_and_stationary_turns(self):
        from mission_planner.preview import speed_preferences
        data = 'time,vx,vy,vz,yaw_rate\n0,0,0,0,0\n0.05,0,0,0,1\n0.07,0,0,0,0\n1,2,0,0,0\n'
        motion = dict(target_speed_m_s=2, maximum_speed_m_s=3)
        result = dict(intent=dict(motion=motion), trajectory_csv=data,
                      vehicle_model=dict(maximum_speed_m_s=3),
                      summary=dict(retiming=dict(speed_limit_m_s=1.8)))
        measured = speed_preferences(dict(source='drawing', document=dict(motion=motion)), result)
        self.assertAlmostEqual(measured['stationary_time_sampled_s'], .07)
        self.assertAlmostEqual(measured['stationary_turn_time_sampled_s'], .07)
        self.assertAlmostEqual(measured['route_distance_sampled_m'], .93)
        self.assertAlmostEqual(measured['planned_route_mean_sampled_m_s'], .93)
