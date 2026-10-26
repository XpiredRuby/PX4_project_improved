"""Independent truth and missing-evidence regressions for rehearsal scoring."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
from planner_rehearsal import assess_frames, assess_aircraft_envelope  # noqa: E402


class RehearsalTests(unittest.TestCase):
    def evidence(self):
        time = np.arange(0., 2.01, .05)
        phase = ["TRAJECTORY"] * 21 + ["PX4_LAND"] * 20
        position = np.zeros((len(time), 3))
        df = pd.DataFrame({"elapsed_s": time, "wall_time": 1000+time,
                           "position_time_boot_ms": time*1000, "phase": phase,
                           "mission_time_s": np.minimum(time, 1.), "target_yaw_unwrapped": 0.,
                           "desired_x": 0., "desired_y": 0., "desired_z": 0., "target_yaw": 0.,
                           "landed_state": [2]*30+[1]*11,
                           "native_landing_reference_x": 0., "native_landing_reference_y": 0.})
        plan = pd.DataFrame({"time": [0., 1.], "x": 0., "y": 0., "z": 0., "yaw": 0.})
        return [df, plan, position, np.zeros(len(time)), np.zeros_like(position),
                {"overall_passed": True}, {"armed": False, "landed_state": 1,
                                           "actuators": [0.]*4, "continuous_confirmation_s": 2.}]

    def test_constructive_runtime_reference_uses_phase_polynomial_not_linear_chords(self):
        from mission_planner.stop_turn_execution import sample_stop_turn_reference
        evidence = self.evidence()
        df, plan, physical, yaw, velocity, audit, ground = evidence
        phases = [dict(name='LEG',start=[0.,0.,0.,0.],end=[1.,0.,0.,0.],duration_s=1.)]
        values = sample_stop_turn_reference(phases, df.mission_time_s.to_numpy())
        df['desired_x'] = values[:, 0]
        physical[:, 0] = values[:, 0]
        # Deliberately sparse CSV: a linear chord would differ by ~0.18 m.
        plan['x'] = [0., 1.]
        result = assess_frames(df,plan,physical,yaw,velocity,audit,ground,constructive_phases=phases)
        check = next(c for c in result['checks'] if c['name']=='Runtime reference follows exact CSV with one translation')
        self.assertTrue(check['passed'])
        legacy = assess_frames(df,plan,physical,yaw,velocity,audit,ground)
        check = next(c for c in legacy['checks'] if c['name']=='Runtime reference follows exact CSV with one translation')
        self.assertFalse(check['passed'])

    def test_complete_truth_and_lifecycle_pass_only_sampled_sitl_scope(self):
        result = assess_frames(*self.evidence())
        self.assertTrue(result["sampled_rehearsal_passed"])
        self.assertFalse(result["hardware_approval"])
        self.assertIn("wall-time", result["timing_scope"])

    def test_truth_error_cannot_be_hidden_by_perfect_estimated_reference(self):
        for kind in ("position", "heading", "velocity", "missing"):
            args = self.evidence()
            if kind == "position":
                args[2][5, 0] = 1.1
            elif kind == "heading":
                args[3][5] = np.radians(11.)
            elif kind == "velocity":
                args[4][30, 2] = .21
            else:
                args[2][10] = np.nan
            with self.subTest(kind=kind):
                self.assertFalse(assess_frames(*args)["sampled_rehearsal_passed"])

    def test_partial_route_rewritten_reference_and_unconfirmed_shutdown_fail(self):
        for kind in ("partial", "reference", "yaw", "shutdown", "nan_output", "clock"):
            args = deepcopy(self.evidence())
            if kind == "partial":
                args[1].loc[1, "time"] = 2.
            elif kind == "reference":
                args[0].loc[8, "desired_x"] = .01
            elif kind == "yaw":
                args[0].loc[8, "target_yaw_unwrapped"] = .01
            elif kind == "shutdown":
                args[-1]["continuous_confirmation_s"] = 1.99
            elif kind == "nan_output":
                args[-1]["actuators"][2] = float("nan")
            else:
                args[0]["position_time_boot_ms"] *= .5
            with self.subTest(kind=kind):
                self.assertFalse(assess_frames(*args)["sampled_rehearsal_passed"])

    def test_every_motor_is_checked_for_a_six_rotor_aircraft(self):
        args = self.evidence()
        args[-1]["actuators"] = [0.] * 6
        self.assertTrue(assess_frames(*args, rotor_count=6)["sampled_rehearsal_passed"])
        args[-1]["actuators"][5] = .2
        self.assertFalse(assess_frames(*args, rotor_count=6)["sampled_rehearsal_passed"])

    def test_declared_profile_accuracy_and_time_limits_cannot_be_ignored(self):
        args = self.evidence()
        df, physical = args[0], args[2]
        df["x"], df["y"], df["z"] = 0., 0., 0.
        profile = {"position_uncertainty_m": .1, "tracking_error_m": .1,
                   "contingency_time_s": 1.}
        budget = {"total_budgeted_s": 4., "route_wall_time_budget_s": 1.5}
        self.assertTrue(all(c["passed"] for c in assess_aircraft_envelope(df, physical, profile, budget)))
        physical[5, 0] = .11
        self.assertFalse(assess_aircraft_envelope(df, physical, profile, budget)[0]["passed"])
        df.loc[5, "x"] = .2
        self.assertFalse(assess_aircraft_envelope(df, physical, profile, budget)[1]["passed"])
        budget["total_budgeted_s"] = 1.5
        self.assertFalse(assess_aircraft_envelope(df, physical, profile, budget)[2]["passed"])


if __name__ == "__main__":
    unittest.main()
