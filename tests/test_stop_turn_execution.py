"""Exact-artifact native adapter checks without opening a MAVLink transport."""

from copy import deepcopy
from datetime import datetime, timedelta, timezone
import csv
import hashlib
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest

from mission_planner.acknowledgement import issue_sitl_acknowledgement
from mission_planner.contracts import canonical_sha256
from mission_planner.preview import preview
from mission_planner.stop_turn_execution import audit_stop_turn_artifacts

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controller"))
from trajectory import Trajectory
from offboard_runner import load_planner_summary  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from planner_sitl_campaign import prepare_case  # noqa: E402
from sitl_scenario_batch import stage_case_artifacts, stage_sitl_acknowledgement  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


class StopTurnExecutionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        raw = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        base = raw["path"]["points"][0]
        raw["path"].update(
            closure="open_stop",
            maximum_deviation_m=0.0,
            points=[
                dict(
                    base,
                    point_id=f"stop-{i}",
                    north_m=n,
                    east_m=e,
                    hard=True,
                    exact=False,
                )
                for i, (n, e) in enumerate(
                    [(0, 0), (0, 8), (8, 8), (8, 0), (3, 0), (3, 4)]
                )
            ],
        )
        raw["motion"].update(
            yaw_mode="fixed",
            fixed_yaw_deg=0.0,
            target_speed_m_s=2.0,
            maximum_speed_m_s=3.0,
        )
        cls.plan = preview(
            dict(source="drawing", document=raw, planning_mode="closest")
        )

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        self.write(self.plan)

    def write(self, plan):
        (self.folder / "trajectory.csv").write_text(plan["trajectory_csv"])
        for name, key in [
            ("summary", "summary"),
            ("verification", "verification"),
            ("intent", "intent"),
            ("vehicle_model", "vehicle_model"),
            ("environment", "environment"),
        ]:
            (self.folder / (name + ".json")).write_text(json.dumps(plan[key]))
        summary = plan["summary"]
        ack = issue_sitl_acknowledgement(
            **{
                k: summary[k]
                for k in (
                    "controller_trajectory_sha256",
                    "candidate_trajectory_sha256",
                    "verification_report_sha256",
                )
            },
            expires_at=(datetime.now(timezone.utc) + timedelta(hours=1)).isoformat(),
            acknowledged_by="offline-test",
        )
        (self.folder / "sitl_acknowledgement.json").write_text(json.dumps(ack))

    def test_real_five_stop_route_independently_audited_and_staged_without_recompilation(
        self,
    ):
        audit = audit_stop_turn_artifacts(self.folder)
        self.assertTrue(audit["accepted"])
        self.assertLess(audit["maximum_serialized_state_error"], 2e-8)
        self.assertFalse(audit["simulator_ran"])
        staged = self.folder / "staged"
        staged.mkdir()
        case = prepare_case(
            "reviewed-plan", staged, self.folder, experimental_stop_turn=True
        )
        self.assertTrue(case["experimental_stop_turn"])
        self.assertEqual(
            (staged / "plan/trajectory.csv").read_bytes(),
            (self.folder / "trajectory.csv").read_bytes(),
        )
        self.assertEqual(
            load_planner_summary(
                self.folder / "summary.json",
                self.folder / "trajectory.csv",
                experimental_stop_turn=True,
            ),
            self.plan["summary"]["controller_cruise_height_m"],
        )

    def test_native_interpolation_reconstructs_audited_polynomials_between_csv_rows(
        self,
    ):
        import numpy as np
        from mission_planner.stop_turn_execution import progress_polynomial

        trajectory = Trajectory(self.folder / "trajectory.csv")
        trajectory.enable_septic_interpolation()
        q, clock = progress_polynomial(), 0.0
        for phase in self.plan["verification"]["phases"]:
            delta = np.subtract(phase["end"], phase["start"])
            for fraction in (0.001, 0.173, 0.499, 0.851, 0.999):
                t = clock + fraction * phase["duration_s"]
                state = trajectory.get_target(t)
                for order, fields in enumerate(
                    (
                        ("x", "y", "z", "yaw"),
                        ("vx", "vy", "vz", "yaw_rate"),
                        ("ax", "ay", "az", "yaw_acceleration"),
                        ("jx", "jy", "jz", "yaw_jerk"),
                    )
                ):
                    expected = (
                        q.deriv(order)(fraction) * delta / phase["duration_s"] ** order
                    )
                    if order == 0:
                        expected += phase["start"]
                    np.testing.assert_allclose(
                        [getattr(state, k) for k in fields],
                        expected,
                        atol=2e-8,
                        rtol=0.0,
                    )
            clock += phase["duration_s"]
        self.assertEqual(trajectory.get_target(0.0), trajectory.points[0])
        self.assertEqual(trajectory.get_target(clock), trajectory.points[-1])

    def test_experimental_campaign_case_is_audited_and_acknowledged(self):
        target = self.folder / "nominal"
        target.mkdir()
        case = prepare_case("stop-turn-nominal", target, experimental_stop_turn=True)
        self.assertTrue(case["experimental_stop_turn"])
        audit = audit_stop_turn_artifacts(target / "plan")
        self.assertTrue(audit["accepted"])
        self.assertEqual(
            load_planner_summary(
                target / "plan/summary.json",
                target / "plan/trajectory.csv",
                experimental_stop_turn=True,
            ),
            20.0,
        )

    def test_actual_batch_staging_retains_all_sidecars_needed_before_transport(self):
        source = self.folder / "staging-source"
        source.mkdir()
        case = prepare_case("stop-turn-nominal", source, experimental_stop_turn=True)
        target = self.folder / "batch-stage"
        target.mkdir()
        trajectory, summary = stage_case_artifacts(case, target)
        stage_sitl_acknowledgement(case, target, trajectory, summary)
        self.assertEqual(
            load_planner_summary(summary, trajectory, experimental_stop_turn=True), 20.0
        )
        (target / "verification.json").unlink()
        with self.assertRaises((ValueError, FileNotFoundError)):
            load_planner_summary(summary, trajectory, experimental_stop_turn=True)

    def test_default_runner_and_campaign_still_refuse_unqualified_family(self):
        with self.assertRaisesRegex(ValueError, "opt-in"):
            load_planner_summary(
                self.folder / "summary.json", self.folder / "trajectory.csv"
            )
        staged = self.folder / "staged"
        staged.mkdir()
        with self.assertRaisesRegex(ValueError, "experimental-stop-turn"):
            prepare_case("reviewed-plan", staged, self.folder)

    def test_rehashed_tampered_derivative_still_fails_independent_reconstruction(self):
        value = deepcopy(self.plan)
        reader = csv.DictReader(io.StringIO(value["trajectory_csv"]))
        fields, rows = reader.fieldnames, list(reader)
        rows[10]["vx"] = "0.25"
        stream = io.StringIO()
        writer = csv.DictWriter(stream, fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
        value["trajectory_csv"] = stream.getvalue()
        digest = hashlib.sha256(value["trajectory_csv"].encode()).hexdigest()
        value["summary"]["controller_trajectory_sha256"] = digest
        value["verification"]["controller_trajectory_sha256"] = digest
        value["summary"]["verification_report_sha256"] = canonical_sha256(
            value["verification"]
        )
        self.write(value)
        with self.assertRaisesRegex(ValueError, "serialized states"):
            audit_stop_turn_artifacts(self.folder)

    def test_full_mission_cannot_be_mistaken_for_route_only(self):
        value = deepcopy(self.plan)
        value["trajectory_csv"] = value["mission"]["trajectory_csv"]
        digest = hashlib.sha256(value["trajectory_csv"].encode()).hexdigest()
        value["summary"]["controller_trajectory_sha256"] = digest
        value["verification"]["controller_trajectory_sha256"] = digest
        value["summary"]["verification_report_sha256"] = canonical_sha256(
            value["verification"]
        )
        self.write(value)
        with self.assertRaisesRegex(ValueError, "route-only CSV"):
            audit_stop_turn_artifacts(self.folder)

    def test_tampered_models_phases_and_intent_fail_binding(self):
        for key in ("vehicle_model", "environment", "intent", "verification"):
            value = deepcopy(self.plan)
            value[key]["tamper"] = True
            self.write(value)
            with self.assertRaises(ValueError):
                audit_stop_turn_artifacts(self.folder)
