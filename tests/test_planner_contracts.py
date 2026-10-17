"""Contract, determinism, and integrity tests for the general planner boundary."""
# ruff: noqa: E402

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.append(str(ROOT / "trajectory_generator"))

from mission_planner.contracts import (
    EnvironmentSnapshot,
    MissionIntent,
    PlanArtifact,
    VehicleModel,
    VerificationReport,
    canonical_json_bytes,
    canonical_sha256,
    load_contract,
    validate_contract_set,
)
from mission_planner.json_schemas import SCHEMAS, write_schemas
from plan_waypoint_mission import compile_plan


ZERO_DIGEST = "0" * 64


def frame():
    return {
        "name": "LOCAL_NED",
        "units": "m",
        "origin_id": "launch-001",
        "altitude_datum": "launch",
    }


def mission():
    return {
        "schema_version": 1,
        "mission_id": "drawn-circle-001",
        "release_class": "SITL_ONLY",
        "frame": frame(),
        "path": {
            "kind": "polyline",
            "closure": "open_stop",
            "maximum_deviation_m": 0.25,
            "points": [
                {
                    "point_id": "start",
                    "north_m": 0.0,
                    "east_m": 0.0,
                    "down_m": -8.0,
                    "hard": True,
                },
                {
                    "point_id": "finish",
                    "north_m": 10.0,
                    "east_m": 0.0,
                    "down_m": -8.0,
                    "hard": True,
                },
            ],
        },
        "motion": {
            "target_speed_m_s": 0.8,
            "maximum_speed_m_s": 1.0,
            "yaw_mode": "tangent",
            "fixed_yaw_deg": None,
            "time_mode": "minimize",
            "requested_duration_s": None,
        },
        "vehicle_model_id": "px4-x500-sitl-v1",
        "environment_id": "empty-world-v1",
        "objective_priority": [
            "legal_and_safety",
            "vehicle_and_environment",
            "intent_fidelity",
            "tracking_margin_and_energy",
            "duration",
        ],
    }


def vehicle():
    return {
        "schema_version": 1,
        "vehicle_model_id": "px4-x500-sitl-v1",
        "configuration": "PX4 v1.17 Gazebo x500 research model",
        "mass_kg": 2.0,
        "maximum_speed_m_s": 3.0,
        "maximum_acceleration_m_s2": 1.0,
        "maximum_jerk_m_s3": 1.5,
        "maximum_yaw_rate_deg_s": 45.0,
        "maximum_yaw_acceleration_deg_s2": 60.0,
        "maximum_yaw_jerk_deg_s3": 120.0,
        "maximum_tilt_deg": 25.0,
        "maximum_body_rate_deg_s": 90.0,
        "minimum_thrust_n": 0.0,
        "maximum_thrust_n": 40.0,
        "identified_for_physical_flight": False,
    }


def environment():
    return {
        "schema_version": 1,
        "environment_id": "empty-world-v1",
        "frame": frame(),
        "max_radius_m": 40.0,
        "max_height_above_launch_m": 30.0,
        "max_drop_below_launch_m": 2.0,
        "max_wind_m_s": 5.0,
        "minimum_clearance_m": 5.0,
        "obstacle_map_sha256": None,
        "obstacle_map_verified": False,
    }


def report(*, accepted=True, trajectory_digest=ZERO_DIGEST):
    return {
        "schema_version": 1,
        "verifier_version": "planner-verifier-v1",
        "candidate_trajectory_sha256": trajectory_digest,
        "accepted": accepted,
        "checks": [
            {
                "requirement_id": "VER-003",
                "status": "PASS" if accepted else "FAIL",
                "hard": True,
                "margin": 0.5 if accepted else -0.5,
                "units": "m",
                "detail": "Synthetic contract-boundary test.",
            }
        ],
    }


class PlannerContractTests(unittest.TestCase):
    def test_valid_contract_set_is_immutable_and_canonical(self):
        intent = MissionIntent.from_dict(mission())
        model = VehicleModel.from_dict(vehicle())
        snapshot = EnvironmentSnapshot.from_dict(environment())
        verification = VerificationReport.from_dict(report())

        validate_contract_set(
            intent,
            model,
            snapshot,
            verification,
            candidate_trajectory_sha256=ZERO_DIGEST,
        )
        first = canonical_json_bytes(intent)
        reordered = dict(reversed(list(mission().items())))
        second = canonical_json_bytes(MissionIntent.from_dict(reordered))
        self.assertEqual(first, second)
        self.assertEqual(canonical_sha256(intent), canonical_sha256(reordered))
        with self.assertRaisesRegex(Exception, "cannot assign"):
            intent.mission_id = "changed"

    def test_unknown_missing_nonfinite_and_wrong_types_fail_closed(self):
        variants = []
        unknown = mission()
        unknown["hidden_default"] = True
        variants.append((unknown, "unknown fields"))
        missing = mission()
        del missing["frame"]
        variants.append((missing, "missing fields"))
        nonfinite = mission()
        nonfinite["path"]["points"][0]["north_m"] = math.nan
        variants.append((nonfinite, "finite"))
        boolean_number = mission()
        boolean_number["motion"]["maximum_speed_m_s"] = True
        variants.append((boolean_number, "numeric"))
        duplicate = mission()
        duplicate["path"]["points"][1]["point_id"] = "start"
        variants.append((duplicate, "unique"))
        wrong_version = mission()
        wrong_version["schema_version"] = 1.0
        variants.append((wrong_version, "integer"))
        wrong_order = mission()
        wrong_order["objective_priority"].reverse()
        variants.append((wrong_order, "safety-first"))

        for raw, message in variants:
            with self.subTest(message=message), self.assertRaisesRegex(
                ValueError, message
            ):
                MissionIntent.from_dict(raw)

    def test_time_speed_and_periodic_semantics_are_explicit(self):
        target = mission()
        target["motion"]["target_speed_m_s"] = 2.0
        with self.assertRaisesRegex(ValueError, "target speed"):
            MissionIntent.from_dict(target)

        duration = mission()
        duration["motion"]["time_mode"] = "exact"
        with self.assertRaisesRegex(ValueError, "duration"):
            MissionIntent.from_dict(duration)

        yaw = mission()
        yaw["motion"].update(yaw_mode="fixed", fixed_yaw_deg=None)
        with self.assertRaisesRegex(ValueError, "fixed yaw"):
            MissionIntent.from_dict(yaw)

        periodic = mission()
        periodic["path"]["closure"] = "periodic"
        with self.assertRaisesRegex(ValueError, "periodic path endpoints"):
            MissionIntent.from_dict(periodic)
        periodic["path"]["points"][1]["north_m"] = 0.1
        self.assertEqual(
            MissionIntent.from_dict(periodic).path.closure,
            "periodic",
        )

    def test_cross_contract_references_and_envelopes_fail_closed(self):
        intent = MissionIntent.from_dict(mission())
        model_raw = vehicle()
        environment_raw = environment()
        verification = VerificationReport.from_dict(report())

        model_raw["vehicle_model_id"] = "another-model"
        with self.assertRaisesRegex(ValueError, "vehicle model identifiers"):
            validate_contract_set(
                intent,
                VehicleModel.from_dict(model_raw),
                EnvironmentSnapshot.from_dict(environment_raw),
                verification,
                candidate_trajectory_sha256=ZERO_DIGEST,
            )

        model_raw = vehicle()
        model_raw["maximum_speed_m_s"] = 0.5
        with self.assertRaisesRegex(ValueError, "speed exceeds"):
            validate_contract_set(
                intent,
                VehicleModel.from_dict(model_raw),
                EnvironmentSnapshot.from_dict(environment_raw),
                verification,
                candidate_trajectory_sha256=ZERO_DIGEST,
            )

        environment_raw["max_radius_m"] = 5.0
        with self.assertRaisesRegex(ValueError, "environment radius"):
            validate_contract_set(
                intent,
                VehicleModel.from_dict(vehicle()),
                EnvironmentSnapshot.from_dict(environment_raw),
                verification,
                candidate_trajectory_sha256=ZERO_DIGEST,
            )

        with self.assertRaisesRegex(ValueError, "another trajectory"):
            validate_contract_set(
                intent,
                VehicleModel.from_dict(vehicle()),
                EnvironmentSnapshot.from_dict(environment()),
                verification,
                candidate_trajectory_sha256="1" * 64,
            )

    def test_map_identity_and_verification_must_agree(self):
        for digest, verified in ((ZERO_DIGEST, False), (None, True)):
            raw = environment()
            raw.update(
                obstacle_map_sha256=digest,
                obstacle_map_verified=verified,
            )
            with self.subTest(
                digest=digest, verified=verified
            ), self.assertRaisesRegex(ValueError, "inconsistent"):
                EnvironmentSnapshot.from_dict(raw)

    def test_report_acceptance_is_derived_from_hard_checks(self):
        inconsistent = report(accepted=True)
        inconsistent["checks"][0]["status"] = "FAIL"
        with self.assertRaisesRegex(ValueError, "does not match"):
            VerificationReport.from_dict(inconsistent)
        duplicate = report()
        duplicate["checks"].append(deepcopy(duplicate["checks"][0]))
        with self.assertRaisesRegex(ValueError, "unique"):
            VerificationReport.from_dict(duplicate)

    def test_artifact_hash_detects_tampering_and_cannot_approve_flight(self):
        artifact = PlanArtifact.create(
            mission_intent_sha256="1" * 64,
            vehicle_model_sha256="2" * 64,
            environment_sha256="3" * 64,
            candidate_trajectory_sha256="4" * 64,
            verification_report_sha256="5" * 64,
            simulation_evidence_sha256="6" * 64,
        )
        restored = PlanArtifact.from_dict(artifact.to_dict())
        self.assertEqual(restored, artifact)
        self.assertFalse(restored.approved)
        self.assertEqual(restored.release_class, "SITL_ONLY")

        tampered = artifact.to_dict()
        tampered["simulation_evidence_sha256"] = "7" * 64
        with self.assertRaisesRegex(ValueError, "does not match"):
            PlanArtifact.from_dict(tampered)
        approved = artifact.to_dict()
        approved.update(approved=True, approval_id="operator-1")
        with self.assertRaisesRegex(ValueError, "cannot carry flight approval"):
            PlanArtifact.from_dict(approved)

    def test_canonical_json_rejects_nonfinite_values(self):
        with self.assertRaises(ValueError):
            canonical_json_bytes({"unsafe": math.inf})

    def test_loader_rejects_nonstandard_constants_and_malformed_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "intent.json"
            path.write_text('{"unsafe": NaN}', encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "non-finite JSON"):
                load_contract(path, MissionIntent.from_dict)
            path.write_text("{", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "cannot load contract"):
                load_contract(path, MissionIntent.from_dict)

    def test_checked_in_json_schemas_match_generator(self):
        checked_in = ROOT / "schemas" / "planner"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            write_schemas(output)
            for name in SCHEMAS:
                with self.subTest(name=name):
                    self.assertEqual(
                        (checked_in / name).read_bytes(),
                        (output / name).read_bytes(),
                    )

    def test_existing_five_stop_baseline_is_pinned(self):
        baseline = json.loads(
            (
                ROOT
                / "schemas"
                / "planner"
                / "five-stop-baseline-v1.json"
            ).read_text(encoding="utf-8")
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "trajectory.csv"
            summary = compile_plan(ROOT / baseline["source"], output)
        self.assertEqual(summary["source_plan_sha256"], baseline["source_plan_sha256"])
        self.assertEqual(summary["trajectory_sha256"], baseline["trajectory_sha256"])
        self.assertEqual(summary["point_count"], baseline["point_count"])
        self.assertEqual(summary["duration_s"], baseline["duration_s"])


if __name__ == "__main__":
    unittest.main()
