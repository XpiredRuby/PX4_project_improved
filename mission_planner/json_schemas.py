"""Generate checked-in JSON Schemas for planner boundary contracts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .contracts import OBJECTIVE_PRIORITY


DRAFT = "https://json-schema.org/draft/2020-12/schema"
SCHEMA_BASE = "https://example.invalid/px4-planner"


def _strict_object(properties, required=None):
    return {
        "type": "object",
        "additionalProperties": False,
        "properties": properties,
        "required": list(required or properties),
    }


def _positive(description: str):
    return {"type": "number", "exclusiveMinimum": 0, "description": description}


def _nonnegative(description: str):
    return {"type": "number", "minimum": 0, "description": description}


def _identifier():
    return {
        "type": "string",
        "pattern": r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$",
    }


def _digest():
    return {"type": "string", "pattern": "^[0-9a-f]{64}$"}


def _frame():
    return _strict_object({
        "name": {"const": "LOCAL_NED"},
        "units": {"const": "m"},
        "origin_id": _identifier(),
        "altitude_datum": {"const": "launch"},
    })


def mission_intent_schema():
    point = _strict_object({
        "point_id": _identifier(),
        "north_m": {"type": "number"},
        "east_m": {"type": "number"},
        "down_m": {"type": "number"},
        "hard": {"type": "boolean"},
    })
    path = _strict_object({
        "kind": {"enum": ["waypoints", "polyline"]},
        "closure": {"enum": ["open_stop", "open_depart", "periodic"]},
        "maximum_deviation_m": _nonnegative("Maximum accepted path error."),
        "points": {
            "type": "array",
            "minItems": 2,
            "maxItems": 10_000,
            "items": point,
        },
    })
    motion = _strict_object({
        "target_speed_m_s": {
            "anyOf": [_positive("Preferred speed."), {"type": "null"}]
        },
        "maximum_speed_m_s": _positive("Hard mission speed ceiling."),
        "yaw_mode": {
            "enum": ["fixed", "tangent", "unconstrained"]
        },
        "fixed_yaw_deg": {
            "anyOf": [
                {"type": "number", "minimum": -180, "maximum": 180},
                {"type": "null"},
            ]
        },
        "time_mode": {"enum": ["minimize", "exact", "maximum", "none"]},
        "requested_duration_s": {
            "anyOf": [_positive("Requested mission duration."), {"type": "null"}]
        },
    })
    schema = _strict_object({
        "schema_version": {"const": 1},
        "mission_id": _identifier(),
        "release_class": {"const": "SITL_ONLY"},
        "frame": _frame(),
        "path": path,
        "motion": motion,
        "vehicle_model_id": _identifier(),
        "environment_id": _identifier(),
        "objective_priority": {
            "type": "array",
            "prefixItems": [{"const": item} for item in OBJECTIVE_PRIORITY],
            "items": False,
            "minItems": len(OBJECTIVE_PRIORITY),
            "maxItems": len(OBJECTIVE_PRIORITY),
        },
    })
    schema.update({
        "$schema": DRAFT,
        "$id": f"{SCHEMA_BASE}/mission-intent-v1.schema.json",
        "title": "MissionIntent v1",
    })
    return schema


def vehicle_model_schema():
    schema = _strict_object({
        "schema_version": {"const": 1},
        "vehicle_model_id": _identifier(),
        "configuration": {"type": "string", "minLength": 1, "maxLength": 256},
        "mass_kg": _positive("All-up modeled mass."),
        "maximum_speed_m_s": _positive("Verified model speed ceiling."),
        "maximum_acceleration_m_s2": _positive(
            "Verified translational acceleration ceiling."
        ),
        "maximum_jerk_m_s3": _positive("Verified translational jerk ceiling."),
        "maximum_yaw_rate_deg_s": _positive(
            "Verified yaw-rate ceiling, at or below the controller limit."
        ),
        "maximum_yaw_acceleration_deg_s2": _positive(
            "Verified yaw-acceleration ceiling, at or below the controller limit."
        ),
        "maximum_yaw_jerk_deg_s3": _positive(
            "Verified yaw-jerk ceiling, at or below the controller limit."
        ),
        "maximum_tilt_deg": _positive("Verified body tilt ceiling."),
        "maximum_body_rate_deg_s": _positive("Verified body-rate ceiling."),
        "minimum_thrust_n": _nonnegative("Minimum modeled collective thrust."),
        "maximum_thrust_n": _positive("Maximum modeled collective thrust."),
        "identified_for_physical_flight": {"type": "boolean"},
    })
    schema.update({
        "$schema": DRAFT,
        "$id": f"{SCHEMA_BASE}/vehicle-model-v1.schema.json",
        "title": "VehicleModel v1",
    })
    return schema


def environment_schema():
    schema = _strict_object({
        "schema_version": {"const": 1},
        "environment_id": _identifier(),
        "frame": _frame(),
        "max_radius_m": _positive("Maximum horizontal radius from launch."),
        "max_height_above_launch_m": _positive("Maximum height above launch."),
        "max_drop_below_launch_m": _nonnegative("Maximum drop below launch."),
        "max_wind_m_s": _nonnegative("Maximum supported wind."),
        "minimum_clearance_m": _nonnegative("Required inflated clearance."),
        "obstacle_map_sha256": {
            "anyOf": [_digest(), {"type": "null"}]
        },
        "obstacle_map_verified": {"type": "boolean"},
    })
    schema.update({
        "$schema": DRAFT,
        "$id": f"{SCHEMA_BASE}/environment-snapshot-v1.schema.json",
        "title": "EnvironmentSnapshot v1",
    })
    return schema


def verification_report_schema():
    check = _strict_object({
        "requirement_id": _identifier(),
        "status": {"enum": ["PASS", "FAIL"]},
        "hard": {"type": "boolean"},
        "margin": {"anyOf": [{"type": "number"}, {"type": "null"}]},
        "units": {
            "anyOf": [
                {"type": "string", "minLength": 1, "maxLength": 32},
                {"type": "null"},
            ]
        },
        "detail": {"type": "string", "minLength": 1, "maxLength": 1024},
    })
    schema = _strict_object({
        "schema_version": {"const": 1},
        "verifier_version": _identifier(),
        "candidate_trajectory_sha256": _digest(),
        "accepted": {"type": "boolean"},
        "checks": {
            "type": "array",
            "minItems": 1,
            "items": check,
        },
    })
    schema.update({
        "$schema": DRAFT,
        "$id": f"{SCHEMA_BASE}/verification-report-v1.schema.json",
        "title": "VerificationReport v1",
    })
    return schema


def plan_artifact_schema():
    schema = _strict_object({
        "schema_version": {"const": 1},
        "release_class": {"const": "SITL_ONLY"},
        "mission_intent_sha256": _digest(),
        "vehicle_model_sha256": _digest(),
        "environment_sha256": _digest(),
        "candidate_trajectory_sha256": _digest(),
        "verification_report_sha256": _digest(),
        "simulation_evidence_sha256": _digest(),
        "approved": {"const": False},
        "approval_id": {"type": "null"},
        "artifact_sha256": _digest(),
    })
    schema.update({
        "$schema": DRAFT,
        "$id": f"{SCHEMA_BASE}/plan-artifact-v1.schema.json",
        "title": "PlanArtifact v1",
    })
    return schema


SCHEMAS = {
    "mission-intent-v1.schema.json": mission_intent_schema,
    "vehicle-model-v1.schema.json": vehicle_model_schema,
    "environment-snapshot-v1.schema.json": environment_schema,
    "verification-report-v1.schema.json": verification_report_schema,
    "plan-artifact-v1.schema.json": plan_artifact_schema,
}


def write_schemas(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    for name, build in SCHEMAS.items():
        (output / name).write_text(
            json.dumps(build(), indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    args = parser.parse_args(argv)
    write_schemas(args.output)


if __name__ == "__main__":
    main()
