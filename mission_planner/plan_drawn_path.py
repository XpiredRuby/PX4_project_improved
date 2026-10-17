"""Compile a strict drawn-polyline intent into a verified controller trajectory."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import uuid

from .contracts import (
    EnvironmentSnapshot,
    MissionIntent,
    VehicleModel,
    canonical_sha256,
    load_contract,
    validate_contract_set,
    validate_planning_inputs,
)
from .export import save_controller_csv
from .geometry import build_smooth_path
from .retiming import retime_stop_path
from .verification import verify_candidate_trajectory


def _write_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    try:
        temporary.write_text(
            json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(path)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def compile_drawn_path(
    *,
    intent_path: Path,
    vehicle_path: Path,
    environment_path: Path,
    trajectory_path: Path,
    report_path: Path,
    summary_path: Path,
    control_spacing_m: float,
    verification_resolution_m: float,
    sample_period_s: float,
    limit_scale: float,
    controller_cruise_height_m: float,
    deviation_utilization: float,
) -> dict:
    intent = load_contract(intent_path, MissionIntent.from_dict)
    vehicle = load_contract(vehicle_path, VehicleModel.from_dict)
    environment = load_contract(environment_path, EnvironmentSnapshot.from_dict)
    validate_planning_inputs(intent, vehicle, environment)
    if (
        not math.isfinite(controller_cruise_height_m)
        or controller_cruise_height_m <= 0.0
    ):
        raise ValueError("controller_cruise_height_m must be finite and positive")
    start = intent.path.points[0]
    if (
        abs(start.north_m) > 1e-9
        or abs(start.east_m) > 1e-9
        or abs(start.down_m + controller_cruise_height_m) > 1e-9
    ):
        raise ValueError(
            "path must start at LOCAL_NED "
            f"(0, 0, {-controller_cruise_height_m}) to match controller rebasing"
        )
    geometry = build_smooth_path(
        intent.path,
        control_spacing_m=control_spacing_m,
        verification_resolution_m=verification_resolution_m,
        deviation_utilization=deviation_utilization,
    )
    candidate = retime_stop_path(
        geometry,
        intent.motion,
        vehicle,
        sample_period_s=sample_period_s,
        limit_scale=limit_scale,
    )
    report = verify_candidate_trajectory(
        geometry,
        candidate,
        intent.motion,
        vehicle,
        environment,
        maximum_deviation_m=intent.path.maximum_deviation_m,
        maximum_limit_scale=limit_scale,
        fidelity_resolution_m=verification_resolution_m,
        controller_cruise_height_m=controller_cruise_height_m,
        maximum_sample_period_s=sample_period_s,
    )
    validate_contract_set(
        intent,
        vehicle,
        environment,
        report,
        candidate_trajectory_sha256=candidate.sha256,
    )

    resolved_outputs = {
        trajectory_path.resolve(),
        report_path.resolve(),
        summary_path.resolve(),
    }
    if len(resolved_outputs) != 3:
        raise ValueError("trajectory, report, and summary paths must be distinct")
    generation = uuid.uuid4().hex
    trajectory_stage = trajectory_path.with_name(
        trajectory_path.name + f".pending-{generation}"
    )
    report_stage = report_path.with_name(
        report_path.name + f".pending-{generation}"
    )
    summary_stage = summary_path.with_name(
        summary_path.name + f".pending-{generation}"
    )
    stages = (trajectory_stage, report_stage, summary_stage)
    for stage in stages:
        stage.unlink(missing_ok=True)
    try:
        save_controller_csv(trajectory_stage, candidate)
        trajectory_sha256 = hashlib.sha256(
            trajectory_stage.read_bytes()
        ).hexdigest()
        _write_json(report_stage, report.to_dict())
        summary = {
            "schema_version": 1,
            "release_class": "SITL_ONLY",
            "mission_intent_sha256": canonical_sha256(intent),
            "vehicle_model_sha256": canonical_sha256(vehicle),
            "environment_sha256": canonical_sha256(environment),
            "candidate_trajectory_sha256": candidate.sha256,
            "controller_trajectory_sha256": trajectory_sha256,
            "verification_report_sha256": canonical_sha256(report),
            "point_count": len(candidate.points),
            "geometry": asdict(geometry.fidelity),
            "smoothing": asdict(geometry.smoothing),
            "retiming": asdict(candidate.summary),
            "controller_cruise_height_m": controller_cruise_height_m,
            "limitations": [
                "fixed yaw only",
                "open stop-to-stop execution only",
                "controller cruise height must match the compiled value",
                "no obstacle-clearance claim without a verified map pipeline",
                "SITL_ONLY",
            ],
        }
        _write_json(summary_stage, summary)
        # Publish as a rollback-capable set, with the executable trajectory
        # last. Normal I/O failures restore every previous generation.
        publications = (
            (report_stage, report_path),
            (summary_stage, summary_path),
            (trajectory_stage, trajectory_path),
        )
        backup_suffix = f".backup-{generation}"
        backups: dict[Path, Path] = {}
        published: set[Path] = set()
        try:
            for _, target in publications:
                if target.exists():
                    backup = target.with_name(target.name + backup_suffix)
                    target.replace(backup)
                    backups[target] = backup
            for stage, target in publications:
                stage.replace(target)
                published.add(target)
        except BaseException:
            for target in published:
                target.unlink(missing_ok=True)
            for target, backup in backups.items():
                if backup.exists():
                    backup.replace(target)
            raise
        else:
            for backup in backups.values():
                backup.unlink(missing_ok=True)
    finally:
        for stage in stages:
            stage.unlink(missing_ok=True)
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--intent", required=True, type=Path)
    parser.add_argument("--vehicle", required=True, type=Path)
    parser.add_argument("--environment", required=True, type=Path)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--control-spacing-m", type=float, default=0.25)
    parser.add_argument("--verification-resolution-m", type=float, default=0.01)
    parser.add_argument("--sample-period-s", type=float, default=0.05)
    parser.add_argument("--limit-scale", type=float, default=0.9)
    parser.add_argument("--controller-cruise-height-m", type=float, default=17.0)
    parser.add_argument("--deviation-utilization", type=float, default=0.75)
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    summary = compile_drawn_path(
        intent_path=args.intent,
        vehicle_path=args.vehicle,
        environment_path=args.environment,
        trajectory_path=args.trajectory,
        report_path=args.report,
        summary_path=args.summary,
        control_spacing_m=args.control_spacing_m,
        verification_resolution_m=args.verification_resolution_m,
        sample_period_s=args.sample_period_s,
        limit_scale=args.limit_scale,
        controller_cruise_height_m=args.controller_cruise_height_m,
        deviation_utilization=args.deviation_utilization,
    )
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
