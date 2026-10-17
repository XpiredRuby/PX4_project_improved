"""Compile a strict drawn-polyline intent into a verified controller trajectory."""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import uuid

from .contracts import (
    EnvironmentSnapshot,
    MissionIntent,
    PathIntent,
    VehicleModel,
    canonical_sha256,
    load_contract,
    validate_contract_set,
    validate_planning_inputs,
)
from .explain import PlanningError
from .export import save_controller_csv
from .geometry import build_smooth_path
from .retiming import concatenate_candidates, retime_stop_path
from .review import write_review
from .verification import merge_verification_reports, verify_candidate_trajectory


def _split_on_hard_points(path: PathIntent) -> tuple[PathIntent, ...]:
    """Break the stroke at every interior point the aircraft must land on.

    Each piece is flown rest-to-rest, so a hard point is a stop that the
    spline interpolates exactly. Endpoints are already stops.
    """
    interior = [
        index
        for index, point in enumerate(path.points)
        if point.hard and index not in (0, len(path.points) - 1)
    ]
    if not interior:
        return (path,)
    if path.closure == "periodic":
        raise PlanningError(
            "a periodic path cannot contain interior hard points",
            limiting_constraint="hard_point",
        )
    boundaries = [0, *interior, len(path.points) - 1]
    pieces = []
    for start, finish in zip(boundaries, boundaries[1:], strict=False):
        selected = path.points[start : finish + 1]
        if len(selected) < 2:
            raise PlanningError(
                "hard points must have a path between them",
                limiting_constraint="hard_point",
            )
        pieces.append(PathIntent.from_dict({
            "kind": path.kind,
            "closure": "open_stop",
            "maximum_deviation_m": path.maximum_deviation_m,
            "points": [asdict(point) for point in selected],
        }))
    return tuple(pieces)


def _segment_spacing(path: PathIntent, requested: float, piece_count: int) -> float:
    if piece_count == 1:
        return requested
    length = sum(
        math.dist(
            (left.north_m, left.east_m, left.down_m),
            (right.north_m, right.east_m, right.down_m),
        )
        for left, right in zip(path.points, path.points[1:], strict=False)
    )
    return min(requested, max(length, 1e-6) / 8.0)


def _segment_motions(motion, geometries, vehicle, *, limit_scale: float):
    """Share one requested duration across hard-point stops."""
    if len(geometries) == 1:
        return (motion,)
    probe = replace(motion, time_mode="minimize", requested_duration_s=None)
    minimums = [
        retime_stop_path(
            geometry, probe, vehicle, loop_count=1, limit_scale=limit_scale
        ).summary.minimum_feasible_duration_s
        for geometry in geometries
    ]
    total = sum(minimums)
    if motion.time_mode == "exact":
        if motion.requested_duration_s + 1e-12 < total:
            raise PlanningError(
                "requested exact duration is infeasible across hard-point stops; "
                f"minimum feasible duration is {total:.6f} s",
                limiting_constraint="duration",
                minimum_feasible_duration_s=total,
            )
        scale = motion.requested_duration_s / total
        return tuple(
            replace(
                motion,
                time_mode="exact",
                requested_duration_s=value * scale,
            )
            for value in minimums
        )
    if motion.time_mode == "maximum" and total > motion.requested_duration_s + 1e-12:
        raise PlanningError(
            "requested maximum duration is infeasible across hard-point stops; "
            f"minimum feasible duration is {total:.6f} s",
            limiting_constraint="duration",
            minimum_feasible_duration_s=total,
        )
    return tuple(
        replace(motion, time_mode="minimize", requested_duration_s=None)
        for _ in geometries
    )


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
    loop_count: int = 1,
    review_path: Path | None = None,
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
    pieces = _split_on_hard_points(intent.path)
    geometries = [
        build_smooth_path(
            piece,
            control_spacing_m=_segment_spacing(piece, control_spacing_m, len(pieces)),
            verification_resolution_m=verification_resolution_m,
            deviation_utilization=deviation_utilization,
        )
        for piece in pieces
    ]
    motions = _segment_motions(
        intent.motion, geometries, vehicle, limit_scale=limit_scale
    )
    planned = []
    reports = []
    for piece, geometry, piece_motion in zip(pieces, geometries, motions, strict=True):
        candidate = retime_stop_path(
            geometry,
            piece_motion,
            vehicle,
            sample_period_s=sample_period_s,
            limit_scale=limit_scale,
            loop_count=loop_count if piece.closure == "periodic" else 1,
        )
        reports.append(verify_candidate_trajectory(
            geometry,
            candidate,
            piece_motion,
            vehicle,
            environment,
            maximum_deviation_m=intent.path.maximum_deviation_m,
            maximum_limit_scale=limit_scale,
            fidelity_resolution_m=verification_resolution_m,
            controller_cruise_height_m=controller_cruise_height_m,
            maximum_sample_period_s=sample_period_s,
        ))
        planned.append(candidate)
    candidate = concatenate_candidates(tuple(planned))
    report = (
        reports[0]
        if len(reports) == 1
        else merge_verification_reports(tuple(reports), candidate.sha256)
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
            "geometry": (
                asdict(geometries[0].fidelity)
                if len(geometries) == 1
                else [asdict(item.fidelity) for item in geometries]
            ),
            "smoothing": (
                asdict(geometries[0].smoothing)
                if len(geometries) == 1
                else [asdict(item.smoothing) for item in geometries]
            ),
            "retiming": asdict(candidate.summary),
            "controller_cruise_height_m": controller_cruise_height_m,
            "limitations": [
                "yaw commands are added to the heading measured at launch; "
                "tangent mode requires the aircraft to start pointed along the initial path tangent",
                "a periodic path starts and stops at its seam; extra laps repeat that loop",
                "interior hard points are full stops",
                "open_depart and unconstrained yaw are refused",
                "controller cruise height must match the compiled value",
                "no obstacle-clearance claim without a verified map pipeline",
                "not a clearance to fly",
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
    if review_path is not None:
        write_review(review_path, summary, candidate)
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
    parser.add_argument("--loop-count", type=int, default=1)
    parser.add_argument("--review", type=Path)
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
        loop_count=args.loop_count,
        review_path=args.review,
    )
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
