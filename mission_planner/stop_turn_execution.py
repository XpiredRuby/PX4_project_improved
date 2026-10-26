"""Experimental route-only SITL adapter for the constructive polynomial family.

This checks retained artifacts before transport. It does not run a simulator,
qualify tracking, authorize hardware, or replace the runner's native lifecycle.
"""

from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path

import numpy as np
from numpy.polynomial import Polynomial

from .contracts import (
    EnvironmentSnapshot,
    MissionIntent,
    VehicleModel,
    canonical_sha256,
    load_contract,
    validate_planning_inputs,
)
from .export import FIELDS
from .lifecycle_verification import verify_phase


def progress_polynomial():
    """Solve rest-to-rest C3 boundary conditions independently of the generator."""
    matrix, target = [], []
    for u, position in ((0.0, 0.0), (1.0, 1.0)):
        for order in range(4):
            matrix.append(
                [
                    0.0
                    if k < order
                    else math.factorial(k)
                    / math.factorial(k - order)
                    * u ** (k - order)
                    for k in range(8)
                ]
            )
            target.append(position if order == 0 else 0.0)
    return Polynomial(np.linalg.solve(matrix, target))


def derivative_peak(poly, order):
    """Numerical critical-root extrema; diverse from the Bernstein certificate."""
    derivative = poly.deriv(order)
    roots = derivative.deriv().roots()
    coordinates = [
        0.0,
        1.0,
        *(float(r.real) for r in roots if abs(r.imag) < 1e-8 and 0.0 < r.real < 1.0),
    ]
    return float(np.max(np.abs(derivative(coordinates))))


def sample_stop_turn_reference(phases, query):
    """Independently evaluate phase positions/yaw on the recorded mission clock.

    Queries outside coverage remain NaN rather than inventing measurements.
    This is a reference evaluator, never a simulator feedback source.
    """
    query = np.asarray(query, dtype=float)
    states = np.full((len(query), 4), np.nan)
    q, clock = progress_polynomial(), 0.0
    for phase in phases:
        duration = phase["duration_s"]
        mask = (query >= clock) & (query <= clock + duration)
        u = (query[mask] - clock) / duration
        start, end = np.asarray(phase["start"]), np.asarray(phase["end"])
        states[mask] = start + q(u)[:, None] * (end - start)
        clock += duration
    return states


def audit_stop_turn_artifacts(
    folder, trajectory_path=None, *, for_execution=True, summary_path=None
):
    folder = Path(folder)
    summary = load_contract(
        summary_path if summary_path is not None else folder / "summary.json", dict
    )
    verification = load_contract(folder / "verification.json", dict)
    raw_intent = load_contract(folder / "intent.json", dict)
    intent = MissionIntent.from_dict(raw_intent)
    vehicle = load_contract(folder / "vehicle_model.json", VehicleModel.from_dict)
    environment = load_contract(
        folder / "environment.json", EnvironmentSnapshot.from_dict
    )
    validate_planning_inputs(intent, vehicle, environment)
    if (
        summary.get("release_class") != "SITL_ONLY"
        or verification.get("accepted") is not True
        or verification.get("method") != "stop_turn_septic_bernstein"
    ):
        raise ValueError(
            "Experimental adapter requires an accepted SITL stop/turn certificate"
        )
    for key, value in (
        ("verification_report_sha256", verification),
        ("mission_intent_sha256", raw_intent),
        ("vehicle_model_sha256", vehicle),
        ("environment_sha256", environment),
    ):
        if canonical_sha256(value) != summary.get(key):
            raise ValueError("Stop/turn artifact digest mismatch: " + key)
    phases = verification.get("phases", [])
    if not phases or canonical_sha256(phases) != summary.get(
        "candidate_trajectory_sha256"
    ):
        raise ValueError("Stop/turn phases do not match the retained candidate")
    if for_execution and intent.motion.yaw_mode != "fixed":
        raise ValueError(
            "Experimental connected stop/turn adapter currently requires fixed heading"
        )
    path = (
        Path(trajectory_path)
        if trajectory_path is not None
        else folder / "trajectory.csv"
    )
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != summary.get(
        "controller_trajectory_sha256"
    ) or digest != verification.get("controller_trajectory_sha256"):
        raise ValueError("Stop/turn route CSV digest mismatch")
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        if tuple(reader.fieldnames or []) != FIELDS:
            raise ValueError(
                "Stop/turn adapter accepts route-only CSV, never full lifecycle CSV"
            )
        rows = np.array([[float(r[k]) for k in FIELDS] for r in reader])
    if (
        rows.ndim != 2
        or len(rows) < 2
        or len(rows) > 20001
        or not np.isfinite(rows).all()
        or rows[0, 0] != 0.0
        or np.any(np.diff(rows[:, 0]) <= 0.0)
        or np.max(np.diff(rows[:, 0])) > 0.05000001
        or np.any(rows[:, -1] != 1.0)
    ):
        raise ValueError(
            "Stop/turn route has invalid samples, clock or contract version"
        )
    height = summary.get("controller_cruise_height_m")
    if (
        not isinstance(height, (int, float))
        or isinstance(height, bool)
        or not math.isfinite(height)
        or height <= 0.0
        or not np.allclose(rows[:, 3], -height, atol=1e-9, rtol=0.0)
    ):
        raise ValueError(
            "Experimental adapter requires constant verified cruise height"
        )
    # Match every translational phase to effective intent vertices and lap order.
    points = [[p.north_m, p.east_m, p.down_m] for p in intent.path.points]
    if intent.path.closure == "periodic":
        points += [
            p for _ in range(summary["retiming"]["loop_count"] - 1) for p in points[1:]
        ]
    ordered = [points[0]]
    for point in points[1:]:
        if math.dist(point, ordered[-1]) > 1e-10:
            ordered.append(point)
    visited = [phases[0]["start"][:3]] + [
        p["end"][:3] for p in phases if p["name"] == "LEG"
    ]
    if len(ordered) != len(visited) or not np.allclose(
        ordered, visited, rtol=0.0, atol=1e-9
    ):
        raise ValueError("Stop/turn phases do not visit the effective route in order")
    q = progress_polynomial()
    clock, previous, max_error = 0.0, None, 0.0
    maximum = np.zeros(3)
    groups = ((1, 2, 3, 4), (5, 6, 7, 8), (9, 10, 11, 15), (12, 13, 14, 16))
    spans = summary.get("route_phases", [])
    if len(spans) != len(phases):
        raise ValueError("Stop/turn phase timing table mismatch")
    for phase, span in zip(phases, spans, strict=True):
        start, end = np.asarray(phase["start"]), np.asarray(phase["end"])
        duration = phase["duration_s"]
        if (
            phase["name"] not in ("LEG", "TURN", "HOLD")
            or start.shape != (4,)
            or end.shape != (4,)
            or not np.isfinite([*start, *end, duration]).all()
            or duration <= 0.0
        ):
            raise ValueError("Invalid stop/turn phase")
        if previous is not None and not np.allclose(
            start, previous, atol=1e-9, rtol=0.0
        ):
            raise ValueError("Stop/turn phase joins are discontinuous")
        if (
            not math.isclose(span["start_time_s"], clock, abs_tol=1e-9)
            or not math.isclose(span["end_time_s"], clock + duration, abs_tol=1e-9)
            or span["name"] != phase["name"]
        ):
            raise ValueError("Stop/turn phase clocks disagree")
        delta = end - start
        if intent.motion.yaw_mode == "fixed" and abs(delta[3]) > 1e-10:
            raise ValueError("Fixed heading adapter cannot execute a changing heading")
        requested_cap = 0.9 * min(
            intent.motion.maximum_speed_m_s,
            intent.motion.target_speed_m_s or vehicle.maximum_speed_m_s,
            vehicle.maximum_speed_m_s,
        )
        if phase["speed_limit_m_s"] > requested_cap + 1e-9:
            raise ValueError("Phase speed cap exceeds the verified intent")
        yaw_peaks = [
            derivative_peak(q, k) * abs(delta[3]) / duration**k for k in (1, 2, 3)
        ]
        yaw_limits = [
            math.radians(getattr(vehicle, name))
            for name in (
                "maximum_yaw_rate_deg_s",
                "maximum_yaw_acceleration_deg_s2",
                "maximum_yaw_jerk_deg_s3",
            )
        ]
        if np.any(np.asarray(yaw_peaks) > np.asarray(yaw_limits) + 1e-8):
            raise ValueError("Independent root-extrema angular dynamics check failed")
        report = verify_phase(phase, vehicle, environment)
        if not report["accepted"]:
            raise ValueError(
                "Recomputed continuous stop/turn physical certificate failed"
            )
        peaks = np.array(
            [
                derivative_peak(q, k) * np.linalg.norm(delta[:3]) / duration**k
                for k in (1, 2, 3)
            ]
        )
        limits = [
            min(phase["speed_limit_m_s"], vehicle.maximum_speed_m_s),
            vehicle.maximum_acceleration_m_s2,
            vehicle.maximum_jerk_m_s3,
        ]
        if np.any(peaks > np.asarray(limits) + 1e-8):
            raise ValueError("Independent root-extrema stop/turn dynamics check failed")
        maximum = np.maximum(maximum, peaks)
        mask = (rows[:, 0] >= clock - 1e-9) & (rows[:, 0] <= clock + duration + 1e-9)
        segment = rows[mask]
        if (
            len(segment) < 2
            or abs(segment[0, 0] - clock) > 1e-9
            or abs(segment[-1, 0] - clock - duration) > 1e-9
        ):
            raise ValueError(
                "Every stop/turn boundary must be an exact exported sample"
            )
        u = np.clip((segment[:, 0] - clock) / duration, 0.0, 1.0)
        for order, columns in enumerate(groups):
            expected = q.deriv(order)(u)[:, None] * delta / duration**order
            if order == 0:
                expected += start
            error = float(np.max(np.abs(segment[:, columns] - expected)))
            max_error = max(max_error, error)
            if error > 2e-8:
                raise ValueError(
                    "Stop/turn serialized states disagree with phase polynomial"
                )
        previous = end
        clock += duration
    if (
        abs(rows[-1, 0] - clock) > 1e-9
        or abs(rows[0, 1]) + abs(rows[0, 2]) > 1e-9
        or np.max(np.abs(rows[[0, -1], 5:17])) > 1e-8
    ):
        raise ValueError(
            "Stop/turn route endpoints, duration or rest conditions failed"
        )
    return dict(
        accepted=True,
        scope="offline exact route adapter; connected tracking NOT VALIDATED",
        route_only=True,
        execution_adapter_eligible=for_execution,
        native_lifecycle_owner="offboard_runner",
        controller_trajectory_sha256=digest,
        samples=len(rows),
        phases=len(phases),
        maximum_serialized_state_error=max_error,
        root_extrema_speed_acceleration_jerk=maximum.tolist(),
        simulator_ran=False,
        physical_flight_approved=False,
    )
