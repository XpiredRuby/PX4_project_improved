"""Ground-to-ground planned mission, separate from the local runner's route CSV.

Flat launch/landing plane z_NED=0. This is a checked reference, not contact,
landing-gear, ground-effect or shutdown simulation evidence.
"""

import csv
import hashlib
import io
import math

from .export import FIELDS
from .flatness import attitude_duration_s
from .allocation import vehicle_rotor_duration_s
from .lifecycle_verification import verify_phase

PERIOD = 0.05
MAX_ROWS = 150_000
PEAKS = (2.1875, 7.513188404399301, 52.5)


def _evaluate(start, end, duration, t):
    u = t / duration
    q = 35 * u**4 - 84 * u**5 + 70 * u**6 - 20 * u**7
    d1 = (140 * u**3 - 420 * u**4 + 420 * u**5 - 140 * u**6) / duration
    d2 = (420 * u**2 - 1680 * u**3 + 2100 * u**4 - 840 * u**5) / duration**2
    d3 = (840 * u - 5040 * u**2 + 8400 * u**3 - 4200 * u**4) / duration**3
    if t == 0.0:
        q, d1, d2, d3 = 0.0, 0.0, 0.0, 0.0
    if t == duration:
        q, d1, d2, d3 = 1.0, 0.0, 0.0, 0.0
    delta = [b - a for a, b in zip(start, end, strict=True)]
    position = [a + d * q for a, d in zip(start, delta, strict=True)]
    row = dict(zip(("x", "y", "z", "yaw"), position, strict=True))
    for fields, derivative in (
        (("vx", "vy", "vz", "yaw_rate"), d1),
        (("ax", "ay", "az", "yaw_acceleration"), d2),
        (("jx", "jy", "jz", "yaw_jerk"), d3),
    ):
        row.update(zip(fields, [d * derivative for d in delta], strict=True))
    return row


def _phase(name, start, end, speed, vehicle, environment):
    distance, turn = math.dist(start[:3], end[:3]), abs(end[3] - start[3])
    a_num, j_num = distance * PEAKS[1], distance * PEAKS[2]
    duration = max(
        1.0,
        distance * PEAKS[0] / speed,
        math.sqrt(a_num / (vehicle.maximum_acceleration_m_s2 * 0.9)),
        (j_num / (vehicle.maximum_jerk_m_s3 * 0.9)) ** (1 / 3),
        turn * PEAKS[0] / math.radians(vehicle.maximum_yaw_rate_deg_s * 0.9),
        math.sqrt(
            turn
            * PEAKS[1]
            / math.radians(vehicle.maximum_yaw_acceleration_deg_s2 * 0.9)
        ),
        (turn * PEAKS[2] / math.radians(vehicle.maximum_yaw_jerk_deg_s3 * 0.9))
        ** (1 / 3),
    )
    attitude, _ = attitude_duration_s(
        acceleration_numerator=a_num,
        jerk_numerator=j_num,
        yaw_rate_numerator=turn * PEAKS[0],
        mass_kg=vehicle.mass_kg,
        tilt_limit_rad=math.radians(vehicle.maximum_tilt_deg * 0.9),
        minimum_thrust_n=vehicle.minimum_thrust_n / 0.9,
        maximum_thrust_n=vehicle.maximum_thrust_n * 0.9,
        maximum_body_rate_rad_s=math.radians(vehicle.maximum_body_rate_deg_s * 0.9),
        disturbance_acceleration_m_s2=getattr(
            vehicle, "disturbance_acceleration_m_s2", 0.0
        ),
        disturbance_jerk_m_s3=getattr(vehicle, "disturbance_jerk_m_s3", 0.0),
    )
    rotor, _ = vehicle_rotor_duration_s(vehicle, a_num, turn * PEAKS[1])
    duration = max(duration, attitude, rotor) * 1.02
    for _ in range(12):
        duration = math.ceil(duration / PERIOD) * PERIOD
        phase = dict(
            name=name, start=start, end=end, duration_s=duration, speed_limit_m_s=speed
        )
        report = verify_phase(phase, vehicle, environment)
        if report["accepted"]:
            phase["verification"] = report
            return phase
        duration *= 1.1
    raise ValueError("Full mission phase is infeasible: " + name)


def build_mission(route_csv, intent, vehicle, environment, profile=None):
    if environment.obstacle_map_verified:
        raise ValueError(
            "Full mission requires mapped takeoff/return/landing clearance; this composer supports the flat empty-world reference only"
        )
    route = list(csv.DictReader(route_csv.splitlines()))
    numeric = [{k: float(r[k]) for k in FIELDS} for r in route]
    if len(numeric) < 2:
        raise ValueError("Full mission requires a route with at least two samples")
    first, last = numeric[0], numeric[-1]
    if first["time"] != 0.0 or any(
        b["time"] <= a["time"] for a, b in zip(numeric, numeric[1:], strict=False)
    ):
        raise ValueError("Route timestamps must start at zero and increase strictly")
    if any(
        abs(row[k]) > 1e-8
        for row in (first, last)
        for k in (
            "vx",
            "vy",
            "vz",
            "ax",
            "ay",
            "az",
            "jx",
            "jy",
            "jz",
            "yaw_rate",
            "yaw_acceleration",
            "yaw_jerk",
        )
    ):
        raise ValueError(
            "Route boundaries must be stationary for continuous phase joins"
        )
    if min(-p.down_m for p in intent.path.points) <= intent.path.maximum_deviation_m:
        raise ValueError(
            "Route height must exceed the smoothing allowance above the flat ground plane"
        )
    if any(not math.isfinite(v) for row in numeric for v in row.values()):
        raise ValueError("Non-finite full mission input")
    if any(row["z"] > 1e-9 for row in numeric):
        raise ValueError("Route crosses below the landing plane")
    if math.hypot(first["x"], first["y"]) > 1e-8:
        raise ValueError(
            "Full mission must start above the launch origin (north=0, east=0)"
        )
    if first["z"] >= 0 or last["z"] >= 0:
        raise ValueError("Route must be above the flat launch/landing plane")

    def pose(row):
        return [row[k] for k in ("x", "y", "z", "yaw")]

    start, finish = pose(first), pose(last)
    ground = [0.0, 0.0, 0.0, start[3]]
    climb = (
        min(
            0.8,
            (profile or {}).get("maximum_climb_m_s", 0.8),
            vehicle.maximum_speed_m_s,
        )
        * 0.9
    )
    descent = (
        min(
            0.3,
            (profile or {}).get("maximum_descent_m_s", 0.3),
            vehicle.maximum_speed_m_s,
        )
        * 0.9
    )
    return_speed = (
        min(
            1.0,
            intent.motion.maximum_speed_m_s,
            intent.motion.target_speed_m_s or 1.0,
            vehicle.maximum_speed_m_s,
        )
        * 0.9
    )
    takeoff = _phase("TAKEOFF", ground, start, climb, vehicle, environment)
    phases, rows = [], []
    clock = 0.0

    def add_phase(phase):
        nonlocal clock
        count = math.ceil(phase["duration_s"] / PERIOD)
        if len(rows) + count + 1 > MAX_ROWS:
            raise ValueError("Full mission exceeds the sample budget")
        phase.update(start_time_s=clock, end_time_s=clock + phase["duration_s"])
        for i in range(count + 1):
            if rows and i == 0:
                continue
            t = phase["duration_s"] * i / count
            row = _evaluate(phase["start"], phase["end"], phase["duration_s"], t)
            row.update(
                time=clock + t,
                phase=phase["name"],
                height_m=0.0 if row["z"] == 0.0 else -row["z"],
                planner_contract_version=1,
            )
            rows.append(row)
        clock += phase["duration_s"]
        phases.append(phase)

    add_phase(takeoff)
    route_start = clock
    for i, row in enumerate(numeric):
        if i == 0:
            continue
        rows.append(
            dict(
                row,
                time=clock + row["time"],
                phase="ROUTE",
                height_m=0.0 if row["z"] == 0.0 else -row["z"],
            )
        )
    clock += last["time"]
    phases.append(
        dict(
            name="ROUTE",
            start_time_s=route_start,
            end_time_s=clock,
            duration_s=last["time"],
            verification_source="original independently checked route and SHA-256",
        )
    )
    if math.hypot(finish[0], finish[1]) > 1e-8:
        if intent.motion.yaw_mode != "fixed":
            heading = math.atan2(-finish[1], -finish[0])
            heading = finish[3] + math.atan2(
                math.sin(heading - finish[3]), math.cos(heading - finish[3])
            )
            if abs(heading - finish[3]) > 1e-8:
                aligned = finish[:3] + [heading]
                add_phase(
                    _phase(
                        "TURN_HOME", finish, aligned, return_speed, vehicle, environment
                    )
                )
                finish = aligned
        home = [0.0, 0.0, finish[2], finish[3]]
        add_phase(
            _phase("RETURN_HOME", finish, home, return_speed, vehicle, environment)
        )
        finish = home
    add_phase(
        _phase(
            "LAND", finish, [0.0, 0.0, 0.0, finish[3]], descent, vehicle, environment
        )
    )
    if len(rows) > MAX_ROWS:
        raise ValueError("Full mission exceeds the sample budget")
    if any(b["time"] <= a["time"] for a, b in zip(rows, rows[1:], strict=False)):
        raise ValueError("Full mission timestamps are not increasing")
    for row in (rows[0], rows[-1]):
        if any(
            abs(row[k]) > 1e-8
            for k in (
                "x",
                "y",
                "z",
                "vx",
                "vy",
                "vz",
                "ax",
                "ay",
                "az",
                "jx",
                "jy",
                "jz",
            )
        ):
            raise ValueError(
                "Full mission must start and finish stationary at the ground origin"
            )
    stream = io.StringIO()
    writer = csv.DictWriter(
        stream, fieldnames=(*FIELDS, "height_m", "phase"), lineterminator="\n"
    )
    writer.writeheader()
    writer.writerows(rows)
    csv_text = stream.getvalue()
    stride = max(1, math.ceil((len(rows) - 1) / 1000))
    indices = {0, len(rows) - 1, *range(0, len(rows), stride)}
    for phase in phases:
        for t in (phase["start_time_s"], phase["end_time_s"]):
            indices.add(min(range(len(rows)), key=lambda i: abs(rows[i]["time"] - t)))
    return dict(
        schema_version=1,
        accepted=True,
        release_class="SITL_ONLY",
        execution_ready=False,
        simulation_status="NOT_RUN",
        ground_plane_z_ned_m=0.0,
        surface_assumption="flat launch plane; no terrain/contact/ground-effect model",
        total_duration_s=clock,
        route_duration_s=last["time"],
        phases=phases,
        controller_route_sha256=hashlib.sha256(route_csv.encode()).hexdigest(),
        trajectory_sha256=hashlib.sha256(csv_text.encode()).hexdigest(),
        verification=dict(
            accepted=True,
            phase_bounds="independent continuous Bernstein bounds",
            route="original independent route certificate",
            stationary_ground_endpoints=True,
            strictly_increasing_time=True,
            actual_landing_or_shutdown_proven=False,
        ),
        samples=[rows[i] for i in sorted(indices)],
        sample_count=len(rows),
        trajectory_csv=csv_text,
    )
