"""Native baseline corpus and analytic timing auditor; never starts a simulator.

The 1-D jerk-limited reference is a relaxation using endpoint displacement.
For curved routes it is not the optimal time of the original constrained path.
No TOPP/COPP package or browser compatibility is claimed by this audit.
"""

from copy import deepcopy
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from mission_planner.preview import preview  # noqa: E402


def scurve_time(distance, speed, acceleration, jerk):
    if distance <= 0.0:
        return 0.0
    tj = min(acceleration / jerk, math.sqrt(speed / jerk))
    ta = max(0.0, speed / acceleration - tj)
    ramp = 2 * tj + ta
    if distance >= speed * ramp:
        return 2 * ramp + (distance - speed * ramp) / speed
    if distance >= 2 * acceleration**3 / jerk**2:
        tj = acceleration / jerk
        ta = (-3 * tj + math.sqrt(tj * tj + 4 * distance / acceleration)) / 2
        return 2 * (2 * tj + ta)
    return 4 * (distance / (2 * jerk)) ** (1 / 3)


def corpus():
    template = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
    fixtures = {
        "straight20": ([(0, 0), (20, 0)], ()),
        "rounded_L": ([(0, 0), (15, 0), (15, 12)], ()),
        "stopped_L": ([(0, 0), (15, 0), (15, 12)], (1,)),
        "zigzag": ([(0, 0), (4, 2), (8, -2), (12, 2), (16, 0)], ()),
        "figure_eight": (
            [
                (0, 0),
                (6, 6),
                (0, 12),
                (-6, 6),
                (0, 0),
                (6, -6),
                (0, -12),
                (-6, -6),
                (0, 0),
            ],
            (),
        ),
        "lawnmower": ([(0, 0), (12, 0), (12, 3), (0, 3), (0, 6), (12, 6)], ()),
        "repeated_visits": ([(0, 0), (6, 0), (0, 0), (10, 0)], ()),
        "many_stops": ([(0, 0), (4, 2), (8, -2), (12, 2), (16, 0)], (1, 2, 3)),
        "near_boundary": ([(0, 0), (39, 0), (35, 3)], ()),
        "tiny_feature": ([(0, 0), (5, 0), (5.05, 0.05), (5.1, 0), (10, 0)], ()),
    }
    for name, (points, stops) in fixtures.items():
        raw = deepcopy(template)
        raw["path"].update(
            maximum_deviation_m=0.5,
            closure="open_stop",
            points=[
                dict(
                    point_id=f"p{i}",
                    north_m=n,
                    east_m=e,
                    down_m=-20.0,
                    hard=i in stops,
                    exact=False,
                )
                for i, (n, e) in enumerate(points)
            ],
        )
        raw["motion"].update(
            yaw_mode="fixed",
            fixed_yaw_deg=0.0,
            target_speed_m_s=1.0,
            maximum_speed_m_s=1.0,
        )
        yield name, dict(source="drawing", document=raw), True
    base = dict(
        schema_version=1,
        mission_id="audit-circle",
        vehicle_model_id="px4-x500-sitl-v1",
        environment_id="empty-world-v1",
        pattern="circle",
        altitude_m=20.0,
        maximum_deviation_m=0.5,
        target_speed_m_s=1.0,
        maximum_speed_m_s=1.0,
        yaw_mode="fixed",
        fixed_yaw_deg=0.0,
        time_mode="minimize",
        requested_duration_s=None,
        circle=dict(center_north_m=8.0, center_east_m=0.0),
    )
    for laps in (1, 2):
        yield (
            f"circle_{laps}_laps",
            dict(source="interview", document=dict(base, loop_count=laps)),
            True,
        )
    oversized = deepcopy(template)
    oversized["path"]["points"] = [
        dict(oversized["path"]["points"][0], point_id=f"p{i}", north_m=float(i))
        for i in range(257)
    ]
    yield "oversized", dict(source="drawing", document=oversized), False
    yield "invalid", dict(source="drawing", document={}), False


def run(destination):
    report = dict(
        schema_version=1,
        baseline=dict(
            speed_cap_m_s=0.9,
            acceleration_cap_m_s2=0.9,
            jerk_cap_m_s3=1.35,
            height_m=20.0,
            radius_m=40.0,
            tilt_deg=25.0,
            soft_target_fraction=0.25,
        ),
        scope="native deterministic planning only; no Gazebo; no cross-runtime equality claim",
        cases=[],
    )
    destination.parent.mkdir(parents=True, exist_ok=True)
    for name, payload, expected in corpus():
        start = time.perf_counter()
        item = dict(name=name, expected_accept=expected)
        try:
            result = preview(payload)
            rt = result["summary"]["retiming"]
            geometry = result["summary"]["smoothing"]
            geometry = geometry if isinstance(geometry, list) else [geometry]
            length = sum(g["curve_length_m"] for g in geometry) * rt["loop_count"]
            points = result["intent"]["path"]["points"]
            distance = math.dist(
                tuple(points[0][k] for k in ("north_m", "east_m", "down_m")),
                tuple(points[-1][k] for k in ("north_m", "east_m", "down_m")),
            )
            lower = max(
                length / rt["speed_limit_m_s"],
                scurve_time(
                    distance,
                    rt["speed_limit_m_s"],
                    rt["acceleration_limit_m_s2"],
                    rt["jerk_limit_m_s3"],
                ),
            )
            item.update(
                accepted=True,
                route_duration_s=rt["selected_duration_s"],
                full_duration_s=result["mission"]["total_duration_s"],
                timing_relaxation_lower_bound_s=lower,
                duration_above_relaxation_fraction=rt["selected_duration_s"] / lower - 1
                if lower
                else None,
                binding_constraint=rt["limiting_constraint"],
                fidelity_policy=result["summary"]["fidelity_policy"],
                checks=result["verification"]["checks"],
                note="gap to a relaxation; not measured suboptimality against a matched path solver",
            )
        except ValueError as error:
            item.update(accepted=False, error=str(error))
        item.update(compute_s=time.perf_counter() - start)
        item["expected_outcome_matched"] = item["accepted"] == expected
        report["cases"].append(item)
        destination.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        print(
            json.dumps(
                {
                    k: item[k]
                    for k in (
                        "name",
                        "accepted",
                        "expected_outcome_matched",
                        "compute_s",
                    )
                }
            ),
            flush=True,
        )
    report["passed"] = all(c["expected_outcome_matched"] for c in report["cases"])
    destination.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    return report["passed"]


if __name__ == "__main__":
    raise SystemExit(0 if run(Path(sys.argv[1])) else 1)
