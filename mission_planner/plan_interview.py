"""Turn a structured interview into the same verified trajectory as a drawing.

Roll and pitch are not questions. They are whatever attitude the acceleration
requires, and the verifier checks them against tilt, thrust, and body rate.
A circle may name an entry, an exit, or both. The lead-in leaves launch and
meets the circle without a stop. The laps become one open path that flies each
launch seam through and stops at the end of the exit, or at the seam when
there is no exit. Yaw may be an explicit heading profile on that one piece.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .contracts import OBJECTIVE_PRIORITY, EnvironmentSnapshot, load_contract
from .explain import PlanningError
from .mapping import OccupancyMap, corridor_polyline
from .plan_drawn_path import compile_drawn_path


def _object(value, field: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{field} must be an object")
    return value


def _number(value, field: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    if minimum is not None and result < minimum:
        raise ValueError(f"{field} must be at least {minimum}")
    return result


def _circle_points(center_north: float, center_east: float, altitude: float, deviation: float):
    radius = math.hypot(center_north, center_east)
    if radius < 1.0:
        raise PlanningError(
            "circle radius must be at least 1 m so the turn stays flyable",
            limiting_constraint="geometry",
        )
    allowance = min(0.25, deviation / (4.0 * radius))
    half = math.acos(max(-1.0, min(1.0, 1.0 - allowance)))
    count = max(12, math.ceil(math.tau / max(2.0 * half, 1e-3)))
    theta0 = math.atan2(-center_east, -center_north)
    points = []
    for index in range(count):
        theta = theta0 + math.tau * index / count
        points.append({
            "point_id": f"circle-{index}",
            "north_m": center_north + radius * math.cos(theta),
            "east_m": center_east + radius * math.sin(theta),
            "down_m": -altitude,
            "hard": False,
            "exact": False,
        })
    points.append(dict(points[0], point_id="circle-close"))
    points[0]["north_m"] = 0.0
    points[0]["east_m"] = 0.0
    points[-1]["north_m"] = 0.0
    points[-1]["east_m"] = 0.0
    return points


def _leg_points(raw_leg, altitude: float, leg: str):
    field = f"interview.{leg}"
    if not isinstance(raw_leg, list) or len(raw_leg) < 1:
        raise ValueError(f"{field} must contain at least one point")
    points = []
    for index, raw in enumerate(raw_leg):
        value = _object(raw, f"{field}[{index}]")
        unknown = set(value) - {"north_m", "east_m", "hard", "exact"}
        if unknown:
            raise ValueError(
                f"{field}[{index}] has unknown fields: " + ", ".join(sorted(unknown))
            )
        if "north_m" not in value or "east_m" not in value:
            raise ValueError(f"{field}[{index}] requires north_m and east_m")
        hard = value.get("hard", False)
        exact = value.get("exact", False)
        if not isinstance(hard, bool) or not isinstance(exact, bool):
            raise ValueError(f"{field}[{index}] hard and exact must be boolean")
        if hard and exact:
            raise ValueError(
                f"{field}[{index}] cannot both stop and be flown through"
            )
        points.append({
            "point_id": f"{leg}-{index}",
            "north_m": _number(value["north_m"], f"{field}[{index}].north_m"),
            "east_m": _number(value["east_m"], f"{field}[{index}].east_m"),
            "down_m": -altitude,
            "hard": hard,
            "exact": exact,
        })
    return points


def _unrolled_laps(closed, laps: int):
    """Repeat the ring so two laps of A B C become A B C A B C A.

    Every return to the launch point is an exact fly-through, including the
    final one. A later exit makes that final seam interior. With no exit the
    final seam is the stop, and an exact mark on an endpoint does not change
    the rest-to-rest time law.
    """
    ring = closed[:-1]
    chain = []
    for lap in range(laps):
        for point in (ring if lap == 0 else ring[1:]):
            chain.append({
                **point,
                "point_id": f"{point['point_id']}-lap-{lap}",
                "hard": False,
                "exact": False,
            })
        chain.append({
            "point_id": f"circle-seam-{lap}",
            "north_m": 0.0,
            "east_m": 0.0,
            "down_m": ring[0]["down_m"],
            "hard": False,
            "exact": True,
        })
    return chain


def _append_exit(chain, exit_points):
    first = exit_points[0]
    if math.hypot(first["north_m"], first["east_m"]) <= 1e-6:
        raise PlanningError(
            "the exit must leave the circle seam",
            limiting_constraint="geometry",
        )
    previous = chain[-1]
    for point in exit_points:
        if math.hypot(
            point["north_m"] - previous["north_m"],
            point["east_m"] - previous["east_m"],
        ) <= 1e-6:
            raise PlanningError(
                "the exit repeats the previous point",
                limiting_constraint="geometry",
            )
        chain.append(point)
        previous = point
    return chain


def _prepend_entry(chain, entry_points):
    """Fly from launch, through the lead-in, and onto the circle seam."""
    if math.hypot(entry_points[0]["north_m"], entry_points[0]["east_m"]) <= 1e-6:
        raise PlanningError(
            "the entry must leave the launch point",
            limiting_constraint="geometry",
        )
    launch = {
        "point_id": "circle-launch",
        "north_m": 0.0,
        "east_m": 0.0,
        "down_m": chain[0]["down_m"],
        "hard": False,
        "exact": False,
    }
    seam = dict(chain[0])
    seam["hard"] = False
    seam["exact"] = True
    previous = launch
    for point in entry_points:
        if math.hypot(
            point["north_m"] - previous["north_m"],
            point["east_m"] - previous["east_m"],
        ) <= 1e-6:
            raise PlanningError(
                "the entry repeats the previous point",
                limiting_constraint="geometry",
            )
        previous = point
    if math.hypot(previous["north_m"], previous["east_m"]) <= 1e-6:
        raise PlanningError(
            "the entry must reach the circle from off the seam",
            limiting_constraint="geometry",
        )
    return [launch, *entry_points, seam, *chain[1:]]


def _survey_points(length: float, width: float, spacing: float, altitude: float):
    if spacing <= 0.0 or length <= 0.0 or width < 0.0:
        raise PlanningError(
            "survey length and lane spacing must be positive",
            limiting_constraint="geometry",
        )
    lanes = max(1, math.floor(width / spacing) + 1)
    points = []
    for lane in range(lanes):
        east = min(width, lane * spacing)
        forward = lane % 2 == 0
        ends = (0.0, length) if forward else (length, 0.0)
        if points:
            points.append({
                "point_id": f"survey-{lane}-join",
                "north_m": ends[0],
                "east_m": east,
                "down_m": -altitude,
                "hard": False,
                "exact": False,
            })
        points.append({
            "point_id": f"survey-{lane}-end",
            "north_m": ends[1],
            "east_m": east,
            "down_m": -altitude,
            "hard": False,
            "exact": False,
        })
    if not points or points[0]["north_m"] != 0.0 or points[0]["east_m"] != 0.0:
        points.insert(0, {
            "point_id": "survey-start",
            "north_m": 0.0,
            "east_m": 0.0,
            "down_m": -altitude,
            "hard": True,
            "exact": False,
        })
    return points


def _waypoint_points(raw_points, altitude: float):
    if not isinstance(raw_points, list) or len(raw_points) < 2:
        raise ValueError("waypoints must contain at least two points")
    points = []
    for index, raw in enumerate(raw_points):
        value = _object(raw, f"waypoints[{index}]")
        allowed = {"north_m", "east_m", "hard", "exact"}
        if set(value) - allowed:
            raise ValueError(f"waypoints[{index}] has unknown fields")
        points.append({
            "point_id": f"waypoint-{index}",
            "north_m": _number(value["north_m"], f"waypoints[{index}].north_m"),
            "east_m": _number(value["east_m"], f"waypoints[{index}].east_m"),
            "down_m": -altitude,
            "hard": bool(value.get("hard", False)),
            "exact": bool(value.get("exact", False)),
        })
    if abs(points[0]["north_m"]) > 1e-9 or abs(points[0]["east_m"]) > 1e-9:
        raise PlanningError(
            "the route must start at the launch point (0, 0)",
            limiting_constraint="geometry",
        )
    return points


def _corridor_points(occupancy, goal_north, goal_east, altitude, deviation, clearance):
    polyline = corridor_polyline(
        occupancy,
        start=(0.0, 0.0),
        goal=(goal_north, goal_east),
        clearance_m=clearance,
        deviation_m=deviation,
    )
    return [
        {
            "point_id": f"corridor-{index}",
            "north_m": north,
            "east_m": east,
            "down_m": -altitude,
            "hard": False,
            "exact": False,
        }
        for index, (north, east) in enumerate(polyline)
    ]


def intent_from_interview(
    raw,
    *,
    occupancy: OccupancyMap | None = None,
    clearance_m: float = 0.0,
) -> tuple[dict, int]:
    """Return a MissionIntent document and the number of laps to fly."""
    value = _object(raw, "interview")
    allowed = {
        "schema_version",
        "mission_id",
        "pattern",
        "altitude_m",
        "maximum_deviation_m",
        "target_speed_m_s",
        "maximum_speed_m_s",
        "yaw_mode",
        "fixed_yaw_deg",
        "time_mode",
        "requested_duration_s",
        "vehicle_model_id",
        "environment_id",
        "loop_count",
        "circle",
        "survey",
        "waypoints",
        "corridor",
        "entry",
        "exit",
        "look_at_north_m",
        "look_at_east_m",
        "look_at_down_m",
        "yaw_profile",
    }
    unknown = set(value) - allowed
    if unknown:
        raise ValueError("interview has unknown fields: " + ", ".join(sorted(unknown)))
    for field in (
        "schema_version",
        "mission_id",
        "pattern",
        "altitude_m",
        "maximum_deviation_m",
        "maximum_speed_m_s",
        "yaw_mode",
        "time_mode",
        "vehicle_model_id",
        "environment_id",
    ):
        if field not in value:
            raise ValueError(f"interview.{field} is required")
    if value["schema_version"] != 1:
        raise ValueError("interview.schema_version must be 1")
    pattern = value["pattern"]
    if pattern not in {"waypoints", "circle", "survey", "corridor"}:
        raise ValueError(
            "interview.pattern must be waypoints, circle, survey, or corridor"
        )
    if pattern != "circle" and ("entry" in value or "exit" in value):
        raise PlanningError(
            "an entry or exit leg is only valid on a circle",
            limiting_constraint="geometry",
        )
    altitude = _number(value["altitude_m"], "interview.altitude_m", minimum=0.1)
    deviation = _number(
        value["maximum_deviation_m"], "interview.maximum_deviation_m", minimum=0.0
    )
    loop_count = int(value.get("loop_count", 1))
    if loop_count < 1 or loop_count > 20:
        raise ValueError("interview.loop_count must be in [1, 20]")
    if pattern == "circle":
        circle = _object(value["circle"], "interview.circle")
        points = _circle_points(
            _number(circle["center_north_m"], "interview.circle.center_north_m"),
            _number(circle["center_east_m"], "interview.circle.center_east_m"),
            altitude,
            deviation,
        )
        if "exit" in value or "entry" in value:
            points = _unrolled_laps(points, loop_count)
            if "exit" in value:
                points = _append_exit(
                    points, _leg_points(value["exit"], altitude, "exit")
                )
            if "entry" in value:
                points = _prepend_entry(
                    points, _leg_points(value["entry"], altitude, "entry")
                )
            closure = "open_stop"
            loop_count = 1
        else:
            closure = "periodic"
    elif pattern == "survey":
        survey = _object(value["survey"], "interview.survey")
        points = _survey_points(
            _number(survey["length_m"], "interview.survey.length_m", minimum=0.1),
            _number(survey["width_m"], "interview.survey.width_m", minimum=0.0),
            _number(survey["lane_spacing_m"], "interview.survey.lane_spacing_m", minimum=0.1),
            altitude,
        )
        closure = "open_stop"
        loop_count = 1
    elif pattern == "waypoints":
        points = _waypoint_points(value["waypoints"], altitude)
        closure = "open_stop"
        loop_count = 1
    else:
        if occupancy is None:
            raise PlanningError(
                "a corridor interview requires an occupancy map",
                limiting_constraint="map",
            )
        corridor = _object(value["corridor"], "interview.corridor")
        points = _corridor_points(
            occupancy,
            _number(corridor["goal_north_m"], "interview.corridor.goal_north_m"),
            _number(corridor["goal_east_m"], "interview.corridor.goal_east_m"),
            altitude,
            deviation,
            clearance_m,
        )
        closure = "open_stop"
        loop_count = 1
    yaw_mode = value["yaw_mode"]
    if yaw_mode not in {"fixed", "tangent", "look_at", "profile"}:
        raise PlanningError(
            "choose fixed, tangent, look-at, or an explicit profile; "
            "roll and pitch are not mission inputs",
            limiting_constraint="yaw",
        )
    profile = value.get("yaw_profile")
    if yaw_mode == "profile" and profile is None:
        raise PlanningError(
            "an explicit yaw profile needs heading samples",
            limiting_constraint="yaw",
        )
    if yaw_mode != "profile" and profile is not None:
        raise PlanningError(
            "yaw profile samples require profile yaw",
            limiting_constraint="yaw",
        )
    look_at = (
        value.get("look_at_north_m"),
        value.get("look_at_east_m"),
        value.get("look_at_down_m"),
    )
    intent = {
        "schema_version": 1,
        "mission_id": value["mission_id"],
        "release_class": "SITL_ONLY",
        "frame": {
            "name": "LOCAL_NED",
            "units": "m",
            "origin_id": "launch-001",
            "altitude_datum": "launch",
        },
        "path": {
            "kind": "polyline",
            "closure": closure,
            "maximum_deviation_m": deviation,
            "points": points,
        },
        "motion": {
            "target_speed_m_s": value.get("target_speed_m_s"),
            "maximum_speed_m_s": _number(
                value["maximum_speed_m_s"],
                "interview.maximum_speed_m_s",
                minimum=0.0,
            ),
            "yaw_mode": yaw_mode,
            "fixed_yaw_deg": value.get("fixed_yaw_deg"),
            "look_at_north_m": None if yaw_mode != "look_at" else look_at[0],
            "look_at_east_m": None if yaw_mode != "look_at" else look_at[1],
            "look_at_down_m": None if yaw_mode != "look_at" else look_at[2],
            "yaw_profile": None if yaw_mode != "profile" else profile,
            "time_mode": value["time_mode"],
            "requested_duration_s": value.get("requested_duration_s"),
        },
        "vehicle_model_id": value["vehicle_model_id"],
        "environment_id": value["environment_id"],
        "objective_priority": list(OBJECTIVE_PRIORITY),
    }
    return intent, loop_count


def compile_interview(
    *,
    interview_path: Path,
    intent_path: Path,
    vehicle_path: Path,
    environment_path: Path,
    trajectory_path: Path,
    report_path: Path,
    summary_path: Path,
    review_path: Path | None = None,
    control_spacing_m: float = 0.5,
    verification_resolution_m: float = 0.02,
    sample_period_s: float = 0.05,
    limit_scale: float = 0.9,
    controller_cruise_height_m: float = 17.0,
    deviation_utilization: float = 0.75,
    obstacle_map_path: Path | None = None,
) -> dict:
    raw = json.loads(interview_path.read_text(encoding="utf-8"))
    environment = load_contract(environment_path, EnvironmentSnapshot.from_dict)
    occupancy = None
    if obstacle_map_path is not None:
        occupancy = load_contract(obstacle_map_path, OccupancyMap.from_dict)
    intent, loop_count = intent_from_interview(
        raw,
        occupancy=occupancy,
        clearance_m=environment.minimum_clearance_m,
    )
    intent_path.parent.mkdir(parents=True, exist_ok=True)
    intent_path.write_text(
        json.dumps(intent, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    summary = compile_drawn_path(
        intent_path=intent_path,
        vehicle_path=vehicle_path,
        environment_path=environment_path,
        trajectory_path=trajectory_path,
        report_path=report_path,
        summary_path=summary_path,
        control_spacing_m=control_spacing_m,
        verification_resolution_m=verification_resolution_m,
        sample_period_s=sample_period_s,
        limit_scale=limit_scale,
        controller_cruise_height_m=controller_cruise_height_m,
        deviation_utilization=deviation_utilization,
        loop_count=loop_count,
        review_path=review_path,
        obstacle_map_path=obstacle_map_path,
    )
    summary["interview_pattern"] = raw["pattern"]
    summary["interview_loop_count"] = loop_count
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return summary


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interview", required=True, type=Path)
    parser.add_argument("--intent", required=True, type=Path)
    parser.add_argument("--vehicle", required=True, type=Path)
    parser.add_argument("--environment", required=True, type=Path)
    parser.add_argument("--trajectory", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--summary", required=True, type=Path)
    parser.add_argument("--review", type=Path)
    parser.add_argument("--control-spacing-m", type=float, default=0.5)
    parser.add_argument("--verification-resolution-m", type=float, default=0.02)
    parser.add_argument("--sample-period-s", type=float, default=0.05)
    parser.add_argument("--limit-scale", type=float, default=0.9)
    parser.add_argument("--controller-cruise-height-m", type=float, default=17.0)
    parser.add_argument("--deviation-utilization", type=float, default=0.75)
    parser.add_argument("--obstacle-map", type=Path)
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    try:
        summary = compile_interview(
            interview_path=args.interview,
            intent_path=args.intent,
            vehicle_path=args.vehicle,
            environment_path=args.environment,
            trajectory_path=args.trajectory,
            report_path=args.report,
            summary_path=args.summary,
            review_path=args.review,
            control_spacing_m=args.control_spacing_m,
            verification_resolution_m=args.verification_resolution_m,
            sample_period_s=args.sample_period_s,
            limit_scale=args.limit_scale,
            controller_cruise_height_m=args.controller_cruise_height_m,
            deviation_utilization=args.deviation_utilization,
            obstacle_map_path=args.obstacle_map,
        )
    except PlanningError as error:
        print(json.dumps(error.to_dict(), indent=2, sort_keys=True))
        raise SystemExit(2) from error
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()
