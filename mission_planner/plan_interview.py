"""Turn a structured interview into the same verified trajectory as a drawing.

Roll and pitch are not questions. They are whatever attitude the acceleration
requires, and the verifier checks them against tilt, thrust, and body rate.
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
        "look_at_north_m",
        "look_at_east_m",
        "look_at_down_m",
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
    if yaw_mode not in {"fixed", "tangent", "look_at"}:
        raise PlanningError(
            "choose fixed, tangent, or look-at yaw; roll and pitch are not mission inputs",
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
