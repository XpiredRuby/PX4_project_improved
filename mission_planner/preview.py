"""Portable plan preview shared by the local server and browser worker."""
from __future__ import annotations

from copy import deepcopy
import csv
import json
import math
from pathlib import Path
import tempfile

from .contracts import EnvironmentSnapshot, MissionIntent, VehicleModel, load_contract, canonical_sha256
from .plan_drawn_path import compile_drawn_path
from .plan_interview import intent_from_interview
from .aircraft import assess_profile, mission_budget

ROOT = Path(__file__).resolve().parents[1]
MAX_RESPONSE_BYTES = 24 * 1024 * 1024
MAX_INPUT_POINTS = 256


def preview(payload: dict) -> dict:
    """Compile one explicit document without importing the flight runner."""
    if (not isinstance(payload, dict) or not {"source", "document"} <= set(payload)
            or set(payload) - {"source", "document", "aircraft_profile"}):
        raise ValueError("preview requires source, document and optionally aircraft_profile")
    source = payload["source"]
    raw = payload["document"]
    if source == "drawing":
        intent = deepcopy(raw)
        loops = 1
    elif source == "interview":
        intent, loops = intent_from_interview(raw)
    else:
        raise ValueError("source must be drawing or interview")
    setup, error_budget = None, None
    if "aircraft_profile" in payload:
        setup = assess_profile(payload["aircraft_profile"])
        if not setup["ready"]:
            return {"accepted": False, "release_class": "SITL_ONLY", "campaign_ran": False,
                    "error": "Complete the aircraft setup questions and resolve compatibility errors.",
                    "aircraft_setup": setup}
        profile = setup["profile"]
        total = _intent_deviation(intent)
        geometric = total - profile["position_uncertainty_m"] - profile["tracking_error_m"]
        if geometric <= 0:
            raise ValueError("path tolerance is consumed by the position and tracking allowances; increase tolerance or improve navigation/tracking")
        error_budget = dict(total_tolerance_m=total, planning_tolerance_m=geometric,
                            position_uncertainty_m=profile["position_uncertainty_m"],
                            tracking_error_m=profile["tracking_error_m"])
        intent["path"]["maximum_deviation_m"] = geometric
        if any(p["down_m"] != intent["path"]["points"][0]["down_m"] for p in intent["path"]["points"]):
            cap = min(profile["maximum_climb_m_s"], profile["maximum_descent_m_s"])
            intent["motion"]["maximum_speed_m_s"] = min(intent["motion"]["maximum_speed_m_s"], cap)
            intent["motion"]["target_speed_m_s"] = min(intent["motion"]["target_speed_m_s"], cap)
            error_budget["vertical_route_speed_cap_m_s"] = cap
    contract = MissionIntent.from_dict(intent)
    if len(contract.path.points) > MAX_INPUT_POINTS:
        raise ValueError(f"preview accepts at most {MAX_INPUT_POINTS} input points")
    altitude = -contract.path.points[0].down_m
    if not math.isfinite(altitude) or altitude <= 0.:
        raise ValueError("preview requires a positive launch-relative cruise height")
    with tempfile.TemporaryDirectory(prefix="px4-preview-") as directory:
        work = Path(directory)
        vehicle_path = ROOT / "config/planner/x500_sitl_vehicle_model.json"
        environment_path = ROOT / "config/planner/empty_world_environment.json"
        if setup is not None:
            vehicle_path, environment_path = work / "vehicle.json", work / "environment.json"
            vehicle_path.write_text(json.dumps(setup["vehicle_model"], allow_nan=False))
            environment = load_contract(ROOT / "config/planner/empty_world_environment.json", dict)
            environment.update(environment_id="profile-env-" + setup["profile_sha256"][:16],
                               max_radius_m=profile["max_radius_m"],
                               max_height_above_launch_m=profile["max_height_m"],
                               max_wind_m_s=profile["expected_wind_m_s"])
            environment_path.write_text(json.dumps(environment, allow_nan=False))
        intent_path = work / "intent.json"
        intent_path.write_text(json.dumps(intent, allow_nan=False), encoding="utf-8")
        trajectory = work / "trajectory.csv"
        report = work / "verification.json"
        summary = compile_drawn_path(
            intent_path=intent_path,
            vehicle_path=vehicle_path,
            environment_path=environment_path,
            trajectory_path=trajectory,
            report_path=report,
            summary_path=work / "summary.json",
            control_spacing_m=.5,
            verification_resolution_m=.02,
            sample_period_s=.05,
            limit_scale=.9,
            controller_cruise_height_m=altitude,
            deviation_utilization=.75,
            loop_count=loops,
        )
        if trajectory.stat().st_size > MAX_RESPONSE_BYTES // 2:
            raise ValueError("trajectory exceeds the preview download budget; use the CLI")
        csv_text = trajectory.read_text(encoding="utf-8")
        rows = list(csv.DictReader(csv_text.splitlines()))
        budget = None
        if setup is not None:
            if any(abs(float(r["vz"])) > min(profile["maximum_climb_m_s"], profile["maximum_descent_m_s"]) + 1e-8 for r in rows):
                raise ValueError("vertical route speed exceeds the aircraft climb/descent envelope")
            budget = mission_budget(setup, summary["retiming"]["selected_duration_s"],
                                    altitude, -float(rows[-1]["z"]))
            summary.update(aircraft_profile_sha256=setup["profile_sha256"],
                           mission_budget_sha256=canonical_sha256(budget),
                           path_error_budget_sha256=canonical_sha256(error_budget))
        stride = max(1, math.ceil((len(rows) - 1) / 1000))
        indices = sorted({*range(0, len(rows), stride), len(rows) - 1})
        return {
            "accepted": True,
            "release_class": "SITL_ONLY",
            "campaign_ran": False,
            "simulation_status": "NOT_RUN",
            "flight_approval": False,
            "intent": contract.to_dict(),
            "vehicle_model": load_contract(vehicle_path, VehicleModel.from_dict).to_dict(),
            "environment": load_contract(environment_path, EnvironmentSnapshot.from_dict).to_dict(),
            "aircraft_setup": setup,
            "mission_budget": budget,
            "path_error_budget": error_budget,
            "simulation_compatible": setup is None,
            "summary": summary,
            "verification": json.loads(report.read_text(encoding="utf-8")),
            "samples": [
                {key: float(rows[i][key]) for key in
                 ("time", "x", "y", "z", "yaw", "vx", "vy", "vz", "ax", "ay", "az")}
                for i in indices
            ],
            "display_samples_decimated": stride > 1,
            "trajectory_csv": csv_text,
            "preview_settings": {
                "control_spacing_m": .5, "verification_resolution_m": .02,
                "sample_period_s": .05, "limit_scale": .9,
                "deviation_utilization": .75,
            },
        }


def _intent_deviation(intent):
    # Parse before arithmetic so strings, bools and non-finite input are refused.
    return MissionIntent.from_dict(intent).path.maximum_deviation_m


