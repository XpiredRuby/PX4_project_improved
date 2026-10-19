"""Reusable aircraft onboarding, compatibility checks and mission budgets.

No aircraft is identified from its brand name or mass alone. Inputs describe
the loaded configuration and remain user declarations, never flight approval.
The question catalog is shared by the CLI, HTTP API and browser form.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .allocation import Rotor, allocation_margin
from .contracts import (VehicleModel, canonical_sha256, _identifier, _number,
                        _text, load_contract)
from .flatness import GRAVITY_M_S2


def question(key, label, kind="number", unit="", help_text="", **options):
    return dict(key=key, label=label, kind=kind, unit=unit, help=help_text, **options)


QUESTIONS = [
    question("profile_id", "Aircraft configuration ID", "text", help_text="Use a new ID when payload, battery, propellers or tuning changes."),
    question("name", "Aircraft/configuration name", "text"),
    question("vehicle_type", "Aircraft type", "choice", choices=["multicopter", "fixed_wing", "vtol", "helicopter"]),
    question("autopilot", "Flight stack", "choice", choices=["px4", "ardupilot", "other"]),
    question("mass_kg", "Loaded takeoff mass", unit="kg", help_text="Include battery, payload and every rotor. Use the upper bound of measurement uncertainty."),
    question("layout", "Rotor layout", "choice", choices=["quad_x", "hex_x", "octo_x", "custom"]),
    question("arm_length_m", "Center of mass to rotor distance", unit="m", help_text="Regular layouts only; use custom rotors for offsets, coaxial or asymmetric geometry."),
    question("rotor_minimum_thrust_n", "Per-rotor minimum thrust", unit="N", help_text="Use an upper bound on the minimum controllable force.", minimum=0),
    question("rotor_maximum_thrust_n", "Per-rotor available thrust", unit="N", help_text="Use a lower bound at the worst allowed battery state, temperature and density."),
    question("yaw_torque_per_thrust_m", "Rotor yaw torque / thrust", unit="m", help_text="Positive magnitude for regular alternating-spin rotors; obtain from motor/propeller data or identification."),
    question("yaw_inertia_kg_m2", "Loaded yaw inertia", unit="kg m²", help_text="Upper bound about the loaded center of mass, from CAD or identification."),
    question("roll_pitch_torque_reserve_nm", "Simultaneous roll/pitch torque reserve", unit="N m", help_text="Required on each axis; must cover actual demands within the declared flight envelope. This does not identify those demands."),
    question("maximum_speed_m_s", "Validated speed ceiling", unit="m/s", help_text="Use the lowest applicable vehicle, controller and operating limit."),
    question("maximum_acceleration_m_s2", "Validated acceleration ceiling", unit="m/s²"),
    question("maximum_jerk_m_s3", "Validated jerk ceiling", unit="m/s³"),
    question("maximum_tilt_deg", "Validated tilt ceiling", unit="deg", maximum=80),
    question("maximum_body_rate_deg_s", "Validated body-rate ceiling", unit="deg/s"),
    question("maximum_yaw_rate_deg_s", "Validated yaw-rate ceiling", unit="deg/s"),
    question("maximum_yaw_acceleration_deg_s2", "Validated yaw-acceleration ceiling", unit="deg/s²"),
    question("maximum_yaw_jerk_deg_s3", "Validated yaw-jerk ceiling", unit="deg/s³"),
    question("maximum_climb_m_s", "Conservative takeoff climb rate", unit="m/s", help_text="Used for budgeting; must not exceed the validated vertical speed."),
    question("maximum_descent_m_s", "Conservative landing descent rate", unit="m/s", help_text="Include slow landing behavior in this effective rate."),
    question("usable_battery_wh", "Battery energy available at launch", unit="Wh", help_text="Derated usable energy at current charge, payload and conditions, before the reserve below."),
    question("power_upper_bound_w", "Conservative power upper bound", unit="W", help_text="Measured/justified bound for the entire declared speed, wind, payload and temperature envelope. Hover power alone is insufficient."),
    question("reserve_fraction", "Battery reserve fraction", help_text="0.25 means retain 25% of the declared available energy.", minimum=0.1, maximum=0.8),
    question("settle_time_s", "Takeoff/landing settling allowance", unit="s", minimum=0),
    question("lifecycle_time_upper_bound_s", "Takeoff, return and landing time upper bound", unit="s", help_text="A justified upper bound including heading initialization, climb, transit/return, settling and complete landing. Maximum speeds alone cannot establish this bound."),
    question("route_time_multiplier", "Route wall-time allowance / planned duration", minimum=1, help_text="Include tracking-governor slowing and pauses; use a justified upper bound, not the ideal mission clock."),
    question("contingency_time_s", "Diversion/failsafe flight allowance", unit="s", help_text="Budget for the selected abort route; include any return flight."),
    question("disturbance_acceleration_m_s2", "External-force/model-error acceleration bound", unit="m/s²", minimum=0, help_text="Include drag, wind and model uncertainty. Nonzero wind requires a positive disturbance bound; do not infer it from wind speed alone."),
    question("disturbance_jerk_m_s3", "Disturbance acceleration rate bound", unit="m/s³", minimum=0, help_text="Bound the rate of change of the disturbance for the body-rate check."),
    question("maximum_wind_m_s", "Validated wind ceiling", unit="m/s", minimum=0),
    question("expected_wind_m_s", "Mission wind upper bound", unit="m/s", minimum=0),
    question("position_uncertainty_m", "Position error allowance", unit="m", help_text="Declare a conservative bound for navigation; not inferred from IMU/GPS hardware names."),
    question("tracking_error_m", "Tracking error allowance", unit="m", help_text="Reserve within the user's total path tolerance."),
    question("navigation", "Position source", "choice", choices=["gnss_imu", "vio", "mocap", "other"]),
    question("max_radius_m", "Operating radius from launch", unit="m"),
    question("max_height_m", "Maximum launch-relative height", unit="m"),
    question("evidence_notes", "Sources and configuration evidence", "text", help_text="Record thrust data, inertia source, controller limits/tuning, power evidence, loaded CG, battery/propellers and conditions.", max_length=4000),
]
REGULAR_FIELDS = {"arm_length_m", "rotor_minimum_thrust_n", "rotor_maximum_thrust_n", "yaw_torque_per_thrust_m"}
BY_KEY = {q["key"]: q for q in QUESTIONS}
MODEL_LIMITS = ["maximum_speed_m_s", "maximum_acceleration_m_s2", "maximum_jerk_m_s3",
                "maximum_tilt_deg", "maximum_body_rate_deg_s", "maximum_yaw_rate_deg_s",
                "maximum_yaw_acceleration_deg_s2", "maximum_yaw_jerk_deg_s3"]


def catalog():
    return {"schema_version": 1, "questions": QUESTIONS,
            "custom_rotor_fields": list(Rotor.__dataclass_fields__),
            "supported_planning": "parallel-axis fixed-pitch multicopters with 4–16 rotors",
            "supported_execution": "PX4 GNSS/IMU multicopters with a matching registered simulator",
            "unsupported": {"fixed_wing": "Needs a nonzero-airspeed planner, aerodynamic model and takeoff/landing adapter.",
                            "vtol": "Needs transition and fixed-wing planning/validation.",
                            "helicopter": "Needs a helicopter dynamics and actuator adapter."}}


def assess_profile(raw):
    """Return every missing/invalid answer, or a normalized reusable profile/model."""
    if not isinstance(raw, dict):
        raise ValueError("aircraft_profile must be an object")
    unknown = set(raw) - set(BY_KEY) - {"schema_version", "rotors"}
    if unknown:
        raise ValueError("aircraft_profile has unknown fields: " + ", ".join(sorted(unknown)))
    version = raw.get("schema_version", 1)
    if version != 1 or isinstance(version, bool) or not isinstance(version, int):
        raise ValueError("aircraft profile schema_version must be 1")
    normalized, missing, errors = {"schema_version": 1}, [], []
    for q in QUESTIONS:
        key = q["key"]
        if raw.get("layout") == "custom" and key in REGULAR_FIELDS:
            continue
        value = raw.get(key)
        if value is None or value == "":
            missing.append(q)
            continue
        try:
            if q["kind"] == "choice":
                if not isinstance(value, str) or value not in q["choices"]:
                    raise ValueError("choose one of " + ", ".join(q["choices"]))
                normalized[key] = value
            elif q["kind"] == "text":
                normalized[key] = (_identifier(value, key) if key == "profile_id" else
                                   _text(value, key, maximum=q.get("max_length", 256)))
            else:
                minimum = q.get("minimum")
                value = _number(value, key, minimum=minimum,
                                exclusive_minimum=0. if minimum is None else None)
                if "maximum" in q and value > q["maximum"]:
                    raise ValueError(f"must be <= {q['maximum']}")
                normalized[key] = value
        except ValueError as exc:
            errors.append({"field": key, "error": str(exc)})
    if raw.get("layout") == "custom":
        if raw.get("rotors") is None:
            missing.append(question("rotors", "Individual rotor geometry and limits", "rotors",
                                    help_text="Body FRD x/y about loaded CG; signed yaw coefficient; per-rotor forces in N."))
        else:
            try:
                if not isinstance(raw["rotors"], list):
                    raise ValueError("rotors must be an array")
                normalized["rotors"] = [Rotor.from_dict(r).to_dict() for r in raw["rotors"]]
            except ValueError as exc:
                errors.append({"field": "rotors", "error": str(exc)})
    compatibility = []
    if raw.get("vehicle_type") not in (None, "", "multicopter"):
        compatibility.append(catalog()["unsupported"].get(raw["vehicle_type"], "Unsupported aircraft type"))
    if raw.get("autopilot") not in (None, "", "px4"):
        compatibility.append("Execution needs a flight-stack adapter for this autopilot; PX4 is currently implemented.")
    if raw.get("navigation") not in (None, "", "gnss_imu"):
        compatibility.append("The current runner requires the GNSS/IMU navigation adapter; this source is not qualified.")
    result = dict(ready=False, missing_questions=missing, errors=errors,
                  compatibility_blockers=compatibility, flight_approval=False)
    if missing or errors or compatibility:
        return result
    p = normalized
    if p["maximum_climb_m_s"] > p["maximum_speed_m_s"] or p["maximum_descent_m_s"] > p["maximum_speed_m_s"]:
        errors.append({"field": "maximum_climb_m_s", "error": "climb/descent budget rates must not exceed the validated speed ceiling"})
    if p["expected_wind_m_s"] > p["maximum_wind_m_s"]:
        errors.append({"field": "expected_wind_m_s", "error": "mission wind exceeds the aircraft envelope"})
    if p["expected_wind_m_s"] > 0 and (p["disturbance_acceleration_m_s2"] <= 0 or p["disturbance_jerk_m_s3"] <= 0):
        errors.append({"field": "disturbance_acceleration_m_s2", "error": "nonzero wind requires justified positive force and force-rate bounds"})
    try:
        model = model_from_profile(p)
        hover = p["mass_kg"] * GRAVITY_M_S2
        disturbance = p["mass_kg"] * p["disturbance_acceleration_m_s2"]
        margin = allocation_margin(model.rotors, max(0., hover-disturbance), hover+disturbance, 0., model.roll_pitch_torque_reserve_nm)
        if margin <= 1e-9:
            raise ValueError("cannot hover with the loaded mass and simultaneous roll/pitch torque reserve")
        result.update(profile=p, profile_sha256=canonical_sha256(p),
                      vehicle_model=model.to_dict(), vehicle_model_sha256=canonical_sha256(model),
                      hover_thrust_n=hover, hover_actuator_margin=margin,
                      thrust_to_weight=model.maximum_thrust_n / hover,
                      energy_limited_flight_s=p["usable_battery_wh"] * 3600 * (1-p["reserve_fraction"]) / p["power_upper_bound_w"],
                      assumptions=["User-declared bounds must cover the complete loaded flight envelope.",
                                   "The allocator reserves roll/pitch torque; it does not identify actual torque demands.",
                                   "Power is an input upper bound, not a learned aerodynamic/battery model.",
                                   "No obstacle map or physical qualification is inferred from profile completion."])
    except ValueError as exc:
        errors.append({"field": "vehicle_model", "error": str(exc)})
    result["ready"] = not errors
    return result


def model_from_profile(p):
    if p["layout"] == "custom":
        rotors = [Rotor.from_dict(r).to_dict() for r in p["rotors"]]
    else:
        count = {"quad_x": 4, "hex_x": 6, "octo_x": 8}[p["layout"]]
        rotors = [Rotor(p["arm_length_m"] * math.cos((i+.5)*2*math.pi/count),
                        p["arm_length_m"] * math.sin((i+.5)*2*math.pi/count),
                        p["yaw_torque_per_thrust_m"] * (-1 if i % 2 else 1),
                        p["rotor_minimum_thrust_n"], p["rotor_maximum_thrust_n"]).to_dict()
                  for i in range(count)]
    model = dict(schema_version=2, vehicle_model_id=p["profile_id"],
                 configuration=p["name"], mass_kg=p["mass_kg"],
                 yaw_inertia_kg_m2=p["yaw_inertia_kg_m2"],
                 roll_pitch_torque_reserve_nm=p["roll_pitch_torque_reserve_nm"],
                 disturbance_acceleration_m_s2=p["disturbance_acceleration_m_s2"],
                 disturbance_jerk_m_s3=p["disturbance_jerk_m_s3"],
                 minimum_thrust_n=sum(r["minimum_thrust_n"] for r in rotors),
                 maximum_thrust_n=sum(r["maximum_thrust_n"] for r in rotors),
                 identified_for_physical_flight=False, rotors=rotors)
    model.update({key: p[key] for key in MODEL_LIMITS})
    return VehicleModel.from_dict(model)


def mission_budget(setup, duration_s, initial_height_m, final_height_m):
    p = setup["profile"]
    kinematic_floor = (initial_height_m / p["maximum_climb_m_s"]
                + final_height_m / p["maximum_descent_m_s"]
                + p["settle_time_s"])
    if p["lifecycle_time_upper_bound_s"] < kinematic_floor:
        raise ValueError("declared lifecycle time upper bound is below the climb/descent/settling minimum")
    overhead = p["lifecycle_time_upper_bound_s"] + p["contingency_time_s"]
    route_budget = duration_s * p["route_time_multiplier"]
    required = route_budget + overhead
    allowed = setup["energy_limited_flight_s"]
    energy = required * p["power_upper_bound_w"] / 3600.
    result = dict(route_duration_s=duration_s, route_wall_time_budget_s=route_budget,
                  kinematic_lifecycle_floor_s=kinematic_floor, lifecycle_and_contingency_s=overhead,
                  total_budgeted_s=required, allowed_flight_s=allowed,
                  upper_bound_energy_wh=energy, available_at_launch_wh=p["usable_battery_wh"],
                  reserve_fraction=p["reserve_fraction"], remaining_margin_s=allowed-required,
                  passed=required <= allowed)
    if not result["passed"]:
        raise ValueError(f"energy budget exceeded: route + takeoff/landing/contingency needs {required:.1f} s; {allowed:.1f} s available after reserve")
    return result


def controller_overrides(p):
    """Bound the existing PX4 runner by this profile, including non-route phases."""
    horizontal = p["maximum_speed_m_s"] * .9
    vertical = min(p["maximum_climb_m_s"], p["maximum_descent_m_s"]) * .9
    acceleration = min(1.5, p["maximum_acceleration_m_s2"] * .9)
    command = min(1.25, horizontal)
    tilt = min(25., p["maximum_tilt_deg"])
    return dict(max_horizontal_speed_m_s=horizontal, max_vertical_speed_m_s=vertical,
                takeoff_max_speed_m_s=min(.8, vertical), return_max_speed_m_s=min(2., horizontal),
                max_accel_m_s2=min(1., acceleration), max_jerk_m_s3=min(1.5, p["maximum_jerk_m_s3"]*.9),
                command_xy_accel_limit_m_s2=acceleration, command_z_accel_limit_m_s2=acceleration,
                navigation_hold_accel_limit_m_s2=min(3., p["maximum_acceleration_m_s2"]),
                cruise_command_speed_limit_m_s=command, cruise_reference_speed_m_s=command*.8,
                max_yaw_rate_deg_s=min(45., p["maximum_yaw_rate_deg_s"]*.9),
                max_yaw_acceleration_deg_s2=min(60., p["maximum_yaw_acceleration_deg_s2"]*.9),
                max_yaw_jerk_deg_s3=min(120., p["maximum_yaw_jerk_deg_s3"]*.9),
                max_operating_radius_m=min(80., p["max_radius_m"]),
                max_height_above_launch_m=min(30., p["max_height_m"]),
                bootstrap_max_tilt_deg=tilt, bootstrap_pitch_deg=-min(10., tilt*.5))


def import_px4_parameters(text):
    """Read-only suggestions from a QGC .params export; never motor identification."""
    if not isinstance(text, str) or len(text) > 200_000:
        raise ValueError("parameter export must be text of at most 200000 characters")
    values = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) != 5:
            raise ValueError("expected QGroundControl five-column .params format")
        system, component, name, number, _kind = fields
        if component != "1":
            continue
        value = float(number)
        if not math.isfinite(value):
            raise ValueError("non-finite PX4 parameter")
        if name in values and values[name] != value:
            raise ValueError(f"conflicting duplicate parameter {name}")
        values[name] = value
    suggestions = {}
    mapping = {
        "maximum_speed_m_s": ["MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN"],
        "maximum_acceleration_m_s2": ["MPC_ACC_HOR", "MPC_ACC_HOR_MAX", "MPC_ACC_UP_MAX", "MPC_ACC_DOWN_MAX"],
        "maximum_jerk_m_s3": ["MPC_JERK_AUTO", "MPC_JERK_MAX"],
        "maximum_tilt_deg": ["MPC_TILTMAX_AIR"],
        "maximum_body_rate_deg_s": ["MC_ROLLRATE_MAX", "MC_PITCHRATE_MAX", "MC_YAWRATE_MAX"],
        "maximum_yaw_rate_deg_s": ["MPC_YAWRAUTO_MAX", "MC_YAWRATE_MAX"],
        "maximum_yaw_acceleration_deg_s2": ["MPC_YAWRAUTO_ACC"],
    }
    for key, names in mapping.items():
        found = {name: values[name] for name in names if name in values and values[name] > 0}
        if found:
            suggestions[key] = {"value": min(found.values()), "parameters": found,
                                "complete_parameter_group": len(found) == len(names)}
    return {"suggestions": suggestions, "parameter_count": len(values),
            "requires_review": True,
            "limitations": "Controller ceilings only. Does not measure mass, thrust, inertia, power, tracking performance or flight readiness."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path)
    parser.add_argument("--parameters", type=Path)
    parser.add_argument("--output-model", type=Path)
    args = parser.parse_args()
    if args.parameters:
        result = import_px4_parameters(args.parameters.read_text())
    else:
        result = assess_profile(load_contract(args.profile, dict) if args.profile else {})
    if args.output_model:
        if not result.get("ready"):
            parser.error("cannot export a vehicle model until setup is complete")
        args.output_model.write_text(json.dumps(result["vehicle_model"], indent=2) + "\n")
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
