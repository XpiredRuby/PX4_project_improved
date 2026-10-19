"""Local, explicit physical-release preparation; never an HTTP arming API.

Records are operator attestations and integrity bindings, not cryptographic
identity or a certification of an aircraft. Original SITL artifacts stay intact.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import sys

from .aircraft import assess_profile
from .contracts import canonical_sha256, load_contract

ROOT = Path(__file__).resolve().parents[1]
CATEGORIES = ("loaded_aircraft_identification", "thrust_power_and_battery",
              "controller_and_bootstrap", "tracking_and_navigation",
              "manual_override_and_failsafes", "site_terrain_and_obstacles",
              "px4_parameters")
PLAN_FILES = ("trajectory.csv", "summary.json", "intent.json", "vehicle_model.json",
              "environment.json", "verification.json", "aircraft_profile.json",
              "mission_budget.json", "path_error_budget.json", "simulation_binding.json")
SHA = re.compile(r"[0-9a-f]{64}")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def source_digest():
    files = sorted(p for folder in ("controller", "mission_planner", "analysis", "scripts")
                   for p in ROOT.joinpath(folder).glob("*.py"))
    return canonical_sha256({str(p.relative_to(ROOT)): digest(p) for p in files})


def timestamp(value):
    if not isinstance(value, str):
        raise ValueError("a timezone-aware expiry is required")
    t = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if t.tzinfo is None:
        raise ValueError("expiry must include timezone")
    return t.timestamp()


def number(value, label, minimum=0., maximum=math.inf):
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not minimum <= value <= maximum):
        raise ValueError(f"{label} must be finite in [{minimum}, {maximum}]")
    return float(value)


def text(value, label):
    if not isinstance(value, str) or not value.strip() or len(value) > 1000:
        raise ValueError(f"{label} is required")
    return value.strip()


def local_file(root, name):
    if not isinstance(name, str) or Path(name).is_absolute():
        raise ValueError("evidence paths must be relative")
    path = (root / name).resolve()
    if not path.is_relative_to(root.resolve()) or not path.is_file():
        raise ValueError(f"evidence unavailable within record directory: {name}")
    if path.stat().st_size > 32 * 1024 * 1024:
        raise ValueError("individual evidence file exceeds 32 MiB")
    return path


def inspect_plan(plan):
    """Verify retained profile-bound nominal rehearsal and its exact runner data."""
    plan = Path(plan)
    summary = load_contract(plan / "summary.json", dict)
    for filename, key in (("intent.json", "mission_intent_sha256"),
                          ("vehicle_model.json", "vehicle_model_sha256"),
                          ("environment.json", "environment_sha256"),
                          ("verification.json", "verification_report_sha256"),
                          ("aircraft_profile.json", "aircraft_profile_sha256"),
                          ("mission_budget.json", "mission_budget_sha256"),
                          ("path_error_budget.json", "path_error_budget_sha256")):
        if canonical_sha256(load_contract(plan / filename, dict)) != summary.get(key):
            raise ValueError(f"reviewed artifact changed: {filename}")
    if (summary.get("release_class") != "SITL_ONLY"
            or digest(plan / "trajectory.csv") != summary.get("controller_trajectory_sha256")
            or load_contract(plan / "verification.json", dict).get("accepted") is not True
            or load_contract(plan / "mission_budget.json", dict).get("passed") is not True):
        raise ValueError("a verified, profile-bound SITL plan is required")
    setup = assess_profile(load_contract(plan / "aircraft_profile.json", dict))
    if not setup["ready"] or setup["vehicle_model_sha256"] != summary["vehicle_model_sha256"]:
        raise ValueError("aircraft profile/model mismatch")
    if len(setup["vehicle_model"]["rotors"]) > 12:
        raise ValueError("the PX4 execution adapter supports at most 12 rotors")
    if load_contract(plan / "intent.json", dict)["motion"]["yaw_mode"] != "fixed":
        raise ValueError("physical release currently requires rehearsed fixed heading")
    run = plan / "rehearsal/reviewed-plan"
    simulation = load_contract(run / "simulation_configuration.json", dict)
    if simulation.get("flight_release_source_sha256") != source_digest():
        raise ValueError("software changed since Gazebo rehearsal; rehearse again")
    assessment = load_contract(run / "assessment.json", dict)
    if (assessment.get("campaign_ran") is not True
            or assessment.get("sampled_rehearsal_passed") is not True
            or assessment.get("scenario_expected") != "nominal"
            or assessment.get("independent_contact_and_hold_audit", {}).get("overall_passed") is not True
            or not assessment.get("checks")
            or any(c.get("passed") is not True for c in assessment["checks"])):
        raise ValueError("a complete passing nominal Gazebo rehearsal is required")
    evidence = assessment.get("evidence_sha256", {})
    required = {"plan/trajectory.csv", "batch/reviewed-plan/audit.json",
                "batch/reviewed-plan/scenario.json", "final_ground.json"}
    if (not isinstance(evidence, dict) or not required <= set(evidence)
            or not any(name.endswith(".ulg") for name in evidence)
            or not any("/research_log_" in name for name in evidence)
            or not any("/run_manifest_" in name for name in evidence)):
        raise ValueError("assessment lacks its raw rehearsal evidence digests")
    for name, expected in evidence.items():
        path = (run / name).resolve()
        if (Path(name).is_absolute() or not path.is_relative_to(run.resolve())
                or not SHA.fullmatch(str(expected)) or digest(path) != expected):
            raise ValueError(f"raw rehearsal evidence changed: {name}")
    ground = load_contract(run / "final_ground.json", dict)
    outputs = ground.get("actuators", [])
    if (ground.get("armed") is not False or ground.get("landed_state") != 1
            or number(ground.get("continuous_confirmation_s"), "ground confirmation") < 2.
            or len(outputs) != len(setup["vehicle_model"]["rotors"])
            or any(abs(number(v, "stopped output", -1., 1.)) > .01 for v in outputs)):
        raise ValueError("rehearsal shutdown evidence is incomplete")
    manifests = list((run / "batch/reviewed-plan").glob("run_manifest_*.json"))
    if len(manifests) != 1:
        raise ValueError("exactly one rehearsal runner manifest is required")
    manifest = load_contract(manifests[0], dict)
    if (manifest.get("outcome") != "SUCCESS" or manifest.get("cleanup_error") is not None
            or manifest["controller_runtime"]["trajectory_sha256"] != digest(plan / "trajectory.csv")):
        raise ValueError("rehearsal runner did not complete this trajectory")
    for p in ROOT.joinpath("controller").glob("*.py"):
        if manifest.get("source_sha256", {}).get(p.name) != digest(p):
            raise ValueError(f"controller changed since rehearsal: {p.name}; rehearse again")
    for name in PLAN_FILES:
        if digest(plan / name) != digest(run / "plan" / name):
            raise ValueError(f"rehearsed plan changed: {name}")
    # Complete configuration, including bootstrap gains, is preserved verbatim.
    sys.path.insert(0, str(ROOT / "controller"))
    from mission_config import MissionConfig
    config = MissionConfig(**manifest["controller_config"])
    config.validate()
    if asdict(config) != manifest["controller_config"]:
        raise ValueError("rehearsal has an incomplete controller configuration")
    return summary, setup, manifest, manifests[0]


def initialize(plan, output):
    plan, output = Path(plan).resolve(), Path(output)
    summary, setup, manifest, _ = inspect_plan(plan)
    request = dict(schema_version=1, record_type="PHYSICAL_QUALIFICATION_REQUEST",
                   plan_directory=str(plan), source_sha256=source_digest(),
                   aircraft_profile_sha256=summary["aircraft_profile_sha256"],
                   controller_config=manifest["controller_config"],
                   reviewer=None, expires_at=None,
                   aircraft=dict(uid2_hex=None, flight_sw_version=None,
                                 flight_custom_version_hex=None, board_version=None,
                                 target_system=None, target_component=1),
                   site=dict(latitude_deg=None, longitude_deg=None,
                             launch_radius_m=None, cleared_radius_m=None,
                             measured_wind_upper_bound_m_s=None),
                   launch=dict(operator=None, mentor=None, mentor_present=False,
                               reviewed_at=None,
                               manual_override_tested=False, area_clear=False,
                               minimum_battery_percent=95, minimum_pack_voltage_v=None),
                   battery_id=0, motor_output_indices=None, motor_disarmed_values=None,
                   motor_stopped_tolerance=None, motor_function_parameters=None, evidence=[dict(category=c, path=None, sha256=None,
                                               reviewed_by=None) for c in CATEGORIES],
                   instructions="Unknown values stay null. Supply measured evidence and qualified human review; this template cannot arm.")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        raise ValueError("request already exists; refusing to overwrite")
    write_json(output, request)
    return request


def validate_request(request, root, *, now=None):
    now = datetime.now(timezone.utc).timestamp() if now is None else now
    if request.get("schema_version") != 1 or request.get("record_type") != "PHYSICAL_QUALIFICATION_REQUEST":
        raise ValueError("unsupported physical qualification record")
    text(request.get("reviewer"), "qualification reviewer")
    if not now < timestamp(request.get("expires_at")) <= now + 30 * 86400:
        raise ValueError("qualification must expire within 30 days and be unexpired")
    if request.get("source_sha256") != source_digest():
        raise ValueError("software changed; new rehearsal and qualification required")
    aircraft = request["aircraft"]
    uid = aircraft.get("uid2_hex")
    if not isinstance(uid, str) or not re.fullmatch(r"[0-9a-f]{36}", uid) or int(uid, 16) == 0:
        raise ValueError("a nonzero 18-byte MAVLink UID2 is required")
    for key, lo, hi in (("flight_sw_version", 1, 2**32-1), ("board_version", 0, 2**32-1),
                        ("target_system", 1, 255), ("target_component", 1, 1)):
        v = number(aircraft.get(key), key, lo, hi)
        if not v.is_integer():
            raise ValueError(f"{key} must be an integer")
    if not re.fullmatch(r"[0-9a-f]{16}", str(aircraft.get("flight_custom_version_hex"))):
        raise ValueError("exact 8-byte PX4 custom firmware version is required")
    launch = request["launch"]
    for key in ("operator", "mentor"):
        text(launch.get(key), key)
    for key in ("mentor_present", "manual_override_tested", "area_clear"):
        if launch.get(key) is not True:
            raise ValueError(f"physical launch requires {key}")
    reviewed = timestamp(launch.get("reviewed_at"))
    if not -5 <= now - reviewed <= 900:
        raise ValueError("launch conditions must be reviewed within the last 15 minutes")
    number(launch.get("minimum_battery_percent"), "minimum battery percent", 95, 100)
    number(launch.get("minimum_pack_voltage_v"), "minimum pack voltage", .1, 1000)
    if request.get("battery_id") != 0 or isinstance(request.get("battery_id"), bool):
        raise ValueError("this adapter requires one qualified pack on BATTERY_STATUS id 0")
    site = request["site"]
    number(site.get("latitude_deg"), "site latitude", -90, 90)
    number(site.get("longitude_deg"), "site longitude", -180, 180)
    number(site.get("launch_radius_m"), "launch radius", .1, 100)
    number(site.get("cleared_radius_m"), "cleared radius", .1, 10000)
    number(site.get("measured_wind_upper_bound_m_s"), "measured wind", 0, 100)
    evidence = request.get("evidence", [])
    if sorted(e.get("category", "") for e in evidence) != sorted(CATEGORIES):
        raise ValueError("each physical evidence category must be supplied exactly once")
    for e in evidence:
        text(e.get("reviewed_by"), "evidence reviewer")
        if not SHA.fullmatch(str(e.get("sha256"))) or digest(local_file(root, e.get("path"))) != e["sha256"]:
            raise ValueError(f"missing or changed evidence: {e['category']}")
    parameter_entry = next(e for e in evidence if e["category"] == "px4_parameters")
    parameters = load_contract(local_file(root, parameter_entry["path"]), dict)
    if not 30 <= len(parameters) <= 2500:
        raise ValueError("approved PX4 parameter snapshot must contain 30–2500 parameters")
    for key, value in parameters.items():
        if not re.fullmatch(r"[A-Z][A-Z0-9_]{0,15}", key):
            raise ValueError("invalid PX4 parameter name")
        number(value, key, -1e12, 1e12)
    return parameters


def prepare(request_path, output):
    request_path, output = Path(request_path).resolve(), Path(output).resolve()
    request = load_contract(request_path, dict)
    parameters = validate_request(request, request_path.parent)
    plan = Path(request["plan_directory"]).resolve()
    summary, setup, runner, _ = inspect_plan(plan)
    if (request["aircraft_profile_sha256"] != setup["profile_sha256"]
            or request["controller_config"] != runner["controller_config"]):
        raise ValueError("qualification does not match this profile and rehearsed controller")
    binding = load_contract(plan / "simulation_binding.json", dict)
    expected = {**runner["px4_parameters"], **binding["controller_parameters"]}
    for key, value in expected.items():
        if key not in parameters or not math.isclose(parameters[key], value, rel_tol=1e-6, abs_tol=1e-6):
            raise ValueError(f"approved physical parameter {key} differs from rehearsal")
    indices = request.get("motor_output_indices")
    count = len(setup["vehicle_model"]["rotors"])
    if (not isinstance(indices, list) or len(indices) != count
            or any(isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < 16 for i in indices)
            or len(set(indices)) != count):
        raise ValueError("qualified motor_output_indices must identify every distinct motor in outputs 0–15")
    stopped = request.get("motor_disarmed_values")
    if not isinstance(stopped, list) or len(stopped) != count:
        raise ValueError("bench-qualified disarmed output for each motor is required")
    for v in stopped:
        number(v, "disarmed motor output", 0, 65535)
    number(request.get("motor_stopped_tolerance"), "stopped output tolerance", 0, 5.)
    functions = request.get("motor_function_parameters")
    if (not isinstance(functions, list) or len(functions) != count or len(set(functions)) != count
            or any(name not in parameters for name in functions)
            or sorted(parameters[name] for name in functions) != list(range(101, 101+count))):
        raise ValueError("live-pinned PX4 output function parameters must map every motor exactly once")
    p, s = setup["profile"], request["site"]
    # Guard-triggered LAND must have enough energy to reach its own bounded
    # cleanup deadline, including mode/failsafe takeover and disarm evidence.
    required_contingency = runner["controller_config"]["land_timeout_s"] + 10.
    if p["contingency_time_s"] < required_contingency:
        raise ValueError(f"physical abort/landing contingency must cover at least {required_contingency:g} s")
    if (s["measured_wind_upper_bound_m_s"] > p["expected_wind_m_s"]
            or s["cleared_radius_m"] < s["launch_radius_m"] + p["max_radius_m"] + p["position_uncertainty_m"]):
        raise ValueError("qualified site/wind does not cover the aircraft operating envelope")
    if output.exists():
        raise ValueError("release directory already exists; refusing to overwrite")
    output.mkdir(parents=True)
    # Self-contained evidence, retaining original SITL labels and provenance.
    try:
        for name in PLAN_FILES:
            dest = output / "plan" / name
            dest.parent.mkdir(exist_ok=True)
            shutil.copyfile(plan / name, dest)
        run = plan / "rehearsal/reviewed-plan"
        for name in ("assessment.json", "final_ground.json", "simulation_configuration.json"):
            shutil.copyfile(run / name, output / name)
        manifests = list((run / "batch/reviewed-plan").glob("run_manifest_*.json"))
        shutil.copyfile(manifests[0], output / "rehearsal_runner.json")
        for i, e in enumerate(request["evidence"]):
            dest = f"evidence/{i}-{e['category']}{Path(e['path']).suffix}"
            (output / "evidence").mkdir(exist_ok=True)
            shutil.copyfile(local_file(request_path.parent, e["path"]), output / dest)
            e["path"] = dest
        request["plan_directory"] = "plan"
        write_json(output / "qualification.json", request)
        files = {str(f.relative_to(output)): digest(f) for f in output.rglob("*") if f.is_file()}
        release = dict(schema_version=1, release_class="PHYSICAL_FLIGHT_PREPARED",
                       source_sha256=source_digest(), files_sha256=files,
                       controller_config=runner["controller_config"],
                       rotor_count=len(setup["vehicle_model"]["rotors"]),
                       expected_px4_parameters=parameters,
                       expected_cruise_height_m=summary["controller_cruise_height_m"],
                       expires_at=request["expires_at"])
        write_json(output / "release.json", release)
        return {"release_sha256": digest(output / "release.json"), "directory": str(output),
                "status": "PREPARED_REQUIRES_LOCAL_LAUNCH_CONFIRMATION"}
    except BaseException:
        shutil.rmtree(output)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    init = commands.add_parser("init", help="Create a blocked physical qualification request")
    init.add_argument("--plan", type=Path, required=True)
    init.add_argument("--output", type=Path, required=True)
    prep = commands.add_parser("prepare", help="Freeze human-qualified release; no vehicle connection")
    prep.add_argument("--request", type=Path, required=True)
    prep.add_argument("--output", type=Path, required=True)
    launch = commands.add_parser("launch", help="Supervised physical operation; can arm")
    launch.add_argument("--release", type=Path, required=True)
    launch.add_argument("--confirm-sha256", required=True)
    launch.add_argument("--connection", required=True)
    launch.add_argument("--operator", required=True)
    launch.add_argument("--mentor", required=True)
    launch.add_argument("--mentor-present", action="store_true", required=True)
    abort = commands.add_parser("abort", help="Request LAND through the running authority guard")
    abort.add_argument("--release", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "init":
        initialize(args.plan, args.output)
        print(f"Blocked qualification request: {args.output}")
    elif args.command == "prepare":
        print(json.dumps(prepare(args.request, args.output), indent=2))
    elif args.command == "abort":
        (args.release.resolve().parent / "ABORT").touch(exist_ok=True)
        print("Abort requested; keep the controller running for PX4 LAND and shutdown confirmation.")
    else:
        from .flight_preflight import FlightAuthority
        sys.path.insert(0, str(ROOT / "controller"))
        from mission_config import MissionConfig
        import offboard_runner
        authority = FlightAuthority(args.release, args.confirm_sha256, args.operator, args.mentor,
                                    args.mentor_present, args.connection)
        offboard_runner.main(config=MissionConfig(**authority.release["controller_config"]),
                             trajectory_path=authority.root / "plan/trajectory.csv",
                             planner_summary_path=authority.root / "plan/summary.json",
                             release_authority=authority, connection_string=args.connection,
                             rotor_count=authority.release["rotor_count"])


if __name__ == "__main__":
    main()
