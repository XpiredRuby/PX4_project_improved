#!/usr/bin/env python3
"""Run isolated local PX4/Gazebo planner scenarios and retain their evidence.

This is SITL-only infrastructure. It requires a compiled local PX4 tree and
never selects a serial device or a non-loopback MAVLink endpoint.
"""
# ruff: noqa: E402
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "scripts")]
from pymavlink import mavutil
from mission_planner.acknowledgement import issue_sitl_acknowledgement, load_sitl_acknowledgement
from mission_planner.contracts import canonical_sha256, load_contract
from mission_planner.plan_drawn_path import compile_drawn_path
from mission_planner.plan_interview import compile_interview
from mission_planner.acceptance_cases import acceptance_cases
from mission_planner.simulation_binding import validate_binding
from mission_planner.aircraft import assess_profile, controller_overrides
from mission_planner.flight_release import source_digest
from sitl_scenario_batch import run_case
from sitl_fault_injector import read_parameter, set_parameter


PARAMETERS = {
    "EKF2_GPS_CTRL": 7, "EKF2_HGT_REF": 1, "EKF2_BARO_CTRL": 0,
    "EKF2_MAG_TYPE": 5, "EKF2_OF_CTRL": 0, "EKF2_EV_CTRL": 0,
    "EKF2_RNG_CTRL": 0, "EKF2_AGP_CTRL": 0, "EKF2_NOAID_TOUT": 6000000,
    "MPC_LAND_SPEED": .4, "MPC_LAND_CRWL": .3,
    "COM_OF_LOSS_T": .5, "COM_OBL_RC_ACT": 4, "COM_DISARM_LAND": 2,
    "COM_LOW_BAT_ACT": 2, "BAT_LOW_THR": .2, "BAT_CRIT_THR": .15,
    "BAT_EMERGEN_THR": .1, "SYS_FAILURE_EN": 0,
}


def write_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".pending")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def ground_evidence(master, timeout_s=8., rotor_count=4):
    """Require fresh disarm, ground and zero actuator outputs for two seconds."""
    for _ in range(1000):
        if master.recv_match(blocking=False) is None:
            break
    for message_id in (245, 375):
        master.mav.command_long_send(master.target_system, 1,
                                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                     0, message_id, 100000, 0, 0, 0, 0, 0)
    seen, timestamps = {}, {}
    started = time.monotonic()
    safe_since = None
    while time.monotonic() - started < timeout_s:
        packet = master.recv_match(blocking=True, timeout=.2)
        now = time.monotonic()
        if packet is not None and packet.get_srcSystem() == master.target_system and packet.get_srcComponent() == 1:
            kind = packet.get_type()
            if kind == "HEARTBEAT":
                seen["armed"] = bool(packet.base_mode & 128)
                timestamps["armed"] = now
            elif kind == "EXTENDED_SYS_STATE":
                seen["landed_state"] = packet.landed_state
                timestamps["landed_state"] = now
            elif kind == "ACTUATOR_OUTPUT_STATUS":
                seen["actuators"] = list(packet.actuator[:rotor_count])
                timestamps["actuators"] = now
        fresh = len(timestamps) == 3 and all(now - t <= 1.5 for t in timestamps.values())
        safe = (fresh and seen.get("armed") is False and seen.get("landed_state") == 1
                and len(seen.get("actuators", [])) == rotor_count
                and all(math.isfinite(v) and abs(v) <= .01 for v in seen["actuators"]))
        if safe:
            safe_since = safe_since if safe_since is not None else now
            if now - safe_since >= 2.:
                return {**seen, "continuous_confirmation_s": now - safe_since}
        else:
            safe_since = None
    raise RuntimeError(f"Fresh ground/disarm/zero-output evidence missing: {seen}")


def connect():
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14601", source_system=253,
                                       source_component=190)
    if master.wait_heartbeat(timeout=15) is None:
        master.close()
        raise RuntimeError("Local simulator heartbeat unavailable")
    master.target_component = 1
    return master


def prepare_case(name, output, reviewed_plan=None):
    artifacts = output / "plan"
    artifacts.mkdir()
    trajectory, summary = artifacts / "trajectory.csv", artifacts / "summary.json"
    case = {"name": name, "expected": "nominal", "trajectory": str(trajectory)}
    if name == "reviewed-plan":
        if reviewed_plan is None:
            raise ValueError("reviewed-plan requires a retained studio artifact")
        for filename in ("trajectory.csv", "summary.json", "intent.json", "verification.json", "vehicle_model.json", "environment.json", "sitl_acknowledgement.json"):
            shutil.copy2(reviewed_plan / filename, artifacts / filename)
        for filename in ("aircraft_profile.json", "mission_budget.json", "path_error_budget.json", "simulation_binding.json"):
            if (reviewed_plan / filename).is_file():
                shutil.copy2(reviewed_plan / filename, artifacts / filename)
        declared = load_contract(summary, dict)
        verification = load_contract(artifacts / "verification.json", dict)
        intent = load_contract(artifacts / "intent.json", dict)
        acknowledgement = artifacts / "sitl_acknowledgement.json"
        load_sitl_acknowledgement(acknowledgement, summary, trajectory)
        if (verification.get("accepted") is not True
                or canonical_sha256(verification) != declared["verification_report_sha256"]
                or canonical_sha256(intent) != declared["mission_intent_sha256"]
                or canonical_sha256(load_contract(artifacts / "vehicle_model.json", dict)) != declared["vehicle_model_sha256"]
                or canonical_sha256(load_contract(artifacts / "environment.json", dict)) != declared["environment_sha256"]):
            raise ValueError("retained verification or intent does not match the reviewed plan")
        if "aircraft_profile_sha256" in declared:
            for filename, key in (("aircraft_profile.json", "aircraft_profile_sha256"),
                                  ("mission_budget.json", "mission_budget_sha256"),
                                  ("path_error_budget.json", "path_error_budget_sha256")):
                if canonical_sha256(load_contract(artifacts / filename, dict)) != declared[key]:
                    raise ValueError("aircraft profile or mission budget does not match the reviewed plan")
            setup = assess_profile(load_contract(artifacts / "aircraft_profile.json", dict))
            if not setup["ready"] or setup["vehicle_model_sha256"] != declared["vehicle_model_sha256"]:
                raise ValueError("aircraft setup no longer matches the reviewed vehicle model")
            case["controller_overrides"] = controller_overrides(setup["profile"])
            if load_contract(artifacts / "mission_budget.json", dict).get("passed") is not True:
                raise ValueError("aircraft mission budget failed")
        if intent["motion"]["yaw_mode"] != "fixed":
            raise ValueError("connected studio rehearsal requires fixed heading")
        case.update(planner_summary=str(summary), sitl_acknowledgement=str(acknowledgement),
                    cruise_height_m=declared["controller_cruise_height_m"])
        return case
    if name == "five-stop-baseline":
        subprocess.run([sys.executable, str(ROOT / "trajectory_generator/plan_waypoint_mission.py"),
                        "--plan", str(ROOT / "missions/five_stop_example.json"),
                        "--output", str(trajectory), "--summary", str(summary)],
                       cwd=ROOT / "trajectory_generator", check=True)
        case["waypoint_summary"] = str(summary)
        return case
    vehicle = artifacts / "vehicle_model.json"
    environment = artifacts / "environment.json"
    shutil.copy2(ROOT / "config/planner/x500_sitl_vehicle_model.json", vehicle)
    shutil.copy2(ROOT / "config/planner/empty_world_environment.json", environment)
    common = dict(
        intent_path=artifacts / "intent.json",
        vehicle_path=vehicle,
        environment_path=environment,
        trajectory_path=trajectory, report_path=artifacts / "verification.json",
        summary_path=summary, review_path=artifacts / "review.html",
        control_spacing_m=.5, verification_resolution_m=.02,
        sample_period_s=.05, limit_scale=.9, controller_cruise_height_m=20.,
        deviation_utilization=.75,
    )
    reference_cases = {"turn-nominal": "l-turn", "hard-stop-nominal": "hard-stop",
                       "survey-nominal": "survey"}
    if name in reference_cases:
        payload = next(payload for label, payload, accepted in acceptance_cases()
                       if label == reference_cases[name] and accepted)
        if payload["source"] == "drawing":
            write_json(common["intent_path"], payload["document"])
            result = compile_drawn_path(**common)
        else:
            source = artifacts / "interview.json"
            write_json(source, payload["document"])
            result = compile_interview(interview_path=source, **common)
    elif name.startswith("circle"):
        interview = json.loads((ROOT / "missions/interview_circle_example.json").read_text())
        # Fixed heading isolates circle tracking from launch-heading alignment.
        interview.update(yaw_mode="fixed", fixed_yaw_deg=0., altitude_m=20.)
        source = artifacts / "interview.json"
        write_json(source, interview)
        result = compile_interview(interview_path=source, **common)
    else:
        intent = json.loads((ROOT / "missions/drawn_path_example.json").read_text())
        for point in intent["path"]["points"]:
            point["down_m"] -= 3.
        write_json(common["intent_path"], intent)
        result = compile_drawn_path(**common)
    ack = issue_sitl_acknowledgement(
        controller_trajectory_sha256=result["controller_trajectory_sha256"],
        candidate_trajectory_sha256=result["candidate_trajectory_sha256"],
        verification_report_sha256=result["verification_report_sha256"],
        expires_at=(datetime.now(timezone.utc) + timedelta(hours=2)).isoformat(),
        acknowledged_by="automated-sitl-regression-harness",
    )
    acknowledgement = artifacts / "sitl_acknowledgement.json"
    write_json(acknowledgement, ack)
    case.update(planner_summary=str(summary), sitl_acknowledgement=str(acknowledgement),
                cruise_height_m=20.)
    if name == "drawn-gps-outage-4s":
        case.update(expected="navigation_response", gps_outage_s=4.,
                    trigger_phase="TRAJECTORY", trigger_clock_s=8.)
    if name == "drawn-control-stall":
        case.update(expected="control_stall", control_stall_s=.4, stall_clock_s=8.)
    return case


def run_one(name, args):
    folder = args.output / name
    folder.mkdir()
    case = prepare_case(name, folder, args.reviewed_plan)
    if name == "reviewed-plan" and args.reviewed_plan is not None:
        # The owner-controlled marker exists in the retained studio plan.
        # This does not introduce a hardware endpoint or allow process killing.
        case["studio_cancel_marker"] = str(args.reviewed_plan / "CANCEL_REQUEST.json")
    binding_path = folder / "plan/simulation_binding.json"
    binding = validate_binding(load_contract(binding_path, dict), args.autopilot) if binding_path.is_file() else None
    if (folder / "plan/aircraft_profile.json").exists() and binding is None:
        raise ValueError("aircraft-specific rehearsal requires a matching registered simulator")
    if binding is not None:
        summary = load_contract(folder / "plan/summary.json", dict)
        if (summary.get("aircraft_profile_sha256") != binding["profile_sha256"]
                or summary["vehicle_model_sha256"] != binding["vehicle_model_sha256"]):
            raise ValueError("aircraft does not match the registered simulator")
    rotor_count = len(binding["rotors"]) if binding else 4
    parameters = dict(PARAMETERS)
    if binding is not None:
        parameters.update(binding["controller_parameters"])
        parameters["MPC_LAND_SPEED"] = min(parameters["MPC_LAND_SPEED"], binding["profile"]["maximum_descent_m_s"])
        parameters["MPC_LAND_CRWL"] = min(parameters["MPC_LAND_CRWL"], parameters["MPC_LAND_SPEED"])
    runtime = folder / "runtime"
    runtime.mkdir()
    build = args.autopilot / "build/px4_sitl_default"
    shutil.copy2(build / "rootfs/gz_env.sh", runtime / "gz_env.sh")
    binary = (build / "bin/px4").resolve()
    environment = os.environ.copy()
    for key in ("PX4_GZ_STANDALONE", "GZ_PARTITION", "PX4_GZ_MODEL_NAME"):
        environment.pop(key, None)
    environment.update(PX4_SYS_AUTOSTART=str(binding["airframe_id"]) if binding else "4001",
                       PX4_SIM_MODEL="gz_" + binding["model"] if binding else "gz_x500",
                       PX4_GZ_WORLD=args.world, PX4_GZ_MODEL_POSE="0,0,0,0,0,0",
                       PX4_SIM_SPEED_FACTOR="1", GZ_IP="127.0.0.1")
    if args.gui:
        environment.pop("HEADLESS", None)
    else:
        environment["HEADLESS"] = "1"
    environment.update({"PX4_PARAM_" + key: str(value) for key, value in parameters.items()})
    config = {"release_class": "SITL_ONLY", "case": case, "parameters": parameters,
              "flight_release_source_sha256": source_digest(),
              "world": args.world, "simulation_speed_factor": 1,
              "world_sha256": digest(args.autopilot / f"Tools/simulation/gz/worlds/{args.world}.sdf"),
              "px4_executable_sha256": digest(binary),
              "controller_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
              "controller_batch_sha256": digest(ROOT / "scripts/sitl_scenario_batch.py"),
              "harness_sha256": digest(Path(__file__)),
              "aircraft_profile_sha256": binding["profile_sha256"] if binding else None,
              "simulation_binding": binding,
              "model_files_sha256": {
                  str(path.relative_to(args.autopilot)): digest(path)
                  for model in ("x500", "x500_base")
                  for path in sorted((args.autopilot / "Tools/simulation/gz/models" / model).rglob("*"))
                  if path.is_file()
              } if binding is None else binding["model_files_sha256"]}
    write_json(folder / "simulation_configuration.json", config)
    log = (folder / "simulator.log").open("w")
    process = subprocess.Popen([str(binary), "-d", str(build / "etc"), "-w", str(runtime)],
                               cwd=runtime, env=environment, stdin=subprocess.DEVNULL,
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    write_json(folder / "owned_process.json", {"pid": process.pid, "pgid": os.getpgid(process.pid)})
    master = None
    gcs = mavutil.mavlink_connection("udpin:127.0.0.1:14550", source_system=254,
                                    source_component=190)
    stop_gcs = threading.Event()

    def ground_station_heartbeat():
        # Keep PX4's GCS health check active using an explicit simulated GCS.
        # This is transport liveness, not human approval or control authority.
        while not stop_gcs.is_set():
            for _ in range(100):
                if gcs.recv_match(blocking=False) is None:
                    break
            gcs.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                   mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                   0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            stop_gcs.wait(.5)

    gcs_thread = threading.Thread(target=ground_station_heartbeat, daemon=True)
    gcs_thread.start()
    try:
        deadline = time.monotonic() + 60
        while "Startup script returned successfully" not in (folder / "simulator.log").read_text(errors="replace"):
            if process.poll() is not None:
                raise RuntimeError("Owned simulator exited during startup")
            if time.monotonic() > deadline:
                raise RuntimeError("Owned simulator startup exceeded 60 s")
            time.sleep(.5)
        subprocess.run([str(build / "bin/px4-mavlink"), "start", "-u", "14600", "-o", "14601",
                        "-t", "127.0.0.1", "-m", "onboard", "-r", "400000"],
                       capture_output=True, timeout=10, check=True)
        master = connect()
        write_json(folder / "initial_ground.json", ground_evidence(master, rotor_count=rotor_count))
        before = {key: read_parameter(master, key) for key in parameters}
        estimator = {key for key in parameters if key.startswith("EKF2_")}
        if any(abs(before[key][0] - parameters[key]) > 1e-5 for key in estimator):
            raise RuntimeError("Estimator startup overrides did not take effect; reboot/configuration required")
        # Airframe startup can override some safety defaults after the rcS
        # environment loop. Apply only to this owned, disarmed SITL runtime.
        for key, value in parameters.items():
            if abs(before[key][0] - value) > 1e-5:
                set_parameter(master, key, value, before[key][1])
        actual = {key: read_parameter(master, key)[0] for key in parameters}
        if any(abs(actual[key] - value) > 1e-5 for key, value in parameters.items()):
            raise RuntimeError("SITL parameter readback differs from the declared configuration")
        write_json(folder / "effective_parameters.json", {"before": before, "verified": actual})
        batch = folder / "batch"
        batch.mkdir()
        result = run_case(case, batch, args.world, master)
        write_json(folder / "result.json", result)
        if not result.get("audit_passed"):
            raise RuntimeError(f"Scenario audit failed: {name}")
        return result
    finally:
        if master is not None:
            master.close()
        if process.poll() is None:
            # Do not terminate an airborne runner. A missing ground check
            # blocks cleanup and all later cases and remains visible.
            check = connect()
            try:
                write_json(folder / "final_ground.json", ground_evidence(check, rotor_count=rotor_count))
            finally:
                check.close()
            command = Path(f"/proc/{process.pid}/cmdline").read_bytes()
            if str(binary).encode() not in command or str(runtime).encode() not in command or os.getpgid(process.pid) != process.pid:
                raise RuntimeError("Owned simulator identity changed; shutdown refused")
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=15)
        log.close()
        stop_gcs.set()
        gcs_thread.join(timeout=2.)
        gcs.close()



def cancellation_verified(run, marker):
    """A cancelled verdict requires actual native LAND/disarm evidence."""
    run, marker = Path(run), Path(marker)
    if not marker.is_file() or not (run / "final_ground.json").is_file():
        return False
    manifests = list((run / "batch/reviewed-plan").glob("run_manifest_*.json"))
    if len(manifests) != 1:
        return False
    try:
        request = load_contract(marker, dict)
        ground = load_contract(run / "final_ground.json", dict)
        manifest = load_contract(manifests[0], dict)
        state = manifest.get("final_state", {})
        outputs = ground.get("actuators", [])
        return (request.get("requested") is True
                and request.get("release_class") == "SITL_ONLY"
                and "studio simulation cancellation requested" in manifest.get("reason", "")
                and manifest.get("outcome") in ("PX4_FAILSAFE", "ABORTED_TO_LAND")
                and manifest.get("cleanup_error") is None
                and manifest.get("cleanup_status") in
                    ("px4_land_and_disarm_confirmed",
                     "px4_failsafe_land_and_disarm_confirmed")
                and state.get("armed") is False and state.get("landed_state") == 1
                and ground.get("armed") is False and ground.get("landed_state") == 1
                and float(ground.get("continuous_confirmation_s", 0)) >= 2.
                and bool(outputs) and all(math.isfinite(v) and abs(v) <= .01 for v in outputs))
    except (OSError, ValueError, TypeError, KeyError):
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-sitl", action="store_true", required=True)
    parser.add_argument("--autopilot", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--world", default="afvl_validation")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--reviewed-plan", type=Path)
    parser.add_argument("--cases", nargs="+", choices=["five-stop-baseline", "drawn-nominal", "circle-nominal", "turn-nominal", "hard-stop-nominal", "survey-nominal", "drawn-gps-outage-4s", "drawn-control-stall", "reviewed-plan"], default=["five-stop-baseline"])
    args = parser.parse_args()
    if (args.reviewed_plan is not None) != (args.cases == ["reviewed-plan"]):
        parser.error("--reviewed-plan requires exactly --cases reviewed-plan")
    # One controller owns the simulator ports across studio/browser processes.
    import fcntl
    lock = open("/tmp/px4-trajectory-studio.lock", "a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        raise RuntimeError("Another planner campaign owns the local simulator") from exc
    args.autopilot = args.autopilot.resolve()
    args.output = args.output.resolve()
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            command = path.read_bytes()
        except OSError:
            continue
        if b"/build/px4_sitl_default/bin/px4\x00" in command:
            raise RuntimeError("A PX4 simulator is already running; existing process preserved")
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    try:
        for name in args.cases:
            write_json(args.output / "status.json", {"stage": "RUNNING", "case": name})
            results.append(run_one(name, args))
            # Assess after the owned simulator has landed, stopped propulsion,
            # and closed its ULog. Startup or a lifecycle pass is insufficient.
            sys.path.insert(0, str(ROOT / "analysis"))
            from planner_rehearsal import assess_case
            assessment = assess_case(args.output / name)
            results[-1]["sampled_rehearsal_passed"] = assessment["sampled_rehearsal_passed"]
            write_json(args.output / "results.json", results)
            if name in ("drawn-nominal", "circle-nominal", "turn-nominal", "hard-stop-nominal", "survey-nominal", "reviewed-plan") and not assessment["sampled_rehearsal_passed"]:
                raise RuntimeError("Sampled rehearsal targets failed; inspect assessment.json")
            if name in ("drawn-gps-outage-4s", "drawn-control-stall") and not assessment["fault_response_passed"]:
                raise RuntimeError("Fault response targets failed; inspect assessment.json")
    except Exception as exc:
        cancelled = (name == "reviewed-plan" and args.reviewed_plan is not None
                     and cancellation_verified(args.output / name,
                                               args.reviewed_plan / "CANCEL_REQUEST.json"))
        if cancelled:
            write_json(args.output / "status.json", {
                "stage": "CANCELLED", "case": name,
                "cancel_confirmed": True,
                "message": "Simulator cancellation verified: PX4 landing, disarm and stopped outputs."})
            return
        write_json(args.output / "status.json", {"stage": "FAILED", "case": name,
                                                  "error": f"{type(exc).__name__}: {exc}"})
        raise
    write_json(args.output / "status.json", {"stage": "COMPLETED", "case_count": len(results)})


if __name__ == "__main__":
    main()
