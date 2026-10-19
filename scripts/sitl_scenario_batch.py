#!/usr/bin/env python3
"""Sequential local SITL trials; halt the batch unless ground/disarm is confirmed."""
# ruff: noqa: E402
import argparse
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "controller"), str(ROOT / "analysis")]
import pandas as pd
from pymavlink import mavutil
from analyze_run import build_safety_audit
from analyze_fault_run import build_fault_safety_audit, build_recovery_safety_audit, build_navigation_response_audit
from sitl_fault_injector import read_parameter, run_fault, set_parameter
from trajectory_quality import trajectory_quality
from sitl_monitor import monitor_child, trial_phase_clock, allow_next_trial, quality_rejection_confirmed
from sitl_wind import gz_service, set_wind


def last_csv_row(folder):
    paths = list(folder.glob("research_log_*.csv"))
    if not paths:
        return None
    with paths[0].open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    return rows[-1] if rows else None


def fixture(world, folder, height, slope_deg=0, ridge=False):
    # Wide pad covers the launch region. No terrain height goes to the controller.
    name = "afvl_test_surface"
    angle = math.radians(slope_deg)
    size = "0.12 12 0.5" if ridge else "12 12 0.2"
    x, centre_height = (.20, .25) if ridge else (0, height - .1)
    sdf = (f'<sdf version="1.9"><model name="{name}"><static>true</static>'
           f'<pose>{x} 0 {centre_height} 0 {angle} 0</pose><link name="pad">'
           f'<collision name="collision"><geometry><box><size>{size}</size>'
           '</box></geometry></collision><visual name="visual"><geometry>'
           f'<box><size>{size}</size></box></geometry><material>'
           '<ambient>0.4 0.1 0.1 1</ambient></material></visual></link></model></sdf>')
    path = folder / "surface.sdf"
    path.write_text(sdf + "\n")
    return gz_service(world, "create", "gz.msgs.EntityFactory", "gz.msgs.Boolean",
                      f'sdf_filename: "{path}" allow_renaming: false')


def run_case(case, output, world, master):
    folder = output / case["name"]
    folder.mkdir(exist_ok=False)
    (folder / "scenario.json").write_text(json.dumps(case, indent=2) + "\n")
    wait_while_armed_or_ground(master, armed=False)
    if read_parameter(master, "SIM_GPS_USED")[0] < 4:
        raise RuntimeError("SITL GPS restoration not confirmed before next trial")
    wind_evidence = set_wind(world, (0, 0, 0))
    # Config injection is explicit and included in the mission manifest.
    code = ("import sys; from dataclasses import replace; "
            f"sys.path.insert(0,{str(ROOT / 'controller')!r}); "
            "from mission_config import MissionConfig; import offboard_runner; "
            "offboard_runner.main(replace(MissionConfig(), "
            f"tracking_governor_enabled={case.get('governor', True)!r}, "
            f"position_prediction_enabled={case.get('prediction', True)!r}))")
    events = []
    battery_restore = {}
    heartbeat_state = {"armed": None, "received_at": -math.inf}
    battery_messages = []
    if case.get("battery_drain"):
        for name in ("SIM_BAT_MIN_PCT", "SIM_BAT_DRAIN"):
            battery_restore[name] = read_parameter(master, name)
    with (folder / "controller.log").open("w") as log:
        child = subprocess.Popen([sys.executable, "-u", "-c", code],
                                 cwd=folder, stdout=log, stderr=subprocess.STDOUT)
        triggered = False
        gust_index = 0
        land_started = None
        def sample_and_inject():
            nonlocal heartbeat_state, triggered, gust_index, land_started
            for _ in range(1000):
                message = master.recv_match(blocking=False)
                if message is None:
                    break
                if message.get_srcSystem() != master.target_system or message.get_srcComponent() != 1:
                    continue
                if message.get_type() == "HEARTBEAT":
                    heartbeat_state = {"armed": bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                                       "received_at": time.monotonic()}
                elif message.get_type() == "BATTERY_STATUS":
                    battery_messages.append(message.to_dict())
            row = last_csv_row(folder)
            if row is not None:
                phase = row["phase"]
                phase_clock = trial_phase_clock(row)
                if phase == "PX4_LAND":
                    if land_started is None:
                        land_started = float(row["elapsed_s"])
                    phase_clock = float(row["elapsed_s"]) - land_started
                if case.get("gust") and phase == "TRAJECTORY":
                    pattern = [(0, (6, -3, 0)), (10, (-6, 3, 0)),
                               (20, (3, 6, 0)), (30, (-3, -6, 0)), (40, (0, 0, 0))]
                    if gust_index < len(pattern) and phase_clock >= pattern[gust_index][0]:
                        require_fresh_armed_row(row, heartbeat_state)
                        velocity = pattern[gust_index][1]
                        evidence = set_wind(world, velocity)
                        events.append({"elapsed_s": row["elapsed_s"], "phase": phase,
                                       "wind_enu_m_s": velocity, "confirmation": evidence})
                        gust_index += 1
                if not triggered and phase == case.get("trigger_phase", "NONE"):
                    if phase_clock >= case.get("trigger_clock_s", 2):
                        require_fresh_armed_row(row, heartbeat_state)
                        event = {"elapsed_s": row["elapsed_s"], "phase": phase}
                        if "gps_outage_s" in case:
                            run_fault(master, "off", case["gps_outage_s"])
                            event["gps_outage_s"] = case["gps_outage_s"]
                        elif case.get("battery_drain"):
                            for name, value in (("SIM_BAT_MIN_PCT", 5), ("SIM_BAT_DRAIN", 1)):
                                set_parameter(master, name, value, battery_restore[name][1])
                            event["battery_parameters"] = {"SIM_BAT_MIN_PCT": 5, "SIM_BAT_DRAIN": 1}
                        elif "raise_m" in case:
                            event["terrain_response"] = fixture(world, folder, case["raise_m"],
                                                               case.get("slope_deg", 0), case.get("ridge", False))
                        elif case.get("remove_surface"):
                            event["terrain_response"] = gz_service(
                                world, "remove", "gz.msgs.Entity", "gz.msgs.Boolean",
                                'name: "afvl_test_surface" type: MODEL')
                        events.append(event)
                        triggered = True

        def record_error(errors):
            (folder / "scenario_errors.json").write_text(json.dumps(errors, indent=2) + "\n")

        exit_code, scenario_errors = monitor_child(child, sample_and_inject, record_error)
    manifests = list(folder.glob("run_manifest_*.json"))
    if len(manifests) != 1:
        raise RuntimeError("No unique completed mission manifest")
    manifest = json.loads(manifests[0].read_text())
    state = manifest["final_state"]
    if state["armed"] is not False or state["landed_state"] != 1:
        raise RuntimeError("Ground/disarm not confirmed; no further trial authorized")
    wait_while_armed_or_ground(master, armed=False)
    for name, (value, kind) in battery_restore.items():
        set_parameter(master, name, value, kind)
    events.append({"reset_wind": set_wind(world, (0, 0, 0)),
                   "initial_wind": wind_evidence})
    (folder / "events.json").write_text(json.dumps(events, indent=2) + "\n")
    (folder / "battery_status.json").write_text(json.dumps(battery_messages, indent=2) + "\n")
    df = pd.read_csv(next(folder.glob("research_log_*.csv")))
    if case.get("expected") == "battery_failsafe":
        view = df.copy()
        view["phase"] = view["phase"].replace({"PX4_FAILSAFE": "PX4_LAND"})
        audit = build_safety_audit(view)
        audit["checks"] = [check for check in audit["checks"]
                           if check["name"] != "Nominal phase sequence"]
        audit["checks"].append({"name": "Native battery LAND cleanup confirmed",
                                "passed": manifest["outcome"] == "PX4_FAILSAFE"
                                and manifest["cleanup_status"] == "px4_failsafe_land_and_disarm_confirmed"
                                and manifest["cleanup_error"] is None,
                                "detail": manifest["reason"]})
        critical_states = {mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_CRITICAL,
                           mavutil.mavlink.MAV_BATTERY_CHARGE_STATE_EMERGENCY}
        warnings = [message.get("charge_state") for message in battery_messages]
        audit["checks"].append({"name": "Critical battery telemetry observed",
                                "passed": any(state in critical_states for state in warnings),
                                "detail": f"charge_states={sorted(set(warnings))}"})
        audit["overall_passed"] = all(check["passed"] for check in audit["checks"])
    elif case.get("expected") in ("navigation_response", "navigation_landing"):
        audit = build_navigation_response_audit(
            df, manifest, allow_recovery=case["expected"] == "navigation_response")
    elif case.get("expected") == "failsafe":
        audit = build_fault_safety_audit(df, manifest)
    elif case.get("expected") == "recovery":
        audit = build_recovery_safety_audit(df, manifest)
    else:
        audit = build_safety_audit(df)
    if case.get("trigger_phase") and not triggered:
        audit["overall_passed"] = False
        audit["checks"].append({"name": "Scenario injected", "passed": False,
                                "detail": "Trigger phase did not occur"})
    required_gusts = (2 if case.get("expected") in ("failsafe", "battery_failsafe")
                      or (case.get("expected") in ("navigation_response", "navigation_landing")
                          and manifest["outcome"] != "SUCCESS") else 5)
    if case.get("gust") and gust_index < required_gusts:
        audit["overall_passed"] = False
        audit["checks"].append({"name": "All gusts applied", "passed": False,
                                "detail": f"Applied {gust_index}/{required_gusts} required gust settings"})
    if scenario_errors:
        audit["overall_passed"] = False
        audit["checks"].append({"name": "Scenario infrastructure completed",
                                "passed": False, "detail": scenario_errors})
    quality = trajectory_quality(df)
    (folder / "audit.json").write_text(json.dumps(audit, indent=2, allow_nan=False) + "\n")
    (folder / "quality.json").write_text(json.dumps(quality, indent=2, allow_nan=False) + "\n")
    return {"name": case["name"], "exit_code": exit_code,
            "audit_passed": audit["overall_passed"], "outcome": manifest["outcome"],
            "quality_rejection_confirmed": quality_rejection_confirmed(manifest, df.to_dict("records")),
            "final_state": state, "scenario_errors": scenario_errors}


def require_fresh_armed_row(row, heartbeat_state):
    if (heartbeat_state["armed"] is not True
            or time.monotonic() - heartbeat_state["received_at"] > 1.5
            or row["armed"].lower() != "true"
            or time.time() - float(row["wall_time"]) > 1.0):
        raise RuntimeError("No fresh armed evidence before SITL disturbance")


def wait_while_armed_or_ground(master, armed):
    for _ in range(1000):
        if master.recv_match(blocking=False) is None:
            break
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=.5)
        if msg is None or msg.get_srcSystem() != master.target_system:
            continue
        if msg.get_srcComponent() != (master.target_component or 1):
            continue
        actual = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if actual != armed:
            raise RuntimeError("Unexpected arming state before next trial")
        return
    raise TimeoutError("No fresh PX4 heartbeat")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm-sitl", action="store_true", required=True)
    parser.add_argument("--cases", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--world", default="afvl_validation")
    args = parser.parse_args()
    # Require this task's real local simulator, not merely a UDP endpoint.
    processes = [p for p in Path("/proc").glob("[0-9]*/cmdline")]
    running = []
    for path in processes:
        try:
            running.append(path.read_bytes())
        except OSError:
            pass
    if not any(b"/PX4-Autopilot/build/px4_sitl_default/bin/px4" in p for p in running):
        raise RuntimeError("Local PX4 SITL executable is not running")
    world = args.world
    gz_service(world, "wind_info", "gz.msgs.Empty", "gz.msgs.Wind", "")
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14601", source_system=253,
                                       source_component=190)
    if master.wait_heartbeat(timeout=15) is None:
        raise TimeoutError("No local PX4 heartbeat")
    master.target_component = mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
    if read_parameter(master, "SIM_GPS_USED")[0] < 4:
        raise RuntimeError("SITL GPS is not healthy")
    args.output.mkdir(parents=True, exist_ok=False)
    results = []
    for case in json.loads(args.cases.read_text()):
        print("START", case["name"], flush=True)
        result = run_case(case, args.output, world, master)
        results.append(result)
        (args.output / "summary.json").write_text(json.dumps(results, indent=2) + "\n")
        print("RESULT", json.dumps(result), flush=True)
        if not allow_next_trial(case, result):
            raise RuntimeError("Unexpected audit failure; batch stopped after landing cleanup")
    master.close()


if __name__ == "__main__":
    main()
