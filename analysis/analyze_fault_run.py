#!/usr/bin/env python3
"""Audit expected GPS-loss recovery or PX4 failsafe SITL runs."""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from pymavlink import mavutil

from analyze_run import NOMINAL_PHASE_SEQUENCE, build_safety_audit, finite


MAX_HOLD_XY_DRIFT_M = 1.0
MAX_HOLD_Z_DRIFT_M = 1.0
MAX_HOLD_PHASE_CLOCK_CHANGE_S = 0.1
MIN_HOLD_EVIDENCE_S = 0.5


def compact_values(series):
    values = [str(value).strip() for value in series.dropna()]
    return [
        value
        for index, value in enumerate(values)
        if index == 0 or value != values[index - 1]
    ]


def fault_phase_check(df, stop_phase="PX4_FAILSAFE"):
    phases = compact_values(df.get("phase", pd.Series(dtype=str)))
    if stop_phase not in phases:
        return False, " -> ".join(phases) or "no phases"

    failsafe_index = phases.index(stop_phase)
    active_prefix = phases[:failsafe_index]
    expected_prefix = list(NOMINAL_PHASE_SEQUENCE[:-1])[: len(active_prefix)]
    valid = (
        bool(active_prefix)
        and active_prefix == expected_prefix
        and phases[failsafe_index:] == [stop_phase]
    )
    return valid, " -> ".join(phases)


def navigation_fault_check(df):
    states = compact_values(
        df.get("navigation_state", pd.Series(dtype=str))
    )
    degraded = any(state in {"DEGRADED", "HOLD", "LOST"} for state in states)
    contained = any(state in {"HOLD", "LOST"} for state in states)
    valid = bool(states) and states[0] == "HEALTHY" and degraded and contained
    return valid, " -> ".join(states) or "no navigation states"


def navigation_recovery_check(df):
    states = compact_values(
        df.get("navigation_state", pd.Series(dtype=str))
    )
    # A delayed control tick can enter HOLD directly. The required evidence
    # is containment followed by recovery, regardless of the intermediate tick.
    required = ["HEALTHY", "HOLD", "HEALTHY"]
    cursor = 0
    for state in states:
        if cursor < len(required) and state == required[cursor]:
            cursor += 1
    return cursor == len(required), " -> ".join(states) or "no navigation states"


def hold_evidence(df, stop_phase="PX4_FAILSAFE"):
    required = {
        "phase",
        "navigation_state",
        "elapsed_s",
        "phase_clock_s",
        "x",
        "y",
        "z",
    }
    missing = sorted(required - set(df.columns))
    if missing:
        return False, "missing fields: " + ", ".join(missing), {}

    stop_positions = np.flatnonzero(
        df["phase"].astype(str).eq(stop_phase).to_numpy()
    )
    if not len(stop_positions):
        return False, f"{stop_phase} phase missing", {}

    active = df.iloc[: int(stop_positions[0])]
    hold = active.loc[
        active["navigation_state"].astype(str).eq("HOLD")
    ].copy()
    numeric_names = ("elapsed_s", "phase_clock_s", "x", "y", "z")
    for name in numeric_names:
        hold[name] = finite(hold[name])
    hold = hold.dropna(subset=list(numeric_names))
    if len(hold) < 2:
        return False, "insufficient active HOLD samples", {}

    duration_s = float(hold["elapsed_s"].iloc[-1] - hold["elapsed_s"].iloc[0])
    x0 = float(hold["x"].iloc[0])
    y0 = float(hold["y"].iloc[0])
    z0 = float(hold["z"].iloc[0])
    max_xy_drift_m = float(
        np.hypot(hold["x"] - x0, hold["y"] - y0).max()
    )
    max_z_drift_m = float((hold["z"] - z0).abs().max())
    phase_clock_change_s = float(
        hold["phase_clock_s"].max() - hold["phase_clock_s"].min()
    )
    metrics = {
        "duration_s": duration_s,
        "max_xy_drift_m": max_xy_drift_m,
        "max_z_drift_m": max_z_drift_m,
        "phase_clock_change_s": phase_clock_change_s,
        "samples": int(len(hold)),
    }
    valid = (
        duration_s >= MIN_HOLD_EVIDENCE_S
        and max_xy_drift_m <= MAX_HOLD_XY_DRIFT_M
        and max_z_drift_m <= MAX_HOLD_Z_DRIFT_M
        and phase_clock_change_s <= MAX_HOLD_PHASE_CLOCK_CHANGE_S
    )
    detail = (
        f"duration={duration_s:.2f}s, XY drift={max_xy_drift_m:.3f}m, "
        f"Z drift={max_z_drift_m:.3f}m, "
        f"phase-clock change={phase_clock_change_s:.3f}s"
    )
    return valid, detail, metrics


def failsafe_manifest_checks(manifest):
    final_state = manifest.get("final_state") or {}
    return [
        {
            "name": "Expected PX4 failsafe outcome",
            "passed": manifest.get("outcome") == "PX4_FAILSAFE",
            "detail": f"outcome={manifest.get('outcome')}",
        },
        {
            "name": "Failsafe landing cleanup completed",
            "passed": (
                manifest.get("cleanup_status")
                == "px4_failsafe_land_and_disarm_confirmed"
                and manifest.get("cleanup_error") is None
            ),
            "detail": (
                f"status={manifest.get('cleanup_status')}, "
                f"error={manifest.get('cleanup_error')}"
            ),
        },
        {
            "name": "Manifest confirms ground and disarm",
            "passed": (
                final_state.get("armed") is False
                and final_state.get("landed_state")
                == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
                and final_state.get("failure_action") == "PX4_FAILSAFE"
            ),
            "detail": (
                f"armed={final_state.get('armed')}, "
                f"landed_state={final_state.get('landed_state')}, "
                f"failure_action={final_state.get('failure_action')}"
            ),
        },
    ]


def recovery_manifest_checks(manifest):
    final_state = manifest.get("final_state") or {}
    return [
        {
            "name": "Expected recovered mission outcome",
            "passed": manifest.get("outcome") == "SUCCESS",
            "detail": f"outcome={manifest.get('outcome')}",
        },
        {
            "name": "Recovered mission has no cleanup error",
            "passed": manifest.get("cleanup_error") is None,
            "detail": f"cleanup_error={manifest.get('cleanup_error')}",
        },
        {
            "name": "Recovered mission confirms ground and disarm",
            "passed": (
                final_state.get("armed") is False
                and final_state.get("landed_state")
                == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND
            ),
            "detail": (
                f"armed={final_state.get('armed')}, "
                f"landed_state={final_state.get('landed_state')}"
            ),
        },
    ]


def build_fault_safety_audit(df, manifest, controlled_land=False):
    completion_view = df.copy()
    if "phase" in completion_view:
        completion_view["phase"] = completion_view["phase"].replace(
            {"PX4_FAILSAFE": "PX4_LAND"}
        )
    baseline = build_safety_audit(completion_view)
    excluded = {"Nominal phase sequence", "No navigation loss"}
    checks = [
        item for item in baseline["checks"] if item["name"] not in excluded
    ]

    stop_phase = "PX4_LAND" if controlled_land else "PX4_FAILSAFE"
    phase_passed, phase_detail = fault_phase_check(df, stop_phase)
    nav_passed, nav_detail = navigation_fault_check(df)
    hold_passed, hold_detail, hold_metrics = hold_evidence(df, stop_phase)
    checks[:0] = [
        {
            "name": "Expected failsafe phase sequence",
            "passed": phase_passed,
            "detail": phase_detail,
        },
        {
            "name": "Navigation fault exercised and contained",
            "passed": nav_passed,
            "detail": nav_detail,
        },
        {
            "name": "HOLD remained bounded before PX4 takeover",
            "passed": hold_passed,
            "detail": hold_detail,
        },
    ]
    if controlled_land:
        state = manifest.get("final_state") or {}
        active_navigation = compact_values(df.loc[df["phase"] != stop_phase, "navigation_state"])
        checks.extend([
            {"name": "Controller navigation timeout requested landing",
             "passed": manifest.get("outcome") == "ABORTED_TO_LAND"
             and "Navigation remained unhealthy" in str(manifest.get("reason") or ""),
             "detail": manifest.get("reason")},
            {"name": "Persistent navigation fault reached LOST before landing",
             "passed": bool(active_navigation) and active_navigation[-1] == "LOST",
             "detail": " -> ".join(active_navigation)},
            {"name": "Controlled landing cleanup confirmed",
             "passed": manifest.get("cleanup_status") == "px4_land_and_disarm_confirmed"
             and manifest.get("cleanup_error") is None,
             "detail": manifest.get("cleanup_status")},
            {"name": "Controlled landing confirms ground and disarm",
             "passed": state.get("armed") is False and state.get("landed_state") == 1
             and state.get("failure_action") == "LAND",
             "detail": state},
        ])
    else:
        checks.extend(failsafe_manifest_checks(manifest))
    return {
        "overall_passed": all(item["passed"] for item in checks),
        "checks": checks,
        "hold_metrics": hold_metrics,
    }


def build_recovery_safety_audit(df, manifest):
    baseline = build_safety_audit(df)
    checks = list(baseline["checks"])
    nav_passed, nav_detail = navigation_recovery_check(df)
    hold_passed, hold_detail, hold_metrics = hold_evidence(
        df,
        stop_phase="PX4_LAND",
    )
    checks[:0] = [
        {
            "name": "Navigation recovered after bounded HOLD",
            "passed": nav_passed,
            "detail": nav_detail,
        },
        {
            "name": "Recovery HOLD remained bounded",
            "passed": hold_passed,
            "detail": hold_detail,
        },
    ]
    checks.extend(recovery_manifest_checks(manifest))
    return {
        "overall_passed": all(item["passed"] for item in checks),
        "checks": checks,
        "hold_metrics": hold_metrics,
    }


def build_navigation_response_audit(df, manifest, allow_recovery=True):
    """Accept bounded recovery or verified landing, never a quality violation."""
    outcome = manifest.get("outcome")
    if outcome == "SUCCESS" and allow_recovery:
        audit = build_recovery_safety_audit(df, manifest)
    elif outcome == "PX4_FAILSAFE":
        audit = build_fault_safety_audit(df, manifest)
    elif outcome == "ABORTED_TO_LAND":
        audit = build_fault_safety_audit(df, manifest, controlled_land=True)
    else:
        audit = build_safety_audit(df)
        audit["checks"].append({"name": "Expected navigation response",
                                "passed": False, "detail": f"outcome={outcome}"})
        audit["overall_passed"] = False
    audit["navigation_response"] = outcome
    return audit


def newest(path, pattern):
    matches = sorted(path.glob(pattern), key=lambda item: item.stat().st_mtime)
    if not matches:
        raise FileNotFoundError(f"No {pattern} file found in {path}")
    return matches[-1]


def main():
    parser = argparse.ArgumentParser(
        description="Audit an expected PX4 GPS-loss SITL run."
    )
    parser.add_argument("run_dir", type=Path)
    parser.add_argument(
        "--expected",
        choices=("failsafe", "recovery", "navigation_response", "navigation_landing"),
        default="failsafe",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    csv_path = newest(run_dir, "research_log_*.csv")
    manifest_path = newest(run_dir, "run_manifest_*.json")
    with manifest_path.open(encoding="utf-8") as handle:
        manifest = json.load(handle)
    dataframe = pd.read_csv(csv_path)
    if args.expected == "failsafe":
        audit = build_fault_safety_audit(dataframe, manifest)
    elif args.expected == "recovery":
        audit = build_recovery_safety_audit(dataframe, manifest)
    else:
        audit = build_navigation_response_audit(
            dataframe, manifest, allow_recovery=args.expected == "navigation_response")

    default_name = (
        "fault_safety_audit.json"
        if args.expected == "failsafe"
        else args.expected + "_safety_audit.json"
    )
    output = args.output or run_dir / default_name
    with output.open("w", encoding="utf-8") as handle:
        json.dump(audit, handle, indent=2, sort_keys=True)
        handle.write("\n")

    for item in audit["checks"]:
        status = "PASS" if item["passed"] else "FAIL"
        print(f"[{status}] {item['name']}: {item['detail']}")
    print(f"Audit: {output}")
    raise SystemExit(0 if audit["overall_passed"] else 1)


if __name__ == "__main__":
    main()
