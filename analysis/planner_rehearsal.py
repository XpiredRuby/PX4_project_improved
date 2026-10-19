"""Predeclared sampled SITL checks against independent simulator truth.

This assessment never clears hardware or certifies physical dynamics. Mission
time may be retarded by the existing tracking governor; wall time is reported.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from sitl_ground_truth import audit_ground_truth, interpolation_coverage, ordered_samples, source_window
from trajectory_quality import distribution


def assess_frames(df, plan, physical, yaw, velocity, audit, ground):
    checks = []

    def add(name, passed, detail):
        checks.append({"name": name, "passed": bool(passed), "detail": detail})

    elapsed = df.elapsed_s.to_numpy(dtype=float)
    if not len(elapsed) or not np.isfinite(elapsed).all() or np.any(np.diff(elapsed) <= 0):
        raise ValueError("telemetry elapsed time must increase and be finite")
    add("Existing lifecycle/scenario audit", audit.get("overall_passed") is True, audit.get("checks", []))
    gaps = np.diff(elapsed)
    add("Telemetry gaps at most 0.25 s", len(gaps) > 0 and np.max(gaps) <= .25,
        {"maximum_s": float(np.max(gaps)) if len(gaps) else None})
    mission = df.phase.eq("TRAJECTORY").to_numpy()
    query = df.loc[mission, "mission_time_s"].to_numpy(dtype=float)
    times = plan.time.to_numpy(dtype=float)
    if not len(times) or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("planned time must increase and be finite")
    complete = (len(query) >= 2 and np.isfinite(query).all()
                and np.all(np.diff(query) >= 0) and query[0] <= times[0] + .1
                and query[-1] >= times[-1] - .1 and query[-1] <= times[-1] + .1)
    add("Entire planned mission clock covered", complete,
        {"planned_s": float(times[-1]), "first_s": float(query[0]) if len(query) else None,
         "last_s": float(query[-1]) if len(query) else None,
         "endpoint_sampling_tolerance_s": .1})
    reference = df.loc[mission, ["desired_x", "desired_y", "desired_z"]].to_numpy(dtype=float)
    planned = np.column_stack([np.interp(query, times, plan[c]) for c in ("x", "y", "z")])
    offsets = reference - planned
    bound = (len(offsets) > 0 and np.isfinite(offsets).all()
             and np.max(np.abs(offsets - offsets[0])) <= 1e-5)
    add("Runtime reference follows exact CSV with one translation", bound,
        {"launch_translation_ned_m": offsets[0].tolist() if len(offsets) else None})
    yaw_ref = df.loc[mission, "target_yaw_unwrapped"].to_numpy(dtype=float)
    yaw_plan = np.interp(query, times, plan.yaw)
    yaw_offsets = np.unwrap(yaw_ref - yaw_plan)
    add("Runtime heading follows exact CSV with one launch offset",
        len(yaw_offsets) > 0 and np.isfinite(yaw_offsets).all()
        and np.max(np.abs(yaw_offsets - yaw_offsets[0])) <= 1e-5,
        {"launch_offset_rad": float(yaw_offsets[0]) if len(yaw_offsets) else None})
    contacts = np.flatnonzero(df.phase.isin(["PX4_LAND", "PX4_FAILSAFE"]).to_numpy()
                             & df.landed_state.eq(1).to_numpy())
    active = np.arange(len(df)) <= (contacts[0] if len(contacts) else len(df))
    coverage = np.isfinite(physical).all(axis=1) & np.isfinite(yaw) & np.isfinite(velocity).all(axis=1)
    add("Truth covers every recorded mission sample through contact", len(contacts) > 0 and np.all(coverage[active]),
        {"required_rows": int(active.sum()), "covered_rows": int(coverage[active].sum()),
         "post_contact_rows_excluded": int((~active).sum())})
    offboard = df.phase.isin(["TAKEOFF", "TRAJECTORY", "RETURN_HOME", "ALIGN"]).to_numpy()
    desired = df[["desired_x", "desired_y", "desired_z"]].to_numpy(dtype=float)
    errors = np.linalg.norm(physical - desired, axis=1)
    yaw_error = np.degrees(np.arctan2(np.sin(yaw - df.target_yaw), np.cos(yaw - df.target_yaw)))
    position_stats, heading_stats = distribution(errors[offboard]), distribution(yaw_error[offboard])
    add("Physical position tracking p95 <= 0.5 m and max <= 1 m",
        position_stats is not None and position_stats["p95"] <= .5 and position_stats["max_abs"] <= 1., position_stats)
    add("Physical heading tracking p95 <= 5 deg and max <= 10 deg",
        heading_stats is not None and heading_stats["p95"] <= 5. and heading_stats["max_abs"] <= 10., heading_stats)
    if len(contacts):
        index = contacts[0]
        landing = df.loc[index, ["native_landing_reference_x", "native_landing_reference_y"]].to_numpy(dtype=float)
        error = float(np.linalg.norm(physical[index, :2] - landing))
        horizontal = float(np.linalg.norm(velocity[index, :2]))
        vertical = float(abs(velocity[index, 2]))
        contact_ok = np.isfinite([error, horizontal, vertical]).all() and error <= 1 and horizontal <= .2 and vertical <= .2
        detail = {"position_error_m": error if np.isfinite(error) else None,
                  "horizontal_speed_m_s": horizontal if np.isfinite(horizontal) else None,
                  "vertical_speed_m_s": vertical if np.isfinite(vertical) else None,
                  "contact_source": "first PX4 ON_GROUND sample during native landing; rigid-body contact timing not independently identified"}
    else:
        contact_ok, detail = False, {"error": "native landing contact absent"}
    add("Touchdown truth within predeclared position/speed targets", contact_ok, detail)
    add("Fresh ground/disarm/zero outputs continuously confirmed for 2 s",
        ground.get("armed") is False and ground.get("landed_state") == 1
        and ground.get("continuous_confirmation_s", 0) >= 2.
        and len(ground.get("actuators", [])) == 4
        and all(np.isfinite(v) and abs(v) <= .01 for v in ground.get("actuators", [])), ground)
    wall = df.wall_time.to_numpy(dtype=float)
    simulated = df.position_time_boot_ms.to_numpy(dtype=float) / 1000.
    indices = np.searchsorted(wall, wall + 1.)
    valid = indices < len(wall)
    left, right = np.flatnonzero(valid), indices[valid]
    ordered = (np.isfinite(wall).all() and np.isfinite(simulated).all()
               and np.all(np.diff(wall) > 0) and np.all(np.diff(simulated) >= 0))
    mean = float((simulated[-1] - simulated[0]) / (wall[-1] - wall[0])) if ordered and len(wall) > 1 else None
    factors = ((simulated[right] - simulated[left]) / (wall[right] - wall[left])) if ordered else np.array([])
    factor_stats = distribution(factors)
    add("Measured mean real-time factor in 0.95–1.05", mean is not None and .95 <= mean <= 1.05,
        {"mean": mean, "one_second_windows": factor_stats,
         "windows_below_0_9": int(np.sum(factors < .9)), "windows_above_1_1": int(np.sum(factors > 1.1))})
    return {"schema_version": 1, "release_class": "SITL_ONLY", "campaign_ran": True,
            "sampled_rehearsal_passed": all(c["passed"] for c in checks),
            "hardware_approval": False, "checks": checks,
            "per_phase": {str(phase): {"position_error_m": distribution(errors[df.phase.eq(phase)]),
                                       "heading_error_deg": distribution(yaw_error[df.phase.eq(phase)])}
                          for phase in ("TAKEOFF", "TRAJECTORY", "RETURN_HOME", "ALIGN")},
            "scope": "sampled tracking, reference binding, land-detector contact, lifecycle and clock evidence; no hardware clearance",
            "timing_scope": "adaptive mission clock; planned derivatives are not a certificate of actual wall-time dynamics",
            "remaining_qualification": ["pre-CSV yaw bootstrap (guarded and retained in ULog, not independently scored here)",
                                        "rigid-body contact timing", "actual dynamics and actuator reserves",
                                        "identified aircraft and environment", "obstacle-map clearance if applicable"]}


def assess_case(folder):
    from pyulog import ULog
    folder = Path(folder)
    run = folder / "batch" / folder.name
    def unique(paths):
        paths = list(paths)
        if len(paths) != 1:
            raise ValueError(f"expected one evidence file; found {len(paths)}")
        return paths[0]
    csv_path = unique(run.glob("research_log_*.csv"))
    ulog_path = unique((folder / "runtime").rglob("*.ulg"))
    names = ["vehicle_global_position_groundtruth", "vehicle_local_position",
             "vehicle_attitude_groundtruth", "vehicle_local_position_groundtruth"]
    datasets = {d.name: d.data for d in ULog(str(ulog_path), message_name_filter_list=names).data_list}
    df = pd.read_csv(csv_path, low_memory=False)
    truth_audit, physical = audit_ground_truth(df, datasets[names[0]], datasets[names[1]])
    samples = df.position_time_boot_ms.to_numpy(dtype=float) * 1000.
    def interpolate(name, keys):
        times, data, _ = ordered_samples(source_window(datasets[name], samples), keys)
        result = np.column_stack([np.interp(samples, times, data[key]) for key in keys])
        result[~interpolation_coverage(samples, times)] = np.nan
        return result
    quaternion = interpolate(names[2], tuple(f"q[{i}]" for i in range(4)))
    # Interpolate unit quaternion components then normalize before computing yaw.
    quaternion /= np.linalg.norm(quaternion, axis=1)[:, None]
    w, x, y, z = quaternion.T
    yaw = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    velocity = interpolate(names[3], ("vx", "vy", "vz"))
    plan_path = folder / "plan/trajectory.csv"
    result = assess_frames(df, pd.read_csv(plan_path), physical, yaw, velocity,
                           json.loads((run / "audit.json").read_text()),
                           json.loads((folder / "final_ground.json").read_text()))
    manifest = json.loads(unique(run.glob("run_manifest_*.json")).read_text())
    plan_digest = hashlib.sha256(plan_path.read_bytes()).hexdigest()
    selected_digest = hashlib.sha256((run / "selected_trajectory.csv").read_bytes()).hexdigest()
    binding = manifest["controller_runtime"].get("trajectory_sha256") == plan_digest == selected_digest
    result["checks"].append({"name": "Executed CSV digest matches retained plan and runner manifest",
                              "passed": binding, "detail": {"sha256": plan_digest}})
    result["sampled_rehearsal_passed"] &= binding
    intent_path = folder / "plan/intent.json"
    if intent_path.is_file():
        intent = json.loads(intent_path.read_text())
        stroke = np.array([[p["north_m"], p["east_m"], p["down_m"]]
                           for p in intent["path"]["points"]], dtype=float)
        mission = df.phase.eq("TRAJECTORY").to_numpy()
        translation = next(c["detail"]["launch_translation_ned_m"] for c in result["checks"]
                           if c["name"] == "Runtime reference follows exact CSV with one translation")
        points = physical[mission] - np.asarray(translation)
        distances = np.full(len(points), np.inf)
        for left, right in zip(stroke[:-1], stroke[1:], strict=True):
            delta = right-left
            length2 = float(delta@delta)
            fraction = np.clip((points-left)@delta / length2, 0., 1.) if length2 > 0 else np.zeros(len(points))
            distances = np.minimum(distances, np.linalg.norm(points-(left+fraction[:, None]*delta), axis=1))
        stats = distribution(distances)
        fidelity = (len(points) > 0 and np.isfinite(distances).all()
                    and np.max(distances) <= intent["path"]["maximum_deviation_m"])
        result["checks"].append({"name": "Sampled physical route stays inside user's geometric tolerance",
                                 "passed": bool(fidelity), "detail": {"distance_m": stats,
                                  "limit_m": intent["path"]["maximum_deviation_m"],
                                  "scope": "point-to-polyline distance; ordering relies on separate mission-clock tracking checks, not a continuous Frechet certificate"}})
        result["sampled_rehearsal_passed"] &= bool(fidelity)
    result["independent_contact_and_hold_audit"] = truth_audit["physical_position_audit"]
    result["sampled_rehearsal_passed"] &= truth_audit["overall_passed"]
    scenario = json.loads((run / "scenario.json").read_text())
    result["scenario_expected"] = scenario.get("expected")
    # A forced interruption is judged on its declared fault response, not on
    # nominal route completion or the intentionally interrupted sampling gap.
    fault_names = {"Existing lifecycle/scenario audit", "Truth covers every recorded mission sample through contact",
                   "Touchdown truth within predeclared position/speed targets",
                   "Fresh ground/disarm/zero outputs continuously confirmed for 2 s",
                   "Measured mean real-time factor in 0.95–1.05",
                   "Executed CSV digest matches retained plan and runner manifest"}
    result["fault_response_passed"] = (scenario.get("expected") in ("navigation_response", "control_stall")
                                       and all(c["passed"] for c in result["checks"] if c["name"] in fault_names)
                                       and truth_audit["overall_passed"])
    rows = np.flatnonzero(df.phase.eq("TRAJECTORY").to_numpy() & np.isfinite(physical).all(axis=1))
    if len(rows):
        rows = rows[::max(1, int(np.ceil(len(rows)/1000)))].tolist()
        translation = next(c["detail"]["launch_translation_ned_m"] for c in result["checks"]
                           if c["name"] == "Runtime reference follows exact CSV with one translation")
        result["track_samples"] = [{"x": float(physical[i, 0]-translation[0]),
                                    "y": float(physical[i, 1]-translation[1]),
                                    "z": float(physical[i, 2]-translation[2])} for i in rows]
    result["evidence_sha256"] = {str(path.relative_to(folder)): hashlib.sha256(path.read_bytes()).hexdigest()
                                  for path in (csv_path, ulog_path, plan_path, run / "audit.json", folder / "final_ground.json",
                                               unique(run.glob("run_manifest_*.json")), run / "scenario.json")}
    import sitl_ground_truth
    result["assessment_implementation_sha256"] = {
        "planner_rehearsal.py": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "sitl_ground_truth.py": hashlib.sha256(Path(sitl_ground_truth.__file__).read_bytes()).hexdigest(),
    }
    (folder / "assessment.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case_folder", type=Path)
    args = parser.parse_args()
    print(json.dumps(assess_case(args.case_folder), indent=2, allow_nan=False))
