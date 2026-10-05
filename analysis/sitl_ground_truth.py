"""Independent simulator truth audit. Truth data never enter the flight controller."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from trajectory_quality import distribution

EARTH_RADIUS_M = 6371000.0
MAX_ALIGNMENT_GAP_US = 250_000.


def ordered_samples(data, keys):
    """Collapse identical duplicate samples; reject regressed/ambiguous clocks."""
    times = np.asarray(data["timestamp"], dtype=float)
    if (times.ndim != 1 or not len(times) or not np.all(np.isfinite(times))
            or np.any(np.diff(times) < 0)):
        raise ValueError("Truth and estimator sample times must be ordered and finite")
    duplicate = np.diff(times) == 0
    columns = {}
    for key in keys:
        values = np.asarray(data.get(key, []), dtype=float)
        if values.shape != times.shape or not np.all(np.isfinite(values)):
            raise ValueError(f"{key} must contain one finite value per timestamp")
        if np.any(duplicate & (values[1:] != values[:-1])):
            raise ValueError("Duplicate source timestamps have conflicting values")
        columns[key] = values
    keep = np.r_[True, ~duplicate]
    return times[keep], {k: v[keep] for k, v in columns.items()}, int(duplicate.sum())


def geodetic_to_ned(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """Local tangent approximation for these sub-100 m SITL routes."""
    north = EARTH_RADIUS_M * np.radians(np.asarray(lat) - ref_lat)
    east = EARTH_RADIUS_M * np.cos(np.radians(ref_lat)) * np.radians(np.asarray(lon) - ref_lon)
    down = ref_alt - np.asarray(alt)
    return np.column_stack((north, east, down))


def interpolation_coverage(samples, times, max_gap_us=MAX_ALIGNMENT_GAP_US):
    """Accept exact observations or short interpolation brackets only."""
    right = np.searchsorted(times, samples, side="left")
    clipped = np.minimum(right, len(times) - 1)
    exact = (right < len(times)) & (times[clipped] == samples)
    left = np.maximum(right - 1, 0)
    bracketed = ((right > 0) & (right < len(times))
                 & (times[clipped] - times[left] <= max_gap_us))
    return exact | bracketed


def audit_physical_position(df, physical):
    """Apply position limits to independent truth, never to EKF position."""
    checks = []

    def add(name, passed, detail):
        checks.append({"name": name, "passed": bool(passed), "detail": str(detail)})

    required = {"phase", "landed_state", "navigation_state",
                "native_landing_reference_x", "native_landing_reference_y"}
    missing = required - set(df.columns)
    if missing:
        add("Physical position evidence complete", False, "missing: " + ", ".join(sorted(missing)))
        return {"overall_passed": False, "checks": checks}
    physical = np.asarray(physical, dtype=float)
    if physical.shape != (len(df), 3):
        raise ValueError("Physical truth must have one XYZ row per telemetry sample")
    native = df.phase.isin(["PX4_LAND", "PX4_FAILSAFE"]).to_numpy()
    contacts = native & pd.to_numeric(df.landed_state, errors="coerce").eq(1).to_numpy()
    indices = np.flatnonzero(contacts)
    add("Physical contact recorded", len(indices) > 0, f"contact samples={len(indices)}")
    if len(indices):
        contact = indices[0]
        references = df[["native_landing_reference_x", "native_landing_reference_y"]].apply(
            pd.to_numeric, errors="coerce").to_numpy(dtype=float)
        native_references = references[native & (np.arange(len(df)) <= contact)]
        frozen = (len(native_references) > 0 and np.isfinite(native_references).all()
                  and np.allclose(native_references, native_references[0], rtol=0, atol=1e-6))
        add("Landing reference frozen before contact", frozen,
            f"native reference samples={len(native_references)}")
        covered = np.isfinite(physical[contact]).all()
        add("Physical contact truth covered", covered, f"contact row={contact}")
        error = (float(np.linalg.norm(physical[contact, :2] - native_references[0]))
                 if frozen and covered else None)
        add("Physical touchdown position within limit", error is not None and error <= 1.5,
            f"error={error!r}m, limit=1.5m")
    hold = df.navigation_state.eq("HOLD").to_numpy() & ~native
    starts = np.flatnonzero(hold & ~np.r_[False, hold[:-1]])
    for number, start in enumerate(starts, 1):
        later = np.flatnonzero(~hold[start:])
        end = start + later[0] if len(later) else len(df)
        positions = physical[start:end]
        covered = np.isfinite(positions).all()
        xy = float(np.linalg.norm(positions[:, :2] - positions[0, :2], axis=1).max()) if covered else None
        z = float(np.abs(positions[:, 2] - positions[0, 2]).max()) if covered else None
        add(f"Physical HOLD {number} remains bounded", covered and xy <= 1. and z <= 1.,
            f"rows={start}:{end}, xy={xy!r}m, z={z!r}m, limits=1m")
    return {"overall_passed": all(item["passed"] for item in checks), "checks": checks,
            "scope": "Independent contact XY and HOLD XYZ only; mission, dynamics and shutdown audits remain required."}


def audit_ground_truth(df, truth, local):
    sample_us = df.position_time_boot_ms.to_numpy(dtype=float) * 1000.
    truth_times, truth, truth_duplicates = ordered_samples(truth, ("lat", "lon", "alt"))
    local_times, local, local_duplicates = ordered_samples(local, ("ref_lat", "ref_lon", "ref_alt"))
    if not np.all(np.isfinite(sample_us)):
        raise ValueError("Truth and estimator sample times must be ordered and finite")
    covered = (interpolation_coverage(sample_us, truth_times)
               & interpolation_coverage(sample_us, local_times))
    # Do not extrapolate truth beyond a recorded flight.
    def values(data, times, key):
        return np.interp(sample_us, times, data[key])

    # An EKF origin is a discrete coordinate frame. Interpolating a reset
    # would invent intermediate frames that never existed in the estimator.
    reference_index = np.clip(np.searchsorted(local_times, sample_us, side="right") - 1,
                              0, len(local_times) - 1)
    physical = geodetic_to_ned(
        values(truth, truth_times, "lat"), values(truth, truth_times, "lon"),
        values(truth, truth_times, "alt"), local["ref_lat"][reference_index],
        local["ref_lon"][reference_index], local["ref_alt"][reference_index],
    )
    physical[~covered] = np.nan
    desired = df[["desired_x", "desired_y", "desired_z"]].to_numpy(dtype=float)
    estimated = df[["x", "y", "z"]].to_numpy(dtype=float)
    physical_error = physical - desired
    estimate_error = physical - estimated
    physical_error[~covered] = np.nan
    estimate_error[~covered] = np.nan
    output = {"coverage": {"rows": len(df), "covered_rows": int(covered.sum())},
              "alignment": "CSV PX4 position clock; truth projected using last reported EKF local origin",
              "maximum_interpolation_gap_s": MAX_ALIGNMENT_GAP_US / 1e6,
              "identical_source_duplicates_removed": {"truth": truth_duplicates, "local_reference": local_duplicates},
              "coordinate_approximation": "Local tangent plane, Earth radius 6371000 m; routes below 100 m",
              "phases": {}}
    for phase in dict.fromkeys(df.phase):
        mask = df.phase.to_numpy() == phase
        reference_active = phase not in ("PX4_LAND", "PX4_FAILSAFE")
        output["phases"][str(phase)] = {
            "offboard_reference_active": reference_active,
            "physical_xy_tracking_error_m": (distribution(np.linalg.norm(physical_error[mask, :2], axis=1))
                                             if reference_active else None),
            "physical_z_tracking_error_m": (distribution(physical_error[mask, 2])
                                            if reference_active else None),
            "estimator_xy_error_m": distribution(np.linalg.norm(estimate_error[mask, :2], axis=1)),
            "estimator_z_error_m": distribution(estimate_error[mask, 2]),
        }
    output["physical_position_audit"] = audit_physical_position(df, physical)
    output["overall_passed"] = output["physical_position_audit"]["overall_passed"]
    return output, physical


def main():
    from pyulog import ULog

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--ulog", type=Path, required=True)
    args = parser.parse_args()
    logs = list(args.run_dir.glob("research_log_*.csv"))
    if len(logs) != 1:
        raise ValueError("Expected one controller CSV")
    ulog = ULog(str(args.ulog), message_name_filter_list=[
        "vehicle_global_position_groundtruth", "vehicle_local_position"])
    datasets = {data.name: data.data for data in ulog.data_list}
    df = pd.read_csv(logs[0], low_memory=False)
    result, physical = audit_ground_truth(df, datasets["vehicle_global_position_groundtruth"],
                                         datasets["vehicle_local_position"])
    digest = hashlib.sha256()
    with args.ulog.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    result["ulog"] = {"name": args.ulog.name, "sha256": digest.hexdigest()}
    (args.run_dir / "ground_truth_audit.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    truth_frame = pd.DataFrame(physical, columns=["truth_x", "truth_y", "truth_z"])
    truth_frame.insert(0, "position_time_boot_ms", df.position_time_boot_ms)
    truth_frame.to_csv(args.run_dir / "aligned_ground_truth.csv.gz", index=False)


if __name__ == "__main__":
    main()
