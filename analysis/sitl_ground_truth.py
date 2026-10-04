"""Independent simulator truth audit. Truth data never enter the flight controller."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import pandas as pd

from trajectory_quality import distribution

EARTH_RADIUS_M = 6371000.0


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


def audit_ground_truth(df, truth, local):
    sample_us = df.position_time_boot_ms.to_numpy(dtype=float) * 1000.
    truth_times, truth, truth_duplicates = ordered_samples(truth, ("lat", "lon", "alt"))
    local_times, local, local_duplicates = ordered_samples(local, ("ref_lat", "ref_lon", "ref_alt"))
    if not np.all(np.isfinite(sample_us)):
        raise ValueError("Truth and estimator sample times must be ordered and finite")
    covered = ((sample_us >= max(truth_times[0], local_times[0]))
               & (sample_us <= min(truth_times[-1], local_times[-1])))
    # Do not extrapolate truth beyond a recorded flight.
    def values(data, times, key):
        return np.interp(sample_us, times, data[key])

    physical = geodetic_to_ned(
        values(truth, truth_times, "lat"), values(truth, truth_times, "lon"),
        values(truth, truth_times, "alt"), values(local, local_times, "ref_lat"),
        values(local, local_times, "ref_lon"), values(local, local_times, "ref_alt"),
    )
    physical[~covered] = np.nan
    desired = df[["desired_x", "desired_y", "desired_z"]].to_numpy(dtype=float)
    estimated = df[["x", "y", "z"]].to_numpy(dtype=float)
    physical_error = physical - desired
    estimate_error = physical - estimated
    physical_error[~covered] = np.nan
    estimate_error[~covered] = np.nan
    output = {"coverage": {"rows": len(df), "covered_rows": int(covered.sum())},
              "alignment": "CSV PX4 position clock; truth projected using contemporaneous EKF local origin",
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
    df = pd.read_csv(logs[0])
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
