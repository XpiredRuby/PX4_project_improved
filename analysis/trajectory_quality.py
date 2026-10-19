"""Tracking and command smoothness from flight CSV; no battery-energy claims."""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd


def distribution(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return None
    return {"samples": int(len(values)),
            "rms": float(np.sqrt(np.mean(values**2))),
            "p95": float(np.quantile(np.abs(values), 0.95)),
            "max_abs": float(np.max(np.abs(values)))}


def trajectory_quality(df):
    time = pd.to_numeric(df["elapsed_s"], errors="coerce").to_numpy()
    dt = np.diff(time)
    if not np.all(np.isfinite(time)) or np.any(dt <= 0):
        raise ValueError("telemetry times must be finite and strictly increasing")
    phase = df["phase"].astype(str).to_numpy()
    report = {"sampling": {"rows": len(df), "max_gap_s": float(dt.max()),
                           "gaps_over_0_25_s": int(np.sum(dt > 0.25))},
              "phases": {},
              "note": "Estimate-relative errors; command effort is not battery energy."}
    for name in dict.fromkeys(phase):
        mask = phase == name
        group = df.loc[mask]
        reference_active = name not in ("PX4_LAND", "PX4_FAILSAFE")
        xy = np.hypot(group.x - group.desired_x, group.y - group.desired_y)
        z = group.z - group.desired_z
        record = {"offboard_reference_active": reference_active,
                  "xy_error_m": distribution(xy) if reference_active else None,
                  "z_error_m": distribution(z) if reference_active else None,
                  "duration_s": float(time[mask][-1] - time[mask][0])}
        if "yaw_error" in group:
            record["yaw_error_deg"] = (distribution(np.degrees(group.yaw_error))
                                       if reference_active else None)
        pair = mask[1:] & mask[:-1] & (dt <= 0.25)
        actual = df[["x", "y", "z"]].to_numpy(dtype=float)
        segments = np.linalg.norm(np.diff(actual, axis=0), axis=1)
        record["estimated_path_length_m"] = float(np.nansum(segments[pair]))
        if not reference_active:
            for metric in ("command_xy_accel_m_s2", "command_z_accel_m_s2",
                           "command_xy_jerk_m_s3", "command_z_jerk_m_s3",
                           "command_acceleration_effort_m2_s3"):
                record[metric] = None
            report["phases"][name] = record
            continue
        command = df[["cmd_vx", "cmd_vy", "cmd_vz"]].to_numpy(dtype=float)
        accel = np.diff(command, axis=0) / dt[:, None]
        accel[~pair] = np.nan
        record["command_xy_accel_m_s2"] = distribution(np.linalg.norm(accel[:, :2], axis=1))
        record["command_z_accel_m_s2"] = distribution(accel[:, 2])
        # Acceleration samples lie at interval midpoints, including unequal dt.
        midpoints = (time[:-1] + time[1:]) / 2
        jerk = np.diff(accel, axis=0) / np.diff(midpoints)[:, None]
        record["command_xy_jerk_m_s3"] = distribution(np.linalg.norm(jerk[:, :2], axis=1))
        record["command_z_jerk_m_s3"] = distribution(jerk[:, 2])
        valid = pair & np.all(np.isfinite(accel), axis=1)
        record["command_acceleration_effort_m2_s3"] = float(
            np.sum(np.sum(accel[valid]**2, axis=1) * dt[valid]))
        report["phases"][name] = record
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = trajectory_quality(pd.read_csv(args.csv))
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
