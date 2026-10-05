"""Offline feasibility of an IMU bridge; this code never sends flight commands."""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

def checked_samples(times, acceleration):
    times = np.asarray(times, dtype=float)
    acceleration = np.asarray(acceleration, dtype=float)
    if (times.ndim != 1 or len(times) < 2 or acceleration.shape != (len(times), 3)
            or not np.isfinite(times).all() or not np.isfinite(acceleration).all()
            or np.any(np.diff(times) <= 0) or np.any(np.diff(times) > .15)):
        raise ValueError("IMU replay needs finite, ordered samples with gaps at most 0.15s")
    return times, acceleration


def estimate_bias(times, acceleration, start_velocity, end_velocity):
    """Freeze a NED acceleration bias using only the preceding healthy window."""
    times, acceleration = checked_samples(times, acceleration)
    duration = times[-1] - times[0]
    endpoints = np.asarray([start_velocity, end_velocity], dtype=float)
    if duration < 2. or endpoints.shape != (2, 3) or not np.isfinite(endpoints).all():
        raise ValueError("Bias calibration needs two seconds and finite endpoint velocities")
    integrated = np.sum((acceleration[:-1] + acceleration[1:]) * .5
                        * np.diff(times)[:, None], axis=0)
    return (integrated - (endpoints[1] - endpoints[0])) / duration


def propagate(times, acceleration, position, velocity, bias):
    """Integrate linearly interpolated acceleration on its actual sample clock."""
    times, acceleration = checked_samples(times, acceleration)
    initial = np.asarray([position, velocity, bias], dtype=float)
    if initial.shape != (3, 3) or not np.isfinite(initial).all():
        raise ValueError("Initial inertial state and bias must be finite XYZ vectors")
    position, velocity, bias = initial.copy()
    positions = [position.copy()]
    velocities = [velocity.copy()]
    for index, dt in enumerate(np.diff(times), 1):
        a0, a1 = acceleration[index - 1] - bias, acceleration[index] - bias
        position = position + velocity * dt + (a0 / 3. + a1 / 6.) * dt**2
        velocity = velocity + (a0 + a1) * .5 * dt
        positions.append(position.copy())
        velocities.append(velocity.copy())
    return np.asarray(positions), np.asarray(velocities)


def replay_outages(df, physical):
    """Truth is used only after propagation, to score each diagnostic replay."""
    from analyze_run import body_specific_force_to_ned

    times = df.elapsed_s.to_numpy(dtype=float)
    acceleration = np.column_stack(body_specific_force_to_ned(*[
        df[key] for key in ("roll", "pitch", "yaw", "imu_xacc", "imu_yacc", "imu_zacc")]))
    bad = df.gps_fix_type.lt(3).to_numpy()
    records = []
    starts = np.flatnonzero(bad & ~np.r_[False, bad[:-1]])
    for first_bad in starts:
        if first_bad == 0:
            raise ValueError("Outage has no preceding healthy state")
        start = first_bad - 1
        recovered = np.flatnonzero(~bad[first_bad:])
        if not len(recovered):
            raise ValueError("Outage has no recorded GPS restoration")
        end = first_bad + recovered[0]
        calibration = np.flatnonzero((times >= times[start] - 4.) & (times <= times[start]))
        if bad[calibration].any():
            raise ValueError("Calibration overlaps an earlier GPS outage")
        ages = df.loc[calibration[0]:end, "imu_age_s"].to_numpy(dtype=float)
        if not np.isfinite(ages).all() or np.any(ages < 0) or np.any(ages > .1):
            raise ValueError("IMU replay has stale or missing inertial telemetry")
        velocities = df[["vx", "vy", "vz"]].to_numpy(dtype=float)
        bias = estimate_bias(times[calibration], acceleration[calibration],
                             velocities[calibration[0]], velocities[start])
        start_position = df.loc[start, ["x", "y", "z"]].to_numpy(dtype=float)
        interval = slice(start, end + 1)
        predicted, _ = propagate(times[interval], acceleration[interval],
                                 start_position, velocities[start], bias)
        actual = np.asarray(physical[interval], dtype=float)
        if actual.shape != predicted.shape or not np.isfinite(actual).all():
            raise ValueError("Independent truth does not cover the outage")
        xy = np.linalg.norm(predicted[:, :2] - actual[:, :2], axis=1)
        z = np.abs(predicted[:, 2] - actual[:, 2])
        duration = float(times[end] - times[start])
        records.append({"start_row": int(start), "end_row": int(end), "duration_s": duration,
                        "calibration_bias_ned_m_s2": bias.tolist(),
                        "maximum_xy_error_m": float(xy.max()), "maximum_z_error_m": float(z.max()),
                        "terminal_xy_error_m": float(xy[-1]),
                        "additional_error_budget_m": {
                            "velocity_0_05_m_s_and_bias_0_01_m_s2": .05 * duration + .005 * duration**2,
                            "velocity_0_10_m_s_and_bias_0_03_m_s2": .1 * duration + .015 * duration**2}})
    return {"outages": records, "flight_control_validated": False,
            "limitations": ["Asynchronous telemetry, not time-synchronized raw IMU mechanization",
                            "NED bias frozen before outage; no calibrated aircraft noise model",
                            "Open-loop replay does not establish closed-loop stability or containment",
                            "Sensitivity budgets exclude initial position and attitude uncertainty"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    logs = list(args.run_dir.glob("research_log_*.csv"))
    if len(logs) != 1:
        raise ValueError("Expected one controller CSV")
    df = pd.read_csv(logs[0], low_memory=False)
    truth = pd.read_csv(args.run_dir / "aligned_ground_truth.csv.gz")
    result = replay_outages(df, truth[["truth_x", "truth_y", "truth_z"]].to_numpy())
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
