#!/usr/bin/env python3
"""Analyze one instrumented Vishnu/PX4 research archive."""

import argparse
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PHASE_COLORS = {
    "TAKEOFF": "#dbeafe",
    "TRAJECTORY": "#dcfce7",
    "LAND": "#fef3c7",
    "DONE": "#e5e7eb",
}


def finite(series):
    return pd.to_numeric(series, errors="coerce").replace(
        [np.inf, -np.inf], np.nan
    )


def wrapped_error(desired, actual):
    return np.arctan2(
        np.sin(desired - actual),
        np.cos(desired - actual),
    )


def quaternion_to_euler(q0, q1, q2, q3):
    quaternion = np.column_stack(
        [np.asarray(value, dtype=float) for value in (q0, q1, q2, q3)]
    )
    norm = np.linalg.norm(quaternion, axis=1)
    valid = np.all(np.isfinite(quaternion), axis=1) & (norm > 1e-12)
    normalized = np.full_like(quaternion, np.nan, dtype=float)
    normalized[valid] = quaternion[valid] / norm[valid, None]
    w, x, y, z = normalized.T
    roll = np.arctan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch = np.arcsin(
        np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    )
    yaw = np.arctan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return roll, pitch, yaw


def body_specific_force_to_ned(roll, pitch, yaw, fx, fy, fz):
    roll = np.asarray(roll, dtype=float)
    pitch = np.asarray(pitch, dtype=float)
    yaw = np.asarray(yaw, dtype=float)
    fx = np.asarray(fx, dtype=float)
    fy = np.asarray(fy, dtype=float)
    fz = np.asarray(fz, dtype=float)

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    ax = (
        cy * cp * fx
        + (cy * sp * sr - sy * cr) * fy
        + (cy * sp * cr + sy * sr) * fz
    )
    ay = (
        sy * cp * fx
        + (sy * sp * sr + cy * cr) * fy
        + (sy * sp * cr - cy * sr) * fz
    )
    az = -sp * fx + cp * sr * fy + cp * cr * fz + 9.80665
    return ax, ay, az


def enrich_derived_signals(df):
    derived = {}
    quaternion_columns = [f"attitude_target_q{index}" for index in range(4)]
    if all(has_signal(df, key) for key in quaternion_columns):
        roll, pitch, yaw = quaternion_to_euler(
            *(finite(df[key]) for key in quaternion_columns)
        )
        derived.update(
            px4_target_roll=roll,
            px4_target_pitch=pitch,
            px4_target_yaw_from_q=yaw,
        )

    acceleration_columns = [
        "roll", "pitch", "yaw", "imu_xacc", "imu_yacc", "imu_zacc"
    ]
    if all(has_signal(df, key) for key in acceleration_columns):
        ax, ay, az = body_specific_force_to_ned(
            *(finite(df[key]) for key in acceleration_columns)
        )
        for axis, values in zip("xyz", (ax, ay, az)):
            derived[f"derived_actual_a{axis}_ned_raw"] = values
            derived[f"derived_actual_a{axis}_ned_filtered"] = (
                pd.Series(values, index=df.index)
                .rolling(window=5, center=True, min_periods=1)
                .mean()
            )
    if derived:
        df[list(derived)] = pd.DataFrame(derived, index=df.index)


def metric_record(name, desired, actual, units, wrap=False):
    desired = np.asarray(desired, dtype=float)
    actual = np.asarray(actual, dtype=float)
    mask = np.isfinite(desired) & np.isfinite(actual)
    desired = desired[mask]
    actual = actual[mask]
    if desired.size == 0:
        return None

    error = (
        wrapped_error(desired, actual)
        if wrap
        else desired - actual
    )
    absolute = np.abs(error)
    return {
        "signal": name,
        "units": units,
        "samples": int(error.size),
        "rmse": float(np.sqrt(np.mean(error ** 2))),
        "mae": float(np.mean(absolute)),
        "max_abs_error": float(np.max(absolute)),
        "bias": float(np.mean(error)),
        "p95_abs_error": float(np.quantile(absolute, 0.95)),
    }


def best_lag(desired, actual, sample_period, max_seconds=2.0):
    desired = np.asarray(desired, dtype=float)
    actual = np.asarray(actual, dtype=float)
    max_samples = max(1, int(round(max_seconds / sample_period)))
    best = None

    for lag in range(-max_samples, max_samples + 1):
        if lag > 0:
            x = desired[:-lag]
            y = actual[lag:]
        elif lag < 0:
            x = desired[-lag:]
            y = actual[:lag]
        else:
            x = desired
            y = actual

        mask = np.isfinite(x) & np.isfinite(y)
        x = x[mask]
        y = y[mask]
        if x.size < 10 or np.std(x) < 1e-9 or np.std(y) < 1e-9:
            continue

        correlation = float(np.corrcoef(x, y)[0, 1])
        if best is None or correlation > best["correlation"]:
            best = {
                "lag_samples": int(lag),
                "lag_seconds": float(lag * sample_period),
                "correlation": correlation,
            }

    return best


def phase_intervals(df):
    intervals = []
    start = 0
    phases = df["phase"].astype(str).to_numpy()
    time_values = df["elapsed_s"].to_numpy(dtype=float)

    for index in range(1, len(df)):
        if phases[index] != phases[index - 1]:
            intervals.append(
                (phases[start], time_values[start], time_values[index - 1])
            )
            start = index

    intervals.append((phases[start], time_values[start], time_values[-1]))
    return intervals


def shade_phases(axis, intervals):
    for phase, start, end in intervals:
        axis.axvspan(
            start,
            end,
            color=PHASE_COLORS.get(phase, "#f3f4f6"),
            alpha=0.20,
            linewidth=0,
        )


def save_figure(fig, path):
    fig.tight_layout()
    fig.savefig(path, dpi=180, bbox_inches="tight")
    plt.close(fig)


def plot_position(df, output, intervals):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for axis, key in zip(axes, ("x", "y", "z")):
        axis.plot(
            df["elapsed_s"],
            df[f"desired_{key}"],
            label="Desired",
            linewidth=1.8,
        )
        axis.plot(
            df["elapsed_s"],
            df[key],
            label="Actual",
            linewidth=1.2,
        )
        shade_phases(axis, intervals)
        axis.set_ylabel(f"{key.upper()} (m)")
        axis.grid(True, alpha=0.3)
    axes[0].legend(ncol=2)
    axes[-1].set_xlabel("Elapsed time (s)")
    fig.suptitle("Desired vs actual position")
    save_figure(fig, output / "01_position_tracking.png")


def plot_velocity(df, output, intervals):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for axis, key in zip(axes, ("x", "y", "z")):
        axis.plot(
            df["elapsed_s"],
            df[f"planned_v{key}"],
            label="Trajectory feedforward",
            linewidth=1.4,
        )
        axis.plot(
            df["elapsed_s"],
            df[f"cmd_v{key}"],
            label="Final outer-loop command",
            linewidth=1.4,
        )
        axis.plot(
            df["elapsed_s"],
            df[f"v{key}"],
            label="PX4 actual velocity",
            linewidth=1.1,
        )
        shade_phases(axis, intervals)
        axis.set_ylabel(f"V{key.upper()} (m/s)")
        axis.grid(True, alpha=0.3)
    axes[0].legend(ncol=3)
    axes[-1].set_xlabel("Elapsed time (s)")
    fig.suptitle("Feedforward, final command, and actual velocity")
    save_figure(fig, output / "02_velocity_tracking.png")


def plot_outer_terms(df, output, intervals):
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for axis, key in zip(axes, ("x", "y", "z")):
        axis.plot(
            df["elapsed_s"],
            df[f"planned_v{key}"],
            label="Feedforward",
            linewidth=1.3,
        )
        axis.plot(
            df["elapsed_s"],
            df[f"pid_{key}_correction"],
            label="PID correction",
            linewidth=1.3,
        )
        axis.plot(
            df["elapsed_s"],
            df[f"cmd_v{key}"],
            label="Final command",
            linewidth=1.4,
        )
        shade_phases(axis, intervals)
        axis.set_ylabel(f"V{key.upper()} (m/s)")
        axis.grid(True, alpha=0.3)
    axes[0].legend(ncol=3)
    axes[-1].set_xlabel("Elapsed time (s)")
    fig.suptitle("Vishnu outer-loop command decomposition")
    save_figure(fig, output / "03_outer_loop_terms.png")


def plot_yaw(df, output, intervals):
    desired = np.unwrap(df["target_yaw"].to_numpy(dtype=float))
    actual = np.unwrap(df["yaw"].to_numpy(dtype=float))
    error = wrapped_error(
        df["target_yaw"].to_numpy(dtype=float),
        df["yaw"].to_numpy(dtype=float),
    )

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    axes[0].plot(
        df["elapsed_s"], np.degrees(desired), label="Desired", linewidth=1.6
    )
    axes[0].plot(
        df["elapsed_s"], np.degrees(actual), label="Actual", linewidth=1.2
    )
    axes[0].set_ylabel("Unwrapped yaw (deg)")
    axes[0].legend(ncol=2)
    axes[1].plot(
        df["elapsed_s"], np.degrees(error), color="#dc2626", linewidth=1.2
    )
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set_ylabel("Wrapped error (deg)")
    axes[1].set_xlabel("Elapsed time (s)")
    for axis in axes:
        shade_phases(axis, intervals)
        axis.grid(True, alpha=0.3)
    fig.suptitle("Yaw tracking without ±180° plotting artifacts")
    save_figure(fig, output / "04_yaw_tracking.png")


def plot_errors(df, output, intervals):
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    for axis, key in zip(axes[:3], ("x", "y", "z")):
        error = df[f"desired_{key}"] - df[key]
        axis.plot(df["elapsed_s"], error, linewidth=1.1)
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.set_ylabel(f"{key.upper()} error (m)")
        shade_phases(axis, intervals)
        axis.grid(True, alpha=0.3)

    yaw_error = wrapped_error(
        df["target_yaw"].to_numpy(dtype=float),
        df["yaw"].to_numpy(dtype=float),
    )
    axes[3].plot(
        df["elapsed_s"], np.degrees(yaw_error), linewidth=1.1
    )
    axes[3].axhline(0.0, color="black", linewidth=0.8)
    axes[3].set_ylabel("Yaw error (deg)")
    axes[3].set_xlabel("Elapsed time (s)")
    shade_phases(axes[3], intervals)
    axes[3].grid(True, alpha=0.3)
    fig.suptitle("Tracking errors")
    save_figure(fig, output / "05_tracking_errors.png")


def plot_attitude(df, output, intervals):
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for key, label in (
        ("roll", "Roll"),
        ("pitch", "Pitch"),
        ("yaw", "Yaw"),
    ):
        axes[0].plot(
            df["elapsed_s"],
            np.degrees(df[key]),
            label=label,
            linewidth=1.0,
        )
    for key, label in (("p", "p"), ("q", "q"), ("r", "r")):
        axes[1].plot(
            df["elapsed_s"],
            np.degrees(df[key]),
            label=label,
            linewidth=1.0,
        )

    axes[0].set_ylabel("Attitude (deg)")
    axes[1].set_ylabel("Body rate (deg/s)")
    axes[1].set_xlabel("Elapsed time (s)")
    for axis in axes:
        shade_phases(axis, intervals)
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=3)
    fig.suptitle("Actual attitude and angular rates")
    save_figure(fig, output / "06_attitude_rates.png")


def plot_timing(df, output, intervals):
    has_setpoint_timing = any(
        has_signal(df, key)
        for key in (
            "setpoint_last_gap_s",
            "setpoint_send_age_s",
            "latest_setpoint_age_s",
        )
    )
    row_count = 4 if has_setpoint_timing else 3
    fig, axes = plt.subplots(
        row_count,
        1,
        figsize=(12, 3 * row_count),
        sharex=True,
    )
    axes[0].plot(
        df["elapsed_s"],
        1000.0 * df["loop_dt_s"],
        label="Loop interval",
        linewidth=1.0,
    )
    axes[0].axhline(50.0, color="#dc2626", linestyle="--", label="50 ms target")
    axes[0].set_ylabel("Loop interval (ms)")
    axes[0].legend(ncol=2)

    for key, label in (
        ("position_age_s", "Position"),
        ("attitude_age_s", "Attitude"),
        ("heartbeat_age_s", "Heartbeat"),
    ):
        if key in df:
            axes[1].plot(
                df["elapsed_s"],
                1000.0 * finite(df[key]),
                label=label,
                linewidth=1.0,
            )
    axes[1].set_ylabel("Core age (ms)")
    axes[1].legend(ncol=3)

    inner_axis = axes[3] if has_setpoint_timing else axes[2]
    if has_setpoint_timing:
        for key, label in (
            ("setpoint_last_gap_s", "Last send gap"),
            ("setpoint_send_age_s", "Send age at log"),
            ("latest_setpoint_age_s", "Command age at log"),
        ):
            if has_signal(df, key):
                axes[2].plot(
                    df["elapsed_s"],
                    1000.0 * finite(df[key]),
                    label=label,
                    linewidth=1.0,
                )
        axes[2].axhline(
            100.0,
            color="#dc2626",
            linestyle="--",
            label="100 ms watchdog",
        )
        axes[2].set_ylabel("Setpoint timing (ms)")
        axes[2].legend(ncol=4)

    for key, label in (
        ("imu_age_s", "IMU"),
        ("px4_position_target_age_s", "PX4 velocity target"),
        ("px4_attitude_target_age_s", "PX4 attitude target"),
        ("actuator_age_s", "Actuator output"),
        ("servo_age_s", "Servo raw"),
    ):
        if key in df:
            inner_axis.plot(
                df["elapsed_s"],
                1000.0 * finite(df[key]),
                label=label,
                linewidth=1.0,
            )
    inner_axis.set_ylabel("Inner-loop age (ms)")
    inner_axis.set_xlabel("Elapsed time (s)")
    handles, _ = inner_axis.get_legend_handles_labels()
    if handles:
        inner_axis.legend(ncol=3)

    for axis in axes:
        shade_phases(axis, intervals)
        axis.grid(True, alpha=0.3)
    fig.suptitle("Controller timing and telemetry freshness")
    save_figure(fig, output / "07_timing_telemetry.png")


def plot_path(df, output):
    trajectory = df[df["phase"] == "TRAJECTORY"]
    fig = plt.figure(figsize=(10, 8))
    axis = fig.add_subplot(111, projection="3d")
    axis.plot(
        trajectory["desired_x"],
        trajectory["desired_y"],
        trajectory["desired_z"],
        label="Desired",
        linewidth=1.8,
    )
    axis.plot(
        trajectory["x"],
        trajectory["y"],
        trajectory["z"],
        label="Actual",
        linewidth=1.2,
    )
    axis.scatter(
        trajectory["x"].iloc[0],
        trajectory["y"].iloc[0],
        trajectory["z"].iloc[0],
        label="Trajectory start",
        s=35,
    )
    axis.scatter(
        trajectory["x"].iloc[-1],
        trajectory["y"].iloc[-1],
        trajectory["z"].iloc[-1],
        label="Trajectory end",
        marker="x",
        s=45,
    )
    axis.set_xlabel("North X (m)")
    axis.set_ylabel("East Y (m)")
    axis.set_zlabel("Down Z (m)")
    axis.set_title("3D trajectory tracking in local NED")
    axis.legend()
    save_figure(fig, output / "08_path_3d.png")


def has_signal(df, key):
    return key in df and np.isfinite(finite(df[key])).any()


def plot_inner_signals(df, output, intervals):
    actuator_prefix = (
        "actuator_output_"
        if has_signal(df, "actuator_output_0")
        else "actuator_norm_"
    )
    available = any(
        has_signal(df, key)
        for key in (
            "px4_target_vx",
            "attitude_target_roll_rate",
            "attitude_target_thrust",
            f"{actuator_prefix}0",
            "servo_raw_1",
        )
    )
    if not available:
        return False

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

    for key, label in (
        ("px4_target_vx", "PX4 target VX"),
        ("px4_target_vy", "PX4 target VY"),
        ("px4_target_vz", "PX4 target VZ"),
    ):
        if has_signal(df, key):
            axes[0].plot(df["elapsed_s"], finite(df[key]), label=label)
    for key, label in (("vx", "Actual VX"), ("vy", "Actual VY"), ("vz", "Actual VZ")):
        if has_signal(df, key):
            axes[0].plot(
                df["elapsed_s"], finite(df[key]), label=label,
                linewidth=1.0, linestyle="--",
            )
    axes[0].set_ylabel("Velocity target (m/s)")

    for key, label in (
        ("attitude_target_roll_rate", "Target p"),
        ("attitude_target_pitch_rate", "Target q"),
        ("attitude_target_yaw_rate", "Target r"),
    ):
        if has_signal(df, key):
            axes[1].plot(
                df["elapsed_s"],
                np.degrees(finite(df[key])),
                label=label,
            )
    for key, label in (("p", "Actual p"), ("q", "Actual q"), ("r", "Actual r")):
        if has_signal(df, key):
            axes[1].plot(
                df["elapsed_s"], np.degrees(finite(df[key])),
                label=label, linewidth=1.0, linestyle="--",
            )
    axes[1].set_ylabel("Rate target (deg/s)")

    for index in range(4):
        key = f"{actuator_prefix}{index}"
        if has_signal(df, key):
            axes[2].plot(
                df["elapsed_s"],
                finite(df[key]),
                label=f"Actuator output {index}",
            )
    axes[2].set_ylabel("Natural output units")

    for index in range(1, 5):
        key = f"servo_raw_{index}"
        if has_signal(df, key):
            axes[3].plot(
                df["elapsed_s"],
                finite(df[key]),
                label=f"Servo raw {index}",
            )
    axes[3].set_ylabel("Reported raw units")
    axes[3].set_xlabel("Elapsed time (s)")

    for axis in axes:
        shade_phases(axis, intervals)
        axis.grid(True, alpha=0.3)
        handles, _ = axis.get_legend_handles_labels()
        if handles:
            axis.legend(ncol=4)
    fig.suptitle("Available PX4 inner-loop and actuator signals")
    save_figure(fig, output / "09_inner_loop_signals.png")
    return True


def plot_outer_inner_chain(df, output, intervals):
    target_attitude = [
        "px4_target_roll", "px4_target_pitch", "px4_target_yaw_from_q"
    ]
    target_rates = [
        "attitude_target_roll_rate",
        "attitude_target_pitch_rate",
        "attitude_target_yaw_rate",
    ]
    if not all(has_signal(df, key) for key in target_attitude + target_rates):
        return False

    fig, axes = plt.subplots(4, 3, figsize=(17, 13), sharex=True)
    position_names = ("x", "y", "z")
    attitude_names = ("roll", "pitch", "yaw")
    actual_rates = ("p", "q", "r")
    column_titles = ("North X / Roll / p", "East Y / Pitch / q", "Down Z / Yaw / r")

    for column, (axis_name, attitude_name, rate_name) in enumerate(
        zip(position_names, attitude_names, actual_rates)
    ):
        axes[0, column].plot(
            df["elapsed_s"], df[f"desired_{axis_name}"],
            label="Desired position", linewidth=1.5,
        )
        axes[0, column].plot(
            df["elapsed_s"], df[axis_name],
            label="Actual position", linewidth=1.0,
        )

        axes[1, column].plot(
            df["elapsed_s"], df[f"planned_v{axis_name}"],
            label="Feedforward", linewidth=1.1,
        )
        axes[1, column].plot(
            df["elapsed_s"], df[f"cmd_v{axis_name}"],
            label="Outer command", linewidth=1.3,
        )
        axes[1, column].plot(
            df["elapsed_s"], df[f"v{axis_name}"],
            label="Actual velocity", linewidth=1.0,
        )

        target_attitude_values = finite(df[f"px4_target_{attitude_name}" if attitude_name != "yaw" else "px4_target_yaw_from_q"])
        actual_attitude_values = finite(df[attitude_name])
        if attitude_name == "yaw":
            target_attitude_values = np.unwrap(target_attitude_values.to_numpy(dtype=float))
            actual_attitude_values = np.unwrap(actual_attitude_values.to_numpy(dtype=float))
        axes[2, column].plot(
            df["elapsed_s"], np.degrees(target_attitude_values),
            label="PX4 attitude target", linewidth=1.3,
        )
        axes[2, column].plot(
            df["elapsed_s"], np.degrees(actual_attitude_values),
            label="Actual attitude", linewidth=1.0,
        )

        target_rate = target_rates[column]
        axes[3, column].plot(
            df["elapsed_s"], np.degrees(finite(df[target_rate])),
            label="PX4 rate target", linewidth=1.3,
        )
        axes[3, column].plot(
            df["elapsed_s"], np.degrees(finite(df[rate_name])),
            label="Actual body rate", linewidth=1.0,
        )

        axes[0, column].set_title(column_titles[column])
        axes[0, column].set_ylabel("Position (m)")
        axes[1, column].set_ylabel("Velocity (m/s)")
        axes[2, column].set_ylabel("Attitude (deg)")
        axes[3, column].set_ylabel("Body rate (deg/s)")
        axes[3, column].set_xlabel("Elapsed time (s)")
        for row in range(4):
            shade_phases(axes[row, column], intervals)
            axes[row, column].grid(True, alpha=0.3)
            axes[row, column].legend(fontsize=8, loc="best")

    fig.suptitle("Outer-to-inner loop target and actual state chain")
    save_figure(fig, output / "11_outer_inner_loop_tracking.png")
    return True


def plot_acceleration_tracking(df, output, intervals):
    required = [
        *(f"planned_a{axis}" for axis in "xyz"),
        *(f"derived_actual_a{axis}_ned_filtered" for axis in "xyz"),
    ]
    if not all(has_signal(df, key) for key in required):
        return False

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for axis, name in zip(axes, "xyz"):
        axis.plot(
            df["elapsed_s"], finite(df[f"planned_a{name}"]),
            label="Planned NED acceleration", linewidth=1.4,
        )
        axis.plot(
            df["elapsed_s"], finite(df[f"derived_actual_a{name}_ned_raw"]),
            label="Derived actual (raw)", linewidth=0.7, alpha=0.25,
        )
        axis.plot(
            df["elapsed_s"], finite(df[f"derived_actual_a{name}_ned_filtered"]),
            label="Derived actual (5-sample mean)", linewidth=1.0,
        )
        shade_phases(axis, intervals)
        axis.set_ylabel(f"A{name.upper()} (m/s^2)")
        axis.grid(True, alpha=0.3)
        axis.legend(ncol=3)
    axes[-1].set_xlabel("Elapsed time (s)")
    fig.suptitle("Planned vs IMU-derived NED acceleration")
    save_figure(fig, output / "12_acceleration_tracking.png")
    return True


def plot_trajectory_derivatives(df, output, intervals):
    available = any(
        has_signal(df, key)
        for key in (
            "planned_jx",
            "planned_jy",
            "planned_jz",
            "planned_yaw_acceleration",
            "planned_yaw_jerk",
        )
    )
    if not available:
        return False

    fig, axes = plt.subplots(5, 1, figsize=(12, 14), sharex=True)

    for key, label in (
        ("planned_ax", "AX"),
        ("planned_ay", "AY"),
        ("planned_az", "AZ"),
    ):
        if has_signal(df, key):
            axes[0].plot(df["elapsed_s"], finite(df[key]), label=label)
    axes[0].set_ylabel("Acceleration (m/s^2)")

    for key, label in (
        ("planned_jx", "JX"),
        ("planned_jy", "JY"),
        ("planned_jz", "JZ"),
    ):
        if has_signal(df, key):
            axes[1].plot(df["elapsed_s"], finite(df[key]), label=label)
    axes[1].set_ylabel("Jerk (m/s^3)")

    axes[2].plot(
        df["elapsed_s"],
        finite(df["planned_yaw_rate"]),
        label="Yaw rate",
    )
    axes[2].set_ylabel("Yaw rate (rad/s)")

    if has_signal(df, "planned_yaw_acceleration"):
        axes[3].plot(
            df["elapsed_s"],
            finite(df["planned_yaw_acceleration"]),
            label="Yaw acceleration",
        )
    axes[3].set_ylabel("Yaw accel. (rad/s²)")

    if has_signal(df, "planned_yaw_jerk"):
        axes[4].plot(
            df["elapsed_s"],
            finite(df["planned_yaw_jerk"]),
            label="Yaw jerk",
        )
    axes[4].set_ylabel("Yaw jerk (rad/s³)")
    axes[4].set_xlabel("Elapsed time (s)")

    for axis in axes:
        shade_phases(axis, intervals)
        axis.grid(True, alpha=0.3)
        handles, _ = axis.get_legend_handles_labels()
        if handles:
            axis.legend(ncol=3)

    fig.suptitle("Exported trajectory derivatives")
    save_figure(fig, output / "10_trajectory_derivatives.png")
    return True


def build_metrics(df):
    trajectory = df[df["phase"] == "TRAJECTORY"].copy()
    records = []

    for key in ("x", "y", "z"):
        records.append(
            metric_record(
                key,
                trajectory[f"desired_{key}"],
                trajectory[key],
                "m",
            )
        )
    for key in ("vx", "vy", "vz"):
        records.append(
            metric_record(
                f"planned_{key}_vs_actual",
                trajectory[f"planned_{key}"],
                trajectory[key],
                "m/s",
            )
        )
        records.append(
            metric_record(
                f"commanded_{key}_vs_actual",
                trajectory[f"cmd_{key}"],
                trajectory[key],
                "m/s",
            )
        )
    records.append(
        metric_record(
            "yaw",
            trajectory["target_yaw"],
            trajectory["yaw"],
            "rad",
            wrap=True,
        )
    )
    for name, target_key, actual_key in (
        ("roll", "px4_target_roll", "roll"),
        ("pitch", "px4_target_pitch", "pitch"),
        ("yaw", "px4_target_yaw_from_q", "yaw"),
    ):
        if has_signal(trajectory, target_key) and has_signal(trajectory, actual_key):
            records.append(
                metric_record(
                    f"px4_attitude_{name}_vs_actual",
                    trajectory[target_key], trajectory[actual_key], "rad", wrap=True,
                )
            )
    for axis, target_key in zip(
        "pqr",
        (
            "attitude_target_roll_rate",
            "attitude_target_pitch_rate",
            "attitude_target_yaw_rate",
        ),
    ):
        if has_signal(trajectory, target_key) and has_signal(trajectory, axis):
            records.append(
                metric_record(
                    f"px4_body_rate_{axis}_vs_actual",
                    trajectory[target_key], trajectory[axis], "rad/s",
                )
            )
    for axis in "xyz":
        target_key = f"planned_a{axis}"
        actual_key = f"derived_actual_a{axis}_ned_filtered"
        if has_signal(trajectory, target_key) and has_signal(trajectory, actual_key):
            records.append(
                metric_record(
                    f"planned_a{axis}_vs_derived_actual",
                    trajectory[target_key], trajectory[actual_key], "m/s^2",
                )
            )
    records = [record for record in records if record is not None]

    sample_period = float(
        np.median(finite(trajectory["loop_dt_s"]).dropna())
    )
    lags = {
        key: best_lag(
            trajectory[f"planned_{key}"],
            trajectory[key],
            sample_period,
        )
        for key in ("vx", "vy", "vz")
    }

    desired_yaw = np.unwrap(
        trajectory["target_yaw"].to_numpy(dtype=float)
    )
    actual_yaw = np.unwrap(trajectory["yaw"].to_numpy(dtype=float))
    dt = np.gradient(trajectory["elapsed_s"].to_numpy(dtype=float))
    desired_yaw_rate = np.gradient(desired_yaw) / dt
    actual_yaw_rate = np.gradient(actual_yaw) / dt
    lags["yaw_rate"] = best_lag(
        desired_yaw_rate,
        actual_yaw_rate,
        sample_period,
    )

    loop_dt = finite(df["loop_dt_s"]).dropna()
    timing = {
        "target_period_s": 0.05,
        "median_period_s": float(loop_dt.iloc[1:].median()),
        "p99_period_s": float(loop_dt.iloc[1:].quantile(0.99)),
        "max_period_s": float(loop_dt.iloc[1:].max()),
        "periods_over_75ms": int((loop_dt.iloc[1:] > 0.075).sum()),
        "missed_periods_total": int(
            finite(df["missed_periods"]).fillna(0).sum()
        ),
    }

    setpoint_gaps = (
        finite(df["setpoint_last_gap_s"]).dropna()
        if "setpoint_last_gap_s" in df
        else pd.Series(dtype=float)
    )
    setpoint_gaps = setpoint_gaps[np.isfinite(setpoint_gaps)]
    if not setpoint_gaps.empty:
        timing["p99_setpoint_gap_s"] = float(
            setpoint_gaps.quantile(0.99)
        )
        timing["max_setpoint_gap_s"] = float(setpoint_gaps.max())

    if "setpoint_max_gap_s" in df:
        cumulative_max = finite(df["setpoint_max_gap_s"]).dropna()
        cumulative_max = cumulative_max[np.isfinite(cumulative_max)]
        if not cumulative_max.empty:
            timing["max_setpoint_gap_s"] = float(cumulative_max.max())

    for column, key in (
        ("setpoint_send_count", "setpoint_send_count_final"),
        ("setpoint_control_sends", "setpoint_control_sends_final"),
        ("setpoint_watchdog_resends", "watchdog_resends"),
    ):
        if column in df:
            values = finite(df[column]).dropna()
            values = values[np.isfinite(values)]
            if not values.empty:
                timing[key] = int(values.max())

    if "trajectory_clock_limited" in df:
        limited = (
            df["trajectory_clock_limited"]
            .astype(str)
            .str.lower()
            .isin(("true", "1", "1.0"))
        )
        timing["trajectory_clock_limited_count"] = int(limited.sum())

    freshness = {}
    for key in (
        "position_age_s",
        "attitude_age_s",
        "heartbeat_age_s",
        "imu_age_s",
        "px4_position_target_age_s",
        "px4_attitude_target_age_s",
        "actuator_age_s",
        "servo_age_s",
    ):
        if key not in df:
            freshness[key] = None
            continue
        values = finite(df[key]).dropna()
        values = values[np.isfinite(values)]
        freshness[key] = (
            None
            if values.empty
            else {
                "median_s": float(values.median()),
                "p99_s": float(values.quantile(0.99)),
                "max_s": float(values.max()),
            }
        )

    phase_summary = []
    for phase, group in df.groupby("phase", sort=False):
        phase_summary.append(
            {
                "phase": str(phase),
                "samples": int(len(group)),
                "start_s": float(group["elapsed_s"].iloc[0]),
                "end_s": float(group["elapsed_s"].iloc[-1]),
                "duration_s": float(
                    group["elapsed_s"].iloc[-1]
                    - group["elapsed_s"].iloc[0]
                ),
            }
        )

    saturation = {}
    for key in (
        "pid_x_saturated",
        "pid_y_saturated",
        "pid_z_saturated",
        "command_x_limited",
        "command_y_limited",
        "command_z_limited",
    ):
        values = df[key].astype(str).str.lower().isin(("true", "1"))
        saturation[key] = int(values.sum())

    return {
        "tracking_metrics": records,
        "tracking_lag": lags,
        "timing": timing,
        "telemetry_freshness": freshness,
        "phase_summary": phase_summary,
        "saturation_counts": saturation,
    }


def markdown_table(headers, rows):
    lines = [
        "| " + " | ".join(headers) + " |",
        "|" + "|".join("---" for _ in headers) + "|",
    ]
    lines.extend("| " + " | ".join(row) + " |" for row in rows)
    return "\n".join(lines)


def write_report(
    archive,
    output,
    metrics,
    inner_plot_created,
    derivative_plot_created,
    chain_plot_created,
    acceleration_plot_created,
):
    tracking_rows = []
    for record in metrics["tracking_metrics"]:
        tracking_rows.append(
            [
                record["signal"],
                record["units"],
                f"{record['rmse']:.5f}",
                f"{record['mae']:.5f}",
                f"{record['max_abs_error']:.5f}",
                f"{record['bias']:.5f}",
            ]
        )

    phase_rows = [
        [
            item["phase"],
            str(item["samples"]),
            f"{item['start_s']:.3f}",
            f"{item['end_s']:.3f}",
            f"{item['duration_s']:.3f}",
        ]
        for item in metrics["phase_summary"]
    ]

    lag_rows = []
    for signal, record in metrics["tracking_lag"].items():
        if record is None:
            lag_rows.append([signal, "n/a", "n/a"])
        else:
            lag_rows.append(
                [
                    signal,
                    f"{record['lag_seconds']:.3f}",
                    f"{record['correlation']:.4f}",
                ]
            )

    timing = metrics["timing"]
    timing_lines = [
        f"- Median loop interval: {timing['median_period_s'] * 1000:.3f} ms",
        f"- 99th-percentile interval: {timing['p99_period_s'] * 1000:.3f} ms",
        f"- Maximum interval: {timing['max_period_s'] * 1000:.3f} ms",
        f"- Intervals above 75 ms: {timing['periods_over_75ms']}",
        f"- Total skipped periods: {timing['missed_periods_total']}",
    ]
    if "max_setpoint_gap_s" in timing:
        timing_lines.append(
            f"- Maximum MAVLink setpoint gap: "
            f"{timing['max_setpoint_gap_s'] * 1000:.3f} ms"
        )
    if "p99_setpoint_gap_s" in timing:
        timing_lines.append(
            f"- 99th-percentile MAVLink setpoint gap: "
            f"{timing['p99_setpoint_gap_s'] * 1000:.3f} ms"
        )
    if "watchdog_resends" in timing:
        timing_lines.append(
            f"- Watchdog setpoint resends: {timing['watchdog_resends']}"
        )
    if "trajectory_clock_limited_count" in timing:
        timing_lines.append(
            f"- Trajectory clock limited samples: "
            f"{timing['trajectory_clock_limited_count']}"
        )

    report = [
        "# PX4 Instrumented Simulation Report",
        "",
        f"Archive: `{archive}`",
        "",
        "## Tracking metrics",
        "",
        markdown_table(
            ["Signal", "Units", "RMSE", "MAE", "Max abs error", "Bias"],
            tracking_rows,
        ),
        "",
        "The `planned_*` signals are trajectory feedforward. The "
        "`cmd_*` signals are the final Vishnu outer-loop velocity "
        "commands after PID correction.",
        "",
        "## Tracking lag",
        "",
        markdown_table(
            ["Signal", "Lag (s)", "Correlation"],
            lag_rows,
        ),
        "",
        "Positive lag means the measured response follows the desired signal.",
        "",
        "## Controller timing",
        "",
        *timing_lines,
        "",
        "## Phase timing",
        "",
        markdown_table(
            ["Phase", "Samples", "Start (s)", "End (s)", "Duration (s)"],
            phase_rows,
        ),
        "",
        "## Plot files",
        "",
        "- `01_position_tracking.png`",
        "- `02_velocity_tracking.png`",
        "- `03_outer_loop_terms.png`",
        "- `04_yaw_tracking.png`",
        "- `05_tracking_errors.png`",
        "- `06_attitude_rates.png`",
        "- `07_timing_telemetry.png`",
        "- `08_path_3d.png`",
    ]
    if inner_plot_created:
        report.append("- `09_inner_loop_signals.png`")
    if derivative_plot_created:
        report.append("- `10_trajectory_derivatives.png`")
    if chain_plot_created:
        report.append("- `11_outer_inner_loop_tracking.png`")
    if acceleration_plot_created:
        report.append("- `12_acceleration_tracking.png`")
    report.extend(
        [
            "",
            "## Interpretation guardrails",
            "",
            "- Yaw error is wrapped to +/- pi; the yaw plot is unwrapped only "
            "for visual continuity.",
            "- IMU acceleration is rotated from body to local NED and gravity "
            "compensated before comparison. A centered five-sample mean is "
            "shown beside the raw derived signal.",
            "- Real-flight comparison must use the same metrics after time, "
            "origin, yaw, and coordinate-frame alignment.",
            "",
        ]
    )
    report_text = "\n".join(report)
    (output / "run_report.md").write_text(
        report_text,
        encoding="utf-8",
    )
    # Retain the legacy name for compatibility with existing archives/tools.
    (output / "baseline_report.md").write_text(
        report_text,
        encoding="utf-8",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("archive", type=Path)
    args = parser.parse_args()

    archive = args.archive.resolve()
    logs = sorted(archive.glob("research_log_*.csv"))
    if len(logs) != 1:
        raise RuntimeError(
            f"Expected one research_log_*.csv in {archive}, found {len(logs)}"
        )

    output = archive / "analysis"
    output.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(logs[0])
    required = {
        "elapsed_s",
        "phase",
        "desired_x",
        "x",
        "planned_vx",
        "cmd_vx",
        "vx",
        "target_yaw",
        "yaw",
        "loop_dt_s",
    }
    missing = required - set(df.columns)
    if missing:
        raise RuntimeError(f"Missing required log columns: {sorted(missing)}")

    numeric_exceptions = {"phase", "phase_transition", "mode", "armed"}
    for column in df.columns:
        if column not in numeric_exceptions:
            df[column] = finite(df[column])

    enrich_derived_signals(df)

    intervals = phase_intervals(df)
    metrics = build_metrics(df)

    plot_position(df, output, intervals)
    plot_velocity(df, output, intervals)
    plot_outer_terms(df, output, intervals)
    plot_yaw(df, output, intervals)
    plot_errors(df, output, intervals)
    plot_attitude(df, output, intervals)
    plot_timing(df, output, intervals)
    plot_path(df, output)
    inner_created = plot_inner_signals(df, output, intervals)
    derivative_created = plot_trajectory_derivatives(
        df,
        output,
        intervals,
    )
    chain_created = plot_outer_inner_chain(df, output, intervals)
    acceleration_created = plot_acceleration_tracking(
        df, output, intervals
    )

    with (output / "metrics.json").open("w", encoding="utf-8") as handle:
        json.dump(metrics, handle, indent=2, allow_nan=False)
    pd.DataFrame(metrics["tracking_metrics"]).to_csv(
        output / "tracking_metrics.csv",
        index=False,
    )
    pd.DataFrame(metrics["phase_summary"]).to_csv(
        output / "phase_summary.csv",
        index=False,
    )
    write_report(
        archive,
        output,
        metrics,
        inner_created,
        derivative_created,
        chain_created,
        acceleration_created,
    )

    print(f"ANALYSIS_COMPLETE={output}")
    print(f"ROWS={len(df)}")
    print(f"INNER_LOOP_PLOT={str(inner_created).lower()}")
    print(
        f"TRAJECTORY_DERIVATIVE_PLOT="
        f"{str(derivative_created).lower()}"
    )
    print(f"OUTER_INNER_CHAIN_PLOT={str(chain_created).lower()}")
    print(f"ACCELERATION_TRACKING_PLOT={str(acceleration_created).lower()}")


if __name__ == "__main__":
    main()
