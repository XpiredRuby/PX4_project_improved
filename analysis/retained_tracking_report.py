#!/usr/bin/env python3
"""Re-audit retained telemetry; never describe historical data as a new run."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def statistics(values):
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    return dict(
        samples=len(values),
        finite_samples=len(finite),
        rms=float(np.sqrt(np.mean(finite**2))) if len(finite) else None,
        p95=float(np.percentile(np.abs(finite), 95)) if len(finite) else None,
        maximum=float(np.max(np.abs(finite))) if len(finite) else None,
    )


def report(source, output, recording_label="Retained recording"):
    source, output = Path(source), Path(output)
    df = pd.read_csv(source)
    columns = ["elapsed_s", "phase"] + [
        prefix + k for prefix in ("desired_", "truth_") for k in ("x", "y", "z")
    ]
    if not set(columns) <= set(df.columns):
        raise ValueError("Retained log lacks aligned desired and truth positions")
    times = df.elapsed_s.to_numpy(float)
    if not np.isfinite(times).all() or np.any(np.diff(times) <= 0.0):
        raise ValueError(
            "Telemetry elapsed clock must be finite and strictly increasing"
        )
    reference = df[["desired_x", "desired_y", "desired_z"]].to_numpy(float)
    truth = df[["truth_x", "truth_y", "truth_z"]].to_numpy(float)
    error = truth - reference
    output.mkdir(parents=True, exist_ok=False)
    plt.rcParams.update(
        {"font.size": 10, "figure.dpi": 130, "axes.grid": True, "grid.alpha": 0.2}
    )

    def phase_bands(axes):
        phases = df.phase.to_numpy()
        changes = np.r_[0, np.flatnonzero(phases[1:] != phases[:-1]) + 1, len(phases)]
        palette = {
            "TAKEOFF": "#43a2ca",
            "TRAJECTORY": "#66c2a4",
            "RETURN_HOME": "#fdbb84",
            "PX4_LAND": "#c994c7",
        }
        for axis in axes:
            for left, right in zip(changes, changes[1:], strict=False):
                phase = str(phases[left])
                axis.axvspan(
                    times[left],
                    times[min(right, len(times) - 1)],
                    color=palette.get(phase, "#cccccc"),
                    alpha=0.10,
                    linewidth=0.0,
                )
            for stamp in times[1:][np.diff(times) > 0.25]:
                axis.axvline(
                    stamp, color="#b2182b", alpha=0.3, linestyle=":", linewidth=0.8
                )

    colors = ("#1984a5", "#8c54a5")
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i, (axis, label) in enumerate(
        zip(axes, ("North", "East", "Down"), strict=True)
    ):
        axis.plot(
            times,
            reference[:, i],
            label="Recorded commanded reference",
            color=colors[0],
        )
        axis.plot(
            times,
            truth[:, i],
            label="Retained simulator position truth",
            color=colors[1],
            linewidth=0.9,
        )
        axis.set_ylabel(label + " (m)")
    phase_bands(axes)
    axes[0].legend()
    axes[-1].set_xlabel("Recorded elapsed time (s)")
    fig.suptitle(recording_label + " · no new simulation")
    fig.tight_layout()
    fig.savefig(output / "position.png")
    plt.close(fig)
    fig, axes = plt.subplots(
        1, 2, figsize=(12, 5), gridspec_kw={"width_ratios": [1, 2]}
    )
    axes[0].plot(
        reference[:, 1], reference[:, 0], label="Recorded reference", color=colors[0]
    )
    axes[0].plot(truth[:, 1], truth[:, 0], label="Simulator truth", color=colors[1])
    axes[0].set_aspect("equal", adjustable="box")
    axes[0].set(xlabel="East (m)", ylabel="North (m)", title="Archived ground track")
    axes[0].legend()
    axes[1].plot(times, np.linalg.norm(error, axis=1), label="3D position error")
    axes[1].plot(times, np.linalg.norm(error[:, :2], axis=1), label="Horizontal error")
    axes[1].plot(times, np.abs(error[:, 2]), label="Vertical error")
    axes[1].set(xlabel="Recorded elapsed time (s)", ylabel="Position error (m)")
    phase_bands([axes[1]])
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output / "track-and-error.png")
    plt.close(fig)
    fig = plt.figure(figsize=(8, 6))
    axis = fig.add_subplot(111, projection="3d")
    axis.plot(
        reference[:, 0],
        reference[:, 1],
        -reference[:, 2],
        label="Recorded reference",
        color=colors[0],
    )
    axis.plot(
        truth[:, 0],
        truth[:, 1],
        -truth[:, 2],
        label="Simulator position truth",
        color=colors[1],
        linewidth=0.9,
    )
    finite_truth = truth[np.isfinite(truth).all(axis=1)]
    finite_reference = reference[np.isfinite(reference).all(axis=1)]
    extent = np.ptp(np.vstack([finite_truth, finite_reference]), axis=0)
    axis.set_box_aspect(np.maximum(extent, 0.1))
    axis.set(
        xlabel="North (m)",
        ylabel="East (m)",
        zlabel="Up (m)",
        title="Archived 3D track · equal physical scale",
    )
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "route-3d.png")
    plt.close(fig)
    available_velocity = all(
        k in df for k in ("vx", "vy", "vz", "planned_vx", "planned_vy", "planned_vz")
    )
    if available_velocity:
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for axis, k in zip(axes, ("vx", "vy", "vz"), strict=True):
            axis.plot(
                times, df["planned_" + k], label="Recorded reference", color=colors[0]
            )
            axis.plot(
                times,
                df[k],
                label="PX4 estimated velocity (not truth)",
                color=colors[1],
                linewidth=0.9,
            )
            axis.set_ylabel(k + " (m/s)")
        phase_bands(axes)
        axes[0].legend()
        axes[-1].set_xlabel("Recorded elapsed time (s)")
        fig.suptitle("Archived estimated velocity versus reference")
        fig.tight_layout()
        fig.savefig(output / "velocity.png")
        plt.close(fig)
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    for axis, k in zip(axes, ("roll", "pitch", "yaw"), strict=True):
        if k in df:
            axis.plot(
                times, np.degrees(np.unwrap(df[k].to_numpy(float))), color=colors[1]
            )
        axis.set_ylabel(k + " (deg)")
    phase_bands(axes)
    axes[-1].set_xlabel("Recorded elapsed time (s)")
    fig.suptitle(
        "Archived PX4 estimated attitude · desired attitude and truth quaternion absent"
    )
    fig.tight_layout()
    fig.savefig(output / "attitude.png")
    plt.close(fig)
    phases = {}
    for phase in df.phase.unique():
        mask = df.phase.to_numpy() == phase
        phases[phase] = dict(
            position_3d_m=statistics(np.linalg.norm(error[mask], axis=1)),
            horizontal_m=statistics(np.linalg.norm(error[mask, :2], axis=1)),
            vertical_m=statistics(error[mask, 2]),
        )
    result = dict(
        source=source.as_posix(),
        source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        scope="Reanalysis of "
        + recording_label
        + "; does not validate new planner code",
        new_simulator_run=False,
        physical_flight=False,
        samples=len(df),
        largest_telemetry_gap_s=float(np.max(np.diff(times))),
        per_phase=phases,
        missing_channels=[
            "simulator truth velocity and acceleration",
            "desired acceleration and jerk",
            "desired yaw and yaw rate",
            "simulator truth quaternion",
            "desired roll and pitch",
            "continuous actuator authority",
        ],
        plot_annotation="Shading separates recorded phases; red dotted lines mark telemetry gaps over 0.25 s",
        velocity_comparison="PX4 estimate vs logged plan; not independent velocity truth",
    )
    (output / "metrics.json").write_text(
        json.dumps(result, indent=2, allow_nan=False) + "\n"
    )
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--recording-label", default="Retained recording")
    args = parser.parse_args()
    print(json.dumps(report(args.source, args.output, args.recording_label), indent=2))
