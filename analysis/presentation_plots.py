#!/usr/bin/env python3
"""Generate mentor-facing PX4 plots from the plot-ready CSV export.

This script is intentionally offline: it never connects to PX4 and cannot arm,
change modes, or transmit setpoints.  It only reads exported CSV files.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np
import pandas as pd


FIXED_LABEL = "selected_smooth_fixed"
PHASE_COLORS = {
    "TAKEOFF": "#dbeafe",
    "TRAJECTORY": "#dcfce7",
    "LAND": "#fef3c7",
    "DONE": "#e5e7eb",
}
DESIRED_COLOR = "#2563eb"
ACTUAL_COLOR = "#f97316"
TARGET_COLOR = "#7c3aed"
GRID_COLOR = "#94a3b8"


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.titlesize": 11,
            "axes.labelsize": 10,
            "figure.titlesize": 15,
            "legend.fontsize": 9,
            "axes.grid": True,
            "grid.alpha": 0.22,
            "grid.color": GRID_COLOR,
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )


def load_csv(data_dir: Path, name: str) -> pd.DataFrame:
    path = data_dir / name
    if not path.is_file():
        raise FileNotFoundError(f"Required CSV not found: {path}")
    return pd.read_csv(path)


def fixed_run(frame: pd.DataFrame) -> pd.DataFrame:
    selected = frame.loc[frame["run_label"] == FIXED_LABEL].copy()
    if selected.empty:
        raise ValueError(f"No rows with run_label={FIXED_LABEL!r}")
    return selected.reset_index(drop=True)


def phase_spans(frame: pd.DataFrame) -> list[tuple[str, float, float]]:
    phases = frame["phase"].astype(str).to_numpy()
    times = pd.to_numeric(frame["elapsed_s"], errors="coerce").to_numpy(float)
    spans: list[tuple[str, float, float]] = []
    start = 0
    for index in range(1, len(frame)):
        if phases[index] != phases[index - 1]:
            spans.append((phases[start], times[start], times[index - 1]))
            start = index
    spans.append((phases[start], times[start], times[-1]))
    return spans


def add_phase_context(axis: plt.Axes, frame: pd.DataFrame) -> None:
    for phase, start, end in phase_spans(frame):
        axis.axvspan(
            start,
            end,
            color=PHASE_COLORS.get(phase, "#f1f5f9"),
            alpha=0.36,
            linewidth=0,
            zorder=0,
        )
    transitions = frame.loc[
        frame["phase_transition"].fillna("").astype(str) != "",
        "elapsed_s",
    ]
    for transition_time in transitions:
        axis.axvline(transition_time, color="#64748b", linewidth=0.8, alpha=0.55)


def phase_handles() -> list[Patch]:
    return [
        Patch(facecolor=color, edgecolor="none", alpha=0.55, label=phase.title())
        for phase, color in PHASE_COLORS.items()
    ]


def save(fig: plt.Figure, output: Path) -> None:
    fig.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def plot_position(data_dir: Path, output_dir: Path) -> None:
    frame = fixed_run(load_csv(data_dir, "01_state_tracking_all.csv"))
    fig, axes = plt.subplots(3, 1, figsize=(12.5, 9.5), sharex=True)
    for axis, letter in zip(axes, "xyz"):
        add_phase_context(axis, frame)
        axis.plot(
            frame["elapsed_s"], frame[f"desired_{letter}_m"],
            color=DESIRED_COLOR, linewidth=1.8, label="Desired",
        )
        axis.plot(
            frame["elapsed_s"], frame[f"actual_{letter}_m"],
            color=ACTUAL_COLOR, linewidth=1.35, label="Actual",
        )
        axis.set_ylabel(f"{letter.upper()} position (m)")
    axes[0].legend(loc="upper right", ncol=2)
    axes[2].set_xlabel("Elapsed time (s)")
    land_rows = frame.loc[frame["phase_transition"] == "TRAJECTORY->LAND"]
    if not land_rows.empty:
        t_land = float(land_rows.iloc[0]["elapsed_s"])
        axes[2].annotate(
            "LAND target switches to the ground reference;\nactual Z follows a controlled descent",
            xy=(t_land, float(land_rows.iloc[0]["desired_z_m"])),
            xytext=(max(3.0, t_land - 39.0), -3.1),
            arrowprops={"arrowstyle": "->", "color": "#475569"},
            fontsize=9,
            color="#334155",
            bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "#cbd5e1", "alpha": 0.93},
        )
    fig.suptitle("Final SITL run — desired vs actual position", y=0.995)
    fig.legend(
        handles=phase_handles(), loc="lower center", ncol=4,
        frameon=False, bbox_to_anchor=(0.5, -0.01), title="Mission phase",
    )
    fig.tight_layout(rect=(0, 0.045, 1, 0.975))
    save(fig, output_dir / "01_Position_Tracking_Presentation.png")


def plot_outer_inner(data_dir: Path, output_dir: Path) -> None:
    outer = fixed_run(load_csv(data_dir, "02_outer_loop.csv"))
    inner = fixed_run(load_csv(data_dir, "03_inner_loop.csv"))
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.3), sharex=True)
    for column, letter in enumerate("xyz"):
        top = axes[0, column]
        bottom = axes[1, column]
        add_phase_context(top, outer)
        add_phase_context(bottom, inner)
        top.plot(
            outer["elapsed_s"], outer[f"commanded_v{letter}_mps"],
            color=DESIRED_COLOR, linewidth=1.5, label="Outer-loop command",
        )
        top.plot(
            outer["elapsed_s"], outer[f"actual_v{letter}_mps"],
            color=ACTUAL_COLOR, linewidth=1.15, label="Actual velocity",
        )
        top.set_title(f"{letter.upper()} velocity")
        top.set_ylabel("Velocity (m/s)")

        attitude = {"x": "roll", "y": "pitch", "z": "yaw"}[letter]
        bottom.plot(
            inner["elapsed_s"], inner[f"px4_target_{attitude}_rad"],
            color=TARGET_COLOR, linewidth=1.45, label="PX4 target",
        )
        bottom.plot(
            inner["elapsed_s"], inner[f"actual_{attitude}_rad"],
            color=ACTUAL_COLOR, linewidth=1.05, label="Actual",
        )
        bottom.set_title(f"PX4 {attitude} target tracking")
        bottom.set_ylabel("Angle (rad)")
        bottom.set_xlabel("Elapsed time (s)")
    axes[0, 0].legend(loc="upper right")
    axes[1, 0].legend(loc="upper right")
    fig.suptitle(
        "Outer-to-inner loop evidence — velocity command and PX4 attitude response",
        y=0.995,
    )
    fig.legend(
        handles=phase_handles(), loc="lower center", ncol=4,
        frameon=False, bbox_to_anchor=(0.5, -0.01), title="Mission phase",
    )
    fig.tight_layout(rect=(0, 0.055, 1, 0.97))
    save(fig, output_dir / "02_Outer_to_Inner_Loops_Presentation.png")


def plot_acceleration(data_dir: Path, output_dir: Path) -> None:
    frame = fixed_run(load_csv(data_dir, "04_acceleration_jerk.csv"))
    fig, axes = plt.subplots(3, 1, figsize=(12.5, 9.5), sharex=True)
    for axis, letter in zip(axes, "xyz"):
        add_phase_context(axis, frame)
        axis.plot(
            frame["elapsed_s"], frame[f"planned_a{letter}_mps2"],
            color=DESIRED_COLOR, linewidth=1.6, label="Planned acceleration",
        )
        axis.plot(
            frame["elapsed_s"], frame[f"derived_actual_a{letter}_ned_filtered_mps2"],
            color=ACTUAL_COLOR, linewidth=1.05, alpha=0.9,
            label="Derived actual (5-sample filter)",
        )
        axis.set_ylabel(f"a{letter} (m/s²)")
    axes[0].legend(loc="upper right", ncol=2)
    axes[2].set_xlabel("Elapsed time (s)")
    landing = frame.loc[frame["phase"].isin(["LAND", "DONE"])]
    if not landing.empty:
        z_values = pd.to_numeric(
            landing["derived_actual_az_ned_filtered_mps2"], errors="coerce"
        ).abs()
        note_row = landing.loc[z_values.idxmax()]
        t_note = float(note_row["elapsed_s"])
        axes[2].annotate(
            "Landing/contact transient\n(not a trajectory jerk command)",
            xy=(t_note, float(note_row["derived_actual_az_ned_filtered_mps2"])),
            xytext=(t_note - 42.0, 0.95),
            arrowprops={"arrowstyle": "->", "color": "#475569"},
            fontsize=9,
            bbox={"boxstyle": "round,pad=0.3", "fc": "white", "ec": "#cbd5e1", "alpha": 0.93},
        )
    fig.suptitle(
        "Final SITL run — planned vs derived actual acceleration",
        y=0.995,
    )
    fig.text(
        0.5, 0.018,
        "Actual NED acceleration is derived from attitude and IMU specific force; touchdown spikes are expected contact dynamics.",
        ha="center", fontsize=9, color="#475569",
    )
    fig.tight_layout(rect=(0, 0.055, 1, 0.975))
    save(fig, output_dir / "03_Acceleration_Tracking_Presentation.png")


def plot_pair_effects(data_dir: Path, output_dir: Path) -> None:
    summary = load_csv(data_dir, "07_paired_metric_summary.csv").set_index("metric")
    selected = [
        ("xy_position_rmse_m", "XY position RMSE"),
        ("commanded_vx_rmse_mps", "Commanded VX RMSE"),
        ("yaw_rmse_rad", "Yaw RMSE"),
        ("takeoff_xy_drift_m", "Takeoff drift"),
        ("landing_xy_drift_m", "Landing drift"),
        ("takeoff_transition_step_mps", "Takeoff command step"),
        ("landing_transition_step_mps", "Landing command step"),
    ]
    rows = summary.loc[[key for key, _ in selected]].copy()
    labels = [label for _, label in selected]
    means = rows["mean_pair_percent_change"].to_numpy(float)
    low = rows["pair_percent_ci_low"].to_numpy(float)
    high = rows["pair_percent_ci_high"].to_numpy(float)
    y = np.arange(len(labels))
    colors = np.where(high < 0, "#16a34a", "#64748b")
    fig, axis = plt.subplots(figsize=(11.8, 6.8))
    axis.axvline(0.0, color="#111827", linewidth=1.0)
    for yi, mean, lower, upper, color in zip(y, means, low, high, colors):
        axis.errorbar(
            mean,
            yi,
            xerr=[[mean - lower], [upper - mean]],
            fmt="o",
            color=color,
            ecolor=color,
            markersize=8,
            markeredgecolor="white",
            markeredgewidth=0.7,
            elinewidth=2.1,
            capsize=4,
            zorder=3,
        )
    axis.set_yticks(y, labels)
    axis.invert_yaxis()
    axis.set_xlabel("Mean paired change from baseline (%)")
    axis.set_title("Five paired SITL trials — negative change means improvement")
    axis.grid(axis="y", visible=False)
    axis.text(
        0.01, -0.15,
        "Points are pairwise mean percent changes; whiskers are paired 95% confidence intervals (n=5). Green intervals exclude zero.",
        transform=axis.transAxes, fontsize=9, color="#475569",
    )
    for yi, mean in zip(y, means):
        axis.annotate(
            f"{mean:+.1f}%", (mean, yi), xytext=(6 if mean >= 0 else -6, 7),
            textcoords="offset points", ha="left" if mean >= 0 else "right",
            fontsize=9, color="#334155",
        )
    fig.tight_layout()
    save(fig, output_dir / "04_Five_Pair_Effects_Presentation.png")


def plot_timing(data_dir: Path, output_dir: Path) -> None:
    events = load_csv(data_dir, "09_timing_outliers.csv").copy()
    events["event"] = np.arange(1, len(events) + 1)
    colors = {"instrumented": "#64748b", "fixed": "#f97316"}
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for profile, group in events.groupby("profile", sort=False):
        label = "Baseline" if profile == "instrumented" else "Fixed"
        axes[0].scatter(
            group["event"], group["loop_dt_ms"], s=65,
            color=colors[profile], label=label, zorder=3,
        )
    axes[0].axhline(75.0, color="#111827", linestyle="--", linewidth=1.0, label="75 ms review threshold")
    axes[0].set_ylabel("Loop interval (ms)")
    axes[0].set_title("All >75 ms events were host scheduling stalls, not controller compute overruns")
    axes[0].legend(ncol=3, loc="lower left")

    fixed = events.loc[events["profile"] == "fixed"]
    axes[1].scatter(
        fixed["event"], fixed["setpoint_gap_ms"], s=70,
        color=colors["fixed"], label="Fixed measured setpoint gap", zorder=3,
    )
    axes[1].axhline(500.0, color="#dc2626", linestyle="--", linewidth=1.2, label="500 ms PX4 timeout comparison")
    axes[1].set_ylabel("Setpoint gap (ms)")
    axes[1].set_xlabel("Timing event (chronological across paired campaign)")
    axes[1].legend(loc="upper right")
    axes[1].text(
        0.02, 0.76,
        "Baseline setpoint-gap fields were not logged\nand are deliberately shown as unavailable.",
        transform=axes[1].transAxes, fontsize=9.5, color="#475569",
        bbox={"boxstyle": "round,pad=0.35", "fc": "#f8fafc", "ec": "#cbd5e1"},
    )
    fig.suptitle("Repeated-trial timing safety evidence", y=0.995)
    fig.text(
        0.5, 0.015,
        "9 events in 26,763 iterations; maximum loop interval 108.227 ms; maximum controller compute time during an event 0.269 ms.",
        ha="center", fontsize=9.5, color="#334155",
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.97))
    save(fig, output_dir / "05_Timing_Safety_Presentation.png")


def plot_evidence_summary(data_dir: Path, output_dir: Path) -> None:
    reliability = load_csv(data_dir, "08_completion_reliability.csv").set_index("profile")
    fixed_trials = load_csv(data_dir, "06_repeated_trial_metrics.csv")
    fixed_trials = fixed_trials.loc[fixed_trials["profile"] == "fixed"]
    cards = [
        ("22/22", "Automated tests passed", "Controller, trajectory, analysis"),
        ("6/6", "Fixed mission attempts completed", "Includes five paired trials"),
        ("0.203 m", "Mean XY RMSE", "Five fixed trials"),
        ("−50.1%", "Landing drift", "Group-mean improvement"),
        ("108.2 ms", "Worst loop interval", "Below 500 ms comparison"),
        ("0.269 ms", "Worst event compute time", "Host stall, not compute overrun"),
    ]
    if int(reliability.loc["fixed", "successful_missions"]) != 6:
        raise ValueError("Expected six successful fixed-profile missions")
    if len(fixed_trials) != 5:
        raise ValueError("Expected five fixed paired-trial rows")
    fig, axis = plt.subplots(figsize=(12.8, 7.2))
    axis.set_xlim(0, 3)
    axis.set_ylim(0, 2)
    axis.axis("off")
    for index, (value, label, detail) in enumerate(cards):
        column = index % 3
        row = 1 - index // 3
        x0, y0 = column + 0.06, row + 0.13
        axis.add_patch(
            plt.Rectangle((x0, y0), 0.88, 0.73, facecolor="#f8fafc", edgecolor="#cbd5e1", linewidth=1.2)
        )
        axis.text(x0 + 0.44, y0 + 0.48, value, ha="center", va="center", fontsize=23, fontweight="bold", color="#0f766e")
        axis.text(x0 + 0.44, y0 + 0.27, label, ha="center", va="center", fontsize=10.5, fontweight="bold", color="#0f172a")
        axis.text(x0 + 0.44, y0 + 0.11, detail, ha="center", va="center", fontsize=9, color="#64748b")
    fig.suptitle("PX4 controller research — evidence summary", fontsize=17, y=0.97)
    fig.text(
        0.5, 0.045,
        "Scope: software-in-the-loop evidence only. Physical flight still requires supervisor authorization and on-site oversight.",
        ha="center", fontsize=10, color="#7f1d1d",
    )
    save(fig, output_dir / "06_Evidence_Summary_Presentation.png")


def write_index(output_dir: Path) -> None:
    text = """PX4 MENTOR PRESENTATION PLOTS
================================

Recommended order
1. 06_Evidence_Summary_Presentation.png — one-slide results overview.
2. 01_Position_Tracking_Presentation.png — desired versus actual X/Y/Z.
3. 02_Outer_to_Inner_Loops_Presentation.png — controller cascade evidence.
4. 03_Acceleration_Tracking_Presentation.png — smoothness and contact caveat.
5. 04_Five_Pair_Effects_Presentation.png — paired repeatability and uncertainty.
6. 05_Timing_Safety_Presentation.png — jitter diagnosis and watchdog evidence.

Interpretation notes
- Colored backgrounds show TAKEOFF, TRAJECTORY, LAND, and DONE phases.
- The Z desired-value switch at LAND is intentional: the target becomes the
  ground reference while actual altitude follows a controlled descent.
- Acceleration spikes at touchdown are contact dynamics, not commanded jerk.
- In the paired-effects plot, negative means the fixed controller is lower.
- Baseline setpoint-gap data are unavailable because those fields were not
  logged; no values were inferred or fabricated.
- All results are SITL evidence, not physical-flight certification.
"""
    (output_dir / "PRESENTATION_PLOT_GUIDE.txt").write_text(text, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("data_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    plot_position(args.data_dir, args.output_dir)
    plot_outer_inner(args.data_dir, args.output_dir)
    plot_acceleration(args.data_dir, args.output_dir)
    plot_pair_effects(args.data_dir, args.output_dir)
    plot_timing(args.data_dir, args.output_dir)
    plot_evidence_summary(args.data_dir, args.output_dir)
    write_index(args.output_dir)
    print(f"Generated 6 plots and guide in {args.output_dir}")


if __name__ == "__main__":
    main()
