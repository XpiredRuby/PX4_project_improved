#!/usr/bin/env python3

import csv
import hashlib
import json
import statistics
import sys
from pathlib import Path


FIELDS = [
    "archive",
    "mode",
    "seed",
    "requested_z_m",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "realized_z_m",
    "spawn_height_error_m",
    "stabilization_s",
    "stabilization_excursion_m",
    "stabilization_horizontal_speed_m_s",
    "stabilization_vertical_excursion_m",
    "stabilization_vertical_speed_m_s",
    "trajectory_x_rmse_m",
    "trajectory_y_rmse_m",
    "trajectory_z_rmse_m",
    "handoff_height_m",
    "handoff_vz_m_s",
    "final_xy_error_m",
    "final_z_error_m",
    "native_disarm",
    "success",
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if len(sys.argv) < 4:
        raise SystemExit(
            "usage: summarize_randomized_trials.py OUTPUT_DIR ARCHIVE [ARCHIVE ...]"
        )
    output = Path(sys.argv[1]).resolve()
    archives = [Path(value).resolve() for value in sys.argv[2:]]
    output.mkdir(parents=True, exist_ok=True)

    records = []
    source_metrics = []
    for archive in archives:
        metrics = json.loads(
            (archive / "randomized_metrics.json").read_text(encoding="utf-8")
        )
        source_metrics.append(metrics)
        spawn = metrics["requested_spawn"]
        landing = metrics["landing"]
        trajectory = metrics["trajectory"]
        stabilization = metrics["stabilization"]
        records.append(
            {
                "archive": str(archive),
                "mode": spawn["mode"],
                "seed": spawn["seed"],
                "requested_z_m": spawn["z"],
                "roll_deg": spawn["roll_deg"],
                "pitch_deg": spawn["pitch_deg"],
                "yaw_deg": spawn["yaw_deg"],
                "realized_z_m": metrics["realized_spawn_height_m"],
                "spawn_height_error_m": (
                    metrics["realized_spawn_height_m"] - spawn["z"]
                ),
                "stabilization_s": stabilization["duration_s"],
                "stabilization_excursion_m": stabilization[
                    "max_horizontal_excursion_m"
                ],
                "stabilization_horizontal_speed_m_s": stabilization[
                    "max_horizontal_speed_m_s"
                ],
                "stabilization_vertical_excursion_m": stabilization[
                    "max_vertical_excursion_m"
                ],
                "stabilization_vertical_speed_m_s": stabilization[
                    "max_abs_vertical_speed_m_s"
                ],
                "trajectory_x_rmse_m": trajectory["x_rmse_m"],
                "trajectory_y_rmse_m": trajectory["y_rmse_m"],
                "trajectory_z_rmse_m": trajectory["z_rmse_m"],
                "handoff_height_m": landing["handoff_height_above_target_m"],
                "handoff_vz_m_s": landing["handoff_vz_m_s"],
                "final_xy_error_m": landing["final_xy_error_m"],
                "final_z_error_m": landing[
                    "final_z_error_to_ground_reference_m"
                ],
                "native_disarm": landing["native_landing_disarm_confirmed"],
                "success": metrics["success"],
            }
        )

    with (output / "trials.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(records)

    aggregate = {
        "trial_count": len(records),
        "success_count": sum(bool(row["success"]) for row in records),
        "native_disarm_count": sum(bool(row["native_disarm"]) for row in records),
        "max_abs_spawn_height_error_m": max(
            abs(row["spawn_height_error_m"]) for row in records
        ),
        "max_final_xy_error_m": max(row["final_xy_error_m"] for row in records),
        "max_abs_final_z_error_m": max(abs(row["final_z_error_m"]) for row in records),
        "max_handoff_vz_m_s": max(row["handoff_vz_m_s"] for row in records),
        "handoff_height_range_m": [
            min(row["handoff_height_m"] for row in records),
            max(row["handoff_height_m"] for row in records),
        ],
        "max_trajectory_z_rmse_m": max(
            row["trajectory_z_rmse_m"] for row in records
        ),
        "median_stabilization_s": statistics.median(
            row["stabilization_s"] for row in records
        ),
        "max_stabilization_excursion_m": max(
            row["stabilization_excursion_m"] for row in records
        ),
        "max_stabilization_horizontal_speed_m_s": max(
            row["stabilization_horizontal_speed_m_s"] for row in records
        ),
        "max_stabilization_vertical_excursion_m": max(
            row["stabilization_vertical_excursion_m"] for row in records
        ),
        "max_stabilization_vertical_speed_m_s": max(
            row["stabilization_vertical_speed_m_s"] for row in records
        ),
    }
    root = Path("/mnt/f/PX4")
    source_files = [
        root / "research/randomized/RandomizedPositionController.py",
        root / "research/randomized/spawn_config.py",
        root / "research/randomized/offboard_runner.py",
        root / "research/analysis/run_randomized_regression.sh",
        root / "research/analysis/analyze_randomized_run.py",
    ]
    summary = {
        "aggregate": aggregate,
        "source_sha256": {str(path): digest(path) for path in source_files},
        "trials": records,
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )

    lines = [
        "# PX4 randomized initialization validation",
        "",
        f"Validated trials: **{aggregate['success_count']}/{aggregate['trial_count']}**",
        "",
        "| Mode / seed | Z requested | Roll / pitch | Final XY | Final Z | Handoff | Z RMSE |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for row in records:
        lines.append(
            f"| {row['mode']} / {row['seed']} | {row['requested_z_m']:.3f} m | "
            f"{row['roll_deg']:.2f}° / {row['pitch_deg']:.2f}° | "
            f"{row['final_xy_error_m']:.3f} m | {row['final_z_error_m']:.3f} m | "
            f"{row['handoff_height_m']:.3f} m @ {row['handoff_vz_m_s']:.3f} m/s | "
            f"{row['trajectory_z_rmse_m']:.3f} m |"
        )
    lines += [
        "",
        "## Aggregate acceptance results",
        "",
        f"- Native PX4 landing/auto-disarm: {aggregate['native_disarm_count']}/{aggregate['trial_count']}",
        f"- Maximum absolute requested-vs-realized spawn-height error: {aggregate['max_abs_spawn_height_error_m']:.6f} m",
        f"- Maximum terminal XY error: {aggregate['max_final_xy_error_m']:.3f} m",
        f"- Maximum absolute terminal Z error: {aggregate['max_abs_final_z_error_m']:.3f} m",
        f"- Maximum handoff vertical speed: {aggregate['max_handoff_vz_m_s']:.3f} m/s",
        f"- Handoff height range: {aggregate['handoff_height_range_m'][0]:.3f}–{aggregate['handoff_height_range_m'][1]:.3f} m",
        f"- Maximum trajectory Z RMSE: {aggregate['max_trajectory_z_rmse_m']:.3f} m",
        f"- Median stabilization time: {aggregate['median_stabilization_s']:.3f} s",
        f"- Maximum stabilization horizontal excursion: {aggregate['max_stabilization_excursion_m']:.3f} m",
        f"- Maximum stabilization horizontal speed: {aggregate['max_stabilization_horizontal_speed_m_s']:.3f} m/s",
        f"- Maximum stabilization vertical excursion: {aggregate['max_stabilization_vertical_excursion_m']:.3f} m",
        f"- Maximum stabilization vertical speed: {aggregate['max_stabilization_vertical_speed_m_s']:.3f} m/s",
        "",
        "Every trial used the same circle trajectory, returned to the captured airborne X0/Y0,",
        "slowed through the final metre, and required PX4 native landing detection and auto-disarm.",
    ]
    (output / "VALIDATION_REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(aggregate, indent=2, sort_keys=True))
    if aggregate["success_count"] != aggregate["trial_count"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()

