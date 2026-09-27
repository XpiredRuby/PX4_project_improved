#!/usr/bin/env python3

import csv
import json
import math
import statistics
import sys
from pathlib import Path


MAX_STABILIZATION_EXCURSION_M = 2.0
MAX_STABILIZATION_HORIZONTAL_SPEED_M_S = 2.0
MAX_STABILIZATION_VERTICAL_EXCURSION_M = 1.0
MAX_STABILIZATION_VERTICAL_SPEED_M_S = 1.5


def number(row, name):
    try:
        return float(row[name])
    except (KeyError, TypeError, ValueError):
        return math.nan


def rmse(values):
    finite = [value for value in values if math.isfinite(value)]
    return math.sqrt(statistics.fmean(value * value for value in finite))


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: analyze_randomized_run.py ARCHIVE")
    archive = Path(sys.argv[1]).resolve()
    logs = sorted(archive.glob("research_log_*.csv"))
    if len(logs) != 1:
        raise RuntimeError(f"expected one research log in {archive}, found {len(logs)}")

    with logs[0].open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    metadata = json.loads((archive / "spawn_config.json").read_text(encoding="utf-8"))
    controller_log = (archive / "controller.log").read_text(
        encoding="utf-8", errors="replace"
    )
    px4_log_path = archive / "px4-sitl.log"
    px4_log = (
        px4_log_path.read_text(encoding="utf-8", errors="replace")
        if px4_log_path.exists()
        else ""
    )
    native_landing_confirmed = (
        "[runner] PX4 native landing/disarm confirmed" in controller_log
        and "Disarmed by landing" in px4_log
    )
    controller = metadata["controller"]
    home = controller["airborne_local_origin"]
    ground_z = float(controller["predicted_ground_local_z"])

    trajectory = [row for row in rows if row["phase"] == "TRAJECTORY"]
    landing = [row for row in rows if row["phase"] == "LAND"]
    done = [row for row in rows if row["phase"] == "DONE"]
    stabilize = [row for row in rows if row["phase"] == "STABILIZE"]
    if not trajectory or not landing or not done:
        raise RuntimeError("run does not contain TRAJECTORY, LAND, and DONE phases")

    handoff = done[0]
    final = rows[-1]
    trajectory_errors = {
        axis: [number(row, axis) - number(row, f"desired_{axis}") for row in trajectory]
        for axis in ("x", "y", "z")
    }
    handoff_xy = math.hypot(
        number(handoff, "x") - float(home["x"]),
        number(handoff, "y") - float(home["y"]),
    )
    final_xy = math.hypot(
        number(final, "x") - float(home["x"]),
        number(final, "y") - float(home["y"]),
    )
    landing_vz = [number(row, "vz") for row in landing]
    landing_cmd_vz = [number(row, "cmd_vz") for row in landing]
    transitions = {
        row["phase_transition"]: number(row, "elapsed_s")
        for row in rows
        if row.get("phase_transition")
    }

    metrics = {
        "archive": str(archive),
        "acceptance_limits": {
            "max_stabilization_excursion_m": MAX_STABILIZATION_EXCURSION_M,
            "max_stabilization_horizontal_speed_m_s": (
                MAX_STABILIZATION_HORIZONTAL_SPEED_M_S
            ),
            "max_stabilization_vertical_excursion_m": (
                MAX_STABILIZATION_VERTICAL_EXCURSION_M
            ),
            "max_stabilization_vertical_speed_m_s": (
                MAX_STABILIZATION_VERTICAL_SPEED_M_S
            ),
        },
        "rows": len(rows),
        "requested_spawn": metadata["requested"],
        "realized_spawn_height_m": controller["spawn_height_m"],
        "ground_target_source": controller["ground_target_source"],
        "ground_reference_error_m": controller["ground_reference_error_m"],
        "phase_transition_elapsed_s": transitions,
        "stabilization": {
            "duration_s": transitions.get("STABILIZE->TRAJECTORY"),
            "max_horizontal_excursion_m": max(
                math.hypot(
                    number(row, "x") - float(home["x"]),
                    number(row, "y") - float(home["y"]),
                )
                for row in stabilize
            ),
            "max_horizontal_speed_m_s": max(
                math.hypot(number(row, "vx"), number(row, "vy"))
                for row in stabilize
            ),
            "max_vertical_excursion_m": max(
                abs(number(row, "z") - float(home["z"])) for row in stabilize
            ),
            "max_abs_vertical_speed_m_s": max(
                abs(number(row, "vz")) for row in stabilize
            ),
            "max_abs_roll_deg": math.degrees(
                max(abs(number(row, "roll")) for row in stabilize)
            ),
            "max_abs_pitch_deg": math.degrees(
                max(abs(number(row, "pitch")) for row in stabilize)
            ),
        },
        "trajectory": {
            "rows": len(trajectory),
            "x_rmse_m": rmse(trajectory_errors["x"]),
            "y_rmse_m": rmse(trajectory_errors["y"]),
            "z_rmse_m": rmse(trajectory_errors["z"]),
        },
        "landing": {
            "rows": len(landing),
            "handoff_xy_error_m": handoff_xy,
            "handoff_height_above_target_m": ground_z - number(handoff, "z"),
            "handoff_vz_m_s": number(handoff, "vz"),
            "max_measured_descent_m_s": max(landing_vz),
            "max_commanded_descent_m_s": max(landing_cmd_vz),
            "final_xy_error_m": final_xy,
            "final_z_error_to_ground_reference_m": number(final, "z") - ground_z,
            "final_vz_m_s": number(final, "vz"),
            "final_armed": final.get("armed"),
            "native_landing_disarm_confirmed": native_landing_confirmed,
        },
    }
    metrics["success"] = bool(
        native_landing_confirmed
        and handoff_xy <= 0.30
        and -0.10 <= metrics["landing"]["handoff_height_above_target_m"] <= 0.30
        and abs(metrics["landing"]["handoff_vz_m_s"]) <= 0.25
        and abs(metrics["landing"]["final_z_error_to_ground_reference_m"]) <= 0.20
        and metrics["stabilization"]["max_horizontal_excursion_m"]
        <= MAX_STABILIZATION_EXCURSION_M
        and metrics["stabilization"]["max_horizontal_speed_m_s"]
        <= MAX_STABILIZATION_HORIZONTAL_SPEED_M_S
        and metrics["stabilization"]["max_vertical_excursion_m"]
        <= MAX_STABILIZATION_VERTICAL_EXCURSION_M
        and metrics["stabilization"]["max_abs_vertical_speed_m_s"]
        <= MAX_STABILIZATION_VERTICAL_SPEED_M_S
    )

    output = archive / "randomized_metrics.json"
    output.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metrics, indent=2, sort_keys=True))
    if not metrics["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
