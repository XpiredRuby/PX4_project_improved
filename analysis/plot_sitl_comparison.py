"""Plot measured wind comparisons, including both repeats and tradeoffs."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("metrics", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rows = json.loads(args.metrics.read_text())
    prefixes = ("gust-final-baseline-", "gust-prediction-only-", "gust-final-full-")
    groups = [[r for r in rows if r["case"].startswith(p)] for p in prefixes]
    if any(len(g) != 2 for g in groups):
        raise ValueError("Expected two repeats for each comparison configuration")
    metrics = (
        ("Physical XY tracking RMS (m)", lambda r: r["truth"]["physical_xy_tracking_error_m"]["rms"]),
        ("Peak physical XY tracking error (m)", lambda r: r["truth"]["physical_xy_tracking_error_m"]["max_abs"]),
        ("Velocity-command XY jerk, p95 (m/s³)", lambda r: r["command_jerk_xy"]["p95"]),
        ("Command acceleration effort (m²/s³)", lambda r: r["effort"]),
    )
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), constrained_layout=True)
    colours = ("#687782", "#4776a5", "#500000")
    for ax, (title, value) in zip(axes.flat, metrics, strict=True):
        samples = [[value(r) for r in g] for g in groups]
        ax.bar(range(3), [np.mean(s) for s in samples], color=colours, alpha=.85)
        for i, sample in enumerate(samples):
            ax.scatter([i - .06, i + .06], sample, c="black", s=25, zorder=3)
        ax.set_xticks(range(3), ["Baseline", "Prediction", "Prediction +\nslowdown"])
        ax.set_title(title, fontsize=11)
        ax.grid(axis="y", alpha=.2)
        ax.set_axisbelow(True)
    duration = [np.mean([r["duration_s"] for r in g]) for g in groups]
    fig.suptitle("Matched Gazebo gust trials: two repeats per setting", fontsize=15)
    fig.supxlabel("Bars: repeat means; dots: individual trials. Route durations: "
                  + ", ".join(f"{d:.1f} s" for d in duration)
                  + ". Effort is a command proxy, not battery energy.", fontsize=9)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    plt.close(fig)


if __name__ == "__main__":
    main()
