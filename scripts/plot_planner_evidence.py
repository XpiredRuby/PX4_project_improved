"""Render retained display tracks; numerical acceptance uses the full evidence."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def render(source, output):
    data = json.loads(Path(source).read_text())
    fig, axes = plt.subplots(1, len(data["cases"]), figsize=(10, 5.8))
    fig.subplots_adjust(top=.77, bottom=.24, left=.08, right=.97, wspace=.3)
    fig.suptitle("Gazebo: planned route and independent truth", x=.08,
                 y=.965, ha="left", fontsize=18, fontweight="bold", color="#18313f")
    fig.text(.08, .895, "PX4 x500  |  Requested 1× speed  |  19 October 2026  |  SITL_ONLY",
             fontsize=10, color="#496473")
    for ax, case in zip(axes, data["cases"], strict=True):
        stroke = case["intent"]["path"]["points"]
        planned, truth = case["planned"], case["flown_truth"]
        ax.plot([p["east_m"] for p in stroke], [p["north_m"] for p in stroke],
                color="#9ba8af", lw=3, alpha=.65, label="Input stroke")
        ax.plot([p["y"] for p in planned], [p["x"] for p in planned],
                color="#087f8c", lw=1.7, label="Verified plan")
        ax.plot([p["y"] for p in truth], [p["x"] for p in truth],
                color="#a34d91", lw=1.3, linestyle="--", label="Simulator truth")
        ax.scatter([planned[0]["y"]], [planned[0]["x"]], marker="+", s=100,
                   color="#18313f", linewidths=1.8, zorder=5)
        ax.set_title("Drawn route" if case["name"].startswith("drawn") else "Circle",
                     loc="left", fontsize=13, fontweight="bold", pad=12)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("East / m", fontsize=10)
        ax.set_ylabel("North / m", fontsize=10)
        ax.grid(color="#dce5e9", lw=.7)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)
        ax.spines[["bottom", "left"]].set_color("#9aadb7")
        ax.tick_params(labelsize=9, colors="#496473")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower left", bbox_to_anchor=(.07, .11),
               ncol=3, frameon=False, fontsize=10)
    fig.text(.08, .08, "Same scale on each route's north/east axes. Launch marked +. Display tracks are decimated.",
             fontsize=9, color="#496473")
    fig.text(.08, .035, "Sampled simulation evidence. Full telemetry and ULog hashes are retained; physical flight is not approved.",
             fontsize=9, color="#496473")
    fig.savefig(output, metadata={"Date": None})
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    render(args.source, args.output)
