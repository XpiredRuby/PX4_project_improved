"""Operator review page for an accepted SITL plan.

The page shows the route, speed, heading, and the limits that set the
duration. It is a review of a simulation-only artifact. It has no control
that dismisses a hard constraint, and it never describes the plan as safe
to fly.
"""

from __future__ import annotations

import html
import math
from pathlib import Path

from .retiming import CandidateTrajectory


def _polyline(points: list[tuple[float, float]], width: float, height: float) -> str:
    if len(points) < 2:
        return ""
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    minimum_x, maximum_x = min(xs), max(xs)
    minimum_y, maximum_y = min(ys), max(ys)
    span_x = max(maximum_x - minimum_x, 1e-6)
    span_y = max(maximum_y - minimum_y, 1e-6)
    pad = 16.0

    def place(x: float, y: float) -> tuple[float, float]:
        return (
            pad + (x - minimum_x) / span_x * (width - 2 * pad),
            height - pad - (y - minimum_y) / span_y * (height - 2 * pad),
        )

    coordinates = " ".join(
        f"{place(x, y)[0]:.2f},{place(x, y)[1]:.2f}" for x, y in points
    )
    return (
        f'<polyline fill="none" stroke="#0b3a5b" stroke-width="2" '
        f'points="{coordinates}"/>'
    )


def render_review(summary: dict, candidate: CandidateTrajectory) -> str:
    """Return a self-contained review document for one accepted plan."""
    points = candidate.points
    route = _polyline(
        [(point.east_m, point.north_m) for point in points], 640, 360
    )
    speed = _polyline(
        [
            (
                point.time_s,
                math.hypot(
                    point.north_velocity_m_s,
                    point.east_velocity_m_s,
                    point.down_velocity_m_s,
                ),
            )
            for point in points
        ],
        640,
        180,
    )
    timing = summary["retiming"]
    heading = timing.get("heading_error_bound_rad")
    heading_text = (
        "fixed heading"
        if heading is None
        else f"{math.degrees(heading):.2f} degrees from the path tangent"
    )
    limitations = "".join(
        f"<li>{html.escape(item)}</li>" for item in summary["limitations"]
    )
    duration = timing["selected_duration_s"]
    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>SITL plan review</title>
<style>
body {{ font-family: sans-serif; margin: 2rem; color: #14202b; }}
.banner {{ background: #fff4d6; border: 1px solid #8a6a00; padding: 1rem; }}
figure {{ margin: 1.5rem 0; }}
svg {{ background: #f4f7f8; border: 1px solid #d0d7de; }}
</style>
</head>
<body>
<p class="banner"><strong>SITL_ONLY.</strong> This review is not a clearance to fly.
Hard-constraint failures are refusals; this page has no control that dismisses them.</p>
<h1>Plan review</h1>
<p>Duration {duration:.2f} s, limited by {html.escape(str(timing["limiting_constraint"]))}.</p>
<ul>
<li>Speed bound {timing["conservative_speed_bound_m_s"]:.3f} m/s of {timing["speed_limit_m_s"]:.3f} m/s</li>
<li>Acceleration bound {timing["conservative_acceleration_bound_m_s2"]:.3f} m/s²</li>
<li>Jerk bound {timing["conservative_jerk_bound_m_s3"]:.3f} m/s³</li>
<li>Tilt bound {timing["tilt_bound_deg"]:.2f} degrees</li>
<li>Thrust {timing["minimum_thrust_bound_n"]:.2f} to {timing["maximum_thrust_bound_n"]:.2f} N</li>
<li>Body-rate bound {timing["body_rate_bound_deg_s"]:.2f} degrees/s</li>
<li>Heading: {html.escape(heading_text)}</li>
<li>Laps {timing["loop_count"]}, hard-point stops {timing["segment_count"]}</li>
</ul>
<figure><figcaption>East and north, metres</figcaption>
<svg viewBox="0 0 640 360" width="640" height="360">{route}</svg></figure>
<figure><figcaption>Speed against time</figcaption>
<svg viewBox="0 0 640 180" width="640" height="180">{speed}</svg></figure>
<h2>Limitations</h2>
<ul>{limitations}</ul>
</body>
</html>
"""


def write_review(
    path: str | Path,
    summary: dict,
    candidate: CandidateTrajectory,
) -> None:
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    try:
        temporary.write_text(render_review(summary, candidate), encoding="utf-8")
        temporary.replace(destination)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
