"""Tangent-yaw profiles with a certified heading error.

The flown yaw is a scalar quintic of normalized mission time's path parameter,
not the raw atan2 of the tangent. A polynomial cannot reproduce atan2 exactly,
so the profile is accepted only when a Lipschitz argument bounds its error
against the true horizontal heading. The aircraft is then commanded to follow
the profile, whose own convex-hull derivatives are what the yaw rate,
acceleration, and jerk limits are checked against.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .explain import PlanningError
from .geometry import (
    BSplinePath,
    tangent_heading_slope_bound,
)
from .timing import PathBounds, SCHEDULE_DEGREE


MAXIMUM_HEADING_ERROR_RAD = math.radians(2.0)
HEADING_INTERVAL_LADDER = (64, 128, 256, 512)
HEADING_SAMPLE_LADDER = (2_000, 8_000)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def horizontal_heading(curve: BSplinePath, parameter: float) -> float:
    """Return the compass heading of the horizontal tangent, in radians."""
    north, east, _ = curve.evaluate(parameter, 1)
    if math.hypot(north, east) <= 0.0:
        raise ValueError(
            "tangent yaw is undefined where the horizontal tangent vanishes"
        )
    return math.atan2(east, north)


def curve_parameter(curve: BSplinePath, unwrapped: float, loops: int) -> float:
    """Map an unwrapped lap coordinate back onto the curve domain."""
    if not curve.periodic:
        return min(max(unwrapped, 0.0), 1.0)
    if unwrapped <= 0.0:
        return 0.0
    if unwrapped >= loops:
        return 1.0
    wrapped = math.fmod(unwrapped, 1.0)
    if wrapped < 1e-15:
        return 0.0
    return wrapped


def walked_heading(
    curve: BSplinePath,
    loops: int,
    count: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Return normalized time and a continuous heading along one or more laps.

    Sampling the raw atan2 jumps by a full turn at every seam. Adding the
    shortest step from one sample to the next removes that jump, provided the
    samples are dense enough that the true heading moves by less than half a
    turn between them.
    """
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError("heading sample count must be an integer of at least 2")
    slope = tangent_heading_slope_bound(curve)
    step = loops / (count - 1)
    if slope * step >= math.pi:
        raise ValueError(
            "heading samples are too coarse to unwrap the tangent uniquely"
        )
    normalized = np.linspace(0.0, 1.0, count)
    heading = np.empty(count)
    previous_raw = None
    unwrapped = 0.0
    for index, coordinate in enumerate(normalized):
        raw = horizontal_heading(
            curve, curve_parameter(curve, float(coordinate) * loops, loops)
        )
        if previous_raw is None:
            unwrapped = raw
        else:
            unwrapped += _wrap(raw - previous_raw)
        previous_raw = raw
        heading[index] = unwrapped
    return normalized, heading


def _clamped_knots(intervals: int) -> np.ndarray:
    internal = np.arange(1, intervals, dtype=float) / intervals
    return np.concatenate((
        np.zeros(SCHEDULE_DEGREE + 1),
        internal,
        np.ones(SCHEDULE_DEGREE + 1),
    ))


def _greville(knots: np.ndarray, count: int) -> np.ndarray:
    return np.asarray([
        float(np.mean(knots[index + 1 : index + SCHEDULE_DEGREE + 1]))
        for index in range(count)
    ])


def _fit_heading(
    normalized: np.ndarray,
    heading: np.ndarray,
    intervals: int,
) -> BSplinePath:
    knots = _clamped_knots(intervals)
    count = intervals + SCHEDULE_DEGREE
    values = np.interp(_greville(knots, count), normalized, heading)
    return BSplinePath(
        control_points=tuple((float(value),) for value in values),
        knots=tuple(float(value) for value in knots),
        degree=SCHEDULE_DEGREE,
        periodic=False,
    )


def heading_error_bound(
    curve: BSplinePath,
    profile: BSplinePath,
    *,
    loops: int,
    count: int,
) -> float:
    """Bound |profile - true heading| over the whole flown parameter.

    On each sample interval the error at the endpoints is known and the error
    derivative is at most the true heading slope plus the profile's own
    derivative bound, so the interval cannot hide more error than half its
    width times that Lipschitz constant.
    """
    normalized, truth = walked_heading(curve, loops, count)
    predicted = np.asarray([
        profile.evaluate(float(value))[0] for value in normalized
    ])
    point_error = float(np.max(np.abs(predicted - truth)))
    slope = tangent_heading_slope_bound(curve) * loops
    slope += profile.derivative_norm_bound(1)
    return point_error + 0.5 * slope / (count - 1)


@dataclass(frozen=True)
class YawTrack:
    """A flyable heading profile and the certificate on its tangent error."""

    profile: BSplinePath
    error_bound_rad: float
    bounds: PathBounds
    loops: int


def plan_tangent_yaw(curve: BSplinePath, *, loops: int = 1) -> YawTrack:
    """Fit the coarsest heading profile whose certified error is inside 2 degrees."""
    if isinstance(loops, bool) or not isinstance(loops, int) or not 1 <= loops <= 20:
        raise ValueError("loops must be an integer in [1, 20]")
    if loops != 1 and not curve.periodic:
        raise ValueError("only a periodic path can be repeated")
    count = HEADING_SAMPLE_LADDER[-1]
    best_error = math.inf
    for intervals in HEADING_INTERVAL_LADDER:
        normalized, heading = walked_heading(curve, loops, count)
        profile = _fit_heading(normalized, heading, intervals)
        error = heading_error_bound(curve, profile, loops=loops, count=count)
        best_error = min(best_error, error)
        if error <= MAXIMUM_HEADING_ERROR_RAD:
            return YawTrack(
                profile=profile,
                error_bound_rad=error,
                bounds=PathBounds.of(profile),
                loops=loops,
            )
    raise PlanningError(
        "tangent heading cannot be followed within "
        f"{math.degrees(MAXIMUM_HEADING_ERROR_RAD):.1f} degrees; best certified "
        f"error is {math.degrees(best_error):.2f} degrees",
        limiting_constraint="yaw_tracking",
    )
