"""Heading profiles with a certified error.

The flown yaw is a scalar quintic of the normalized path parameter, not the
raw atan2 of the tangent and not a free heading command. A polynomial cannot
reproduce atan2 or an arbitrary requested heading exactly, so the profile is
accepted only when a Lipschitz argument bounds its error. The aircraft is then
commanded to follow the profile, whose own convex-hull derivatives are what
the yaw rate, acceleration, and jerk limits are checked against.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .explain import PlanningError
from .geometry import (
    BSplinePath,
    tangent_heading_slope_bound,
    tangent_heading_slope_bounds,
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


def look_at_heading(
    curve: BSplinePath,
    parameter: float,
    target: tuple[float, float, float],
) -> float:
    """Return the horizontal bearing from the curve to a fixed point."""
    north, east, _ = curve.evaluate(parameter)
    offset_north = target[0] - north
    offset_east = target[1] - east
    if math.hypot(offset_north, offset_east) <= 0.0:
        raise ValueError("look-at heading is undefined at the target")
    return math.atan2(offset_east, offset_north)


def look_at_slope_bound(
    curve: BSplinePath,
    target: tuple[float, float, float],
    *,
    count: int = 2_000,
) -> float:
    """Bound |d(bearing)/ds| by path speed over a certified target distance."""
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError("look-at sample count must be an integer of at least 2")
    speed = curve.derivative_norm_bound(1)
    distances = []
    for coordinate in np.linspace(0.0, 1.0, count):
        north, east, _ = curve.evaluate(float(coordinate))
        distances.append(math.hypot(target[0] - north, target[1] - east))
    lower = min(distances) - 0.5 * speed / (count - 1)
    if lower <= 1e-4:
        raise ValueError("look-at target is not separated from the path")
    return speed / lower


def walked_heading(
    curve: BSplinePath,
    loops: int,
    count: int,
    *,
    heading_at=None,
    slope: float | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Return normalized time and a continuous heading along one or more laps.

    Sampling the raw atan2 jumps by a full turn at every seam. Adding the
    shortest step from one sample to the next removes that jump, provided the
    samples are dense enough that the true heading moves by less than half a
    turn between them.
    """
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError("heading sample count must be an integer of at least 2")
    if heading_at is None:
        heading_at = horizontal_heading
    if slope is None:
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
        raw = heading_at(
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
    return np.concatenate(
        (
            np.zeros(SCHEDULE_DEGREE + 1),
            internal,
            np.ones(SCHEDULE_DEGREE + 1),
        )
    )


def _greville(knots: np.ndarray, count: int) -> np.ndarray:
    return np.asarray(
        [
            float(np.mean(knots[index + 1 : index + SCHEDULE_DEGREE + 1]))
            for index in range(count)
        ]
    )


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
    target: tuple[float, float, float] | None = None,
) -> float:
    """Bound |profile - true heading| over the whole flown parameter.

    On each sample interval the error at the endpoints is known and the error
    derivative is at most the true heading slope plus the profile's own
    derivative bound, so the interval cannot hide more error than half its
    width times that Lipschitz constant. ``target`` selects look-at bearing
    instead of the path tangent.
    """
    if target is None:
        slope = tangent_heading_slope_bound(curve)
        heading_at = None
    else:
        slope = look_at_slope_bound(curve, target)

        def heading_at(path, parameter, point=target):
            return look_at_heading(path, parameter, point)

    normalized, truth = walked_heading(
        curve, loops, count, heading_at=heading_at, slope=slope
    )
    return _bounded_profile_error(normalized, truth, profile, slope * loops, count)


def _bounded_profile_error(normalized, truth, profile, truth_slope, count):
    predicted = np.asarray([profile.evaluate(float(value))[0] for value in normalized])
    point_error = float(np.max(np.abs(predicted - truth)))
    slope = truth_slope + profile.derivative_norm_bound(1)
    return point_error + 0.5 * slope / (count - 1)


def adaptive_heading_error_bound(curve, profile, *, loops, target=None):
    """Refine steep spans locally; every gap retains a Lipschitz remainder."""
    if target is None:
        truth_breaks, truth_bounds = tangent_heading_slope_bounds(curve)
        truth_breaks = np.asarray(
            [
                *((lap + b) / loops for lap in range(loops) for b in truth_breaks[:-1]),
                1.0,
            ]
        )
        truth_bounds = np.tile(np.asarray(truth_bounds) * loops, loops)
    else:
        truth_breaks = np.asarray([0.0, 1.0])
        truth_bounds = np.asarray([look_at_slope_bound(curve, target) * loops])
    profile_breaks, profile_bounds = profile.span_norm_bounds(1)
    profile_breaks, profile_bounds = (
        np.asarray(profile_breaks),
        np.asarray(profile_bounds),
    )
    breaks = np.unique(np.concatenate((truth_breaks, profile_breaks)))
    coordinates = [0.0]
    remainder = 0.0
    target_remainder = math.radians(0.25)
    for a, b in zip(breaks, breaks[1:], strict=False):
        middle = (a + b) / 2.0
        ti = min(
            len(truth_bounds) - 1,
            np.searchsorted(truth_breaks, middle, side="right") - 1,
        )
        pi = min(
            len(profile_bounds) - 1,
            np.searchsorted(profile_breaks, middle, side="right") - 1,
        )
        slope = float(truth_bounds[ti] + profile_bounds[pi])
        count = max(1, math.ceil(slope * (b - a) / (2 * target_remainder)))
        if len(coordinates) + count > 32_000:
            raise ValueError("heading certificate refinement budget exceeded")
        coordinates.extend(a + (b - a) * i / count for i in range(1, count + 1))
        remainder = max(remainder, slope * (b - a) / (2 * count))
    previous, unwrapped, error = None, 0.0, 0.0
    for coordinate in coordinates:
        parameter = curve_parameter(curve, float(coordinate) * loops, loops)
        raw = (
            horizontal_heading(curve, parameter)
            if target is None
            else look_at_heading(curve, parameter, target)
        )
        unwrapped = raw if previous is None else unwrapped + _wrap(raw - previous)
        previous = raw
        error = max(error, abs(profile.evaluate(float(coordinate))[0] - unwrapped))
    return error + remainder


@dataclass(frozen=True)
class YawTrack:
    """A flyable heading profile and the certificate on its tangent error."""

    profile: BSplinePath
    error_bound_rad: float
    bounds: PathBounds
    loops: int


def _plan_heading(
    curve: BSplinePath,
    *,
    loops: int,
    heading_at,
    slope: float,
    failure: str,
) -> YawTrack:
    if isinstance(loops, bool) or not isinstance(loops, int) or not 1 <= loops <= 20:
        raise ValueError("loops must be an integer in [1, 20]")
    if loops != 1 and not curve.periodic:
        raise ValueError("only a periodic path can be repeated")
    count = HEADING_SAMPLE_LADDER[-1]
    best_error = math.inf
    normalized, heading = walked_heading(
        curve, loops, count, heading_at=heading_at, slope=slope
    )
    for intervals in HEADING_INTERVAL_LADDER:
        profile = _fit_heading(normalized, heading, intervals)
        error = _bounded_profile_error(
            normalized, heading, profile, slope * loops, count
        )
        best_error = min(best_error, error)
        if error <= MAXIMUM_HEADING_ERROR_RAD:
            return YawTrack(
                profile=profile,
                error_bound_rad=error,
                bounds=PathBounds.of(profile),
                loops=loops,
            )
    # A steep corner can make the gap-between-samples remainder dominate.
    # Refine the certificate rather than relaxing its two-degree threshold.
    error = adaptive_heading_error_bound(
        curve, profile, loops=loops, target=getattr(heading_at, "target", None)
    )
    best_error = min(best_error, error)
    if error <= MAXIMUM_HEADING_ERROR_RAD:
        return YawTrack(
            profile=profile,
            error_bound_rad=error,
            bounds=PathBounds.of(profile),
            loops=loops,
        )
    raise PlanningError(
        f"{failure} cannot be followed within "
        f"{math.degrees(MAXIMUM_HEADING_ERROR_RAD):.1f} degrees; best certified "
        f"error is {math.degrees(best_error):.2f} degrees",
        limiting_constraint="yaw_tracking",
    )


def plan_tangent_yaw(curve: BSplinePath, *, loops: int = 1) -> YawTrack:
    """Fit the coarsest heading profile whose certified error is inside 2 degrees."""
    return _plan_heading(
        curve,
        loops=loops,
        heading_at=None,
        slope=tangent_heading_slope_bound(curve),
        failure="tangent heading",
    )


def explicit_heading_error_bound(profile: BSplinePath, samples, *, count: int) -> float:
    """Bound |profile - requested heading| for an explicit, unwrapped profile.

    The request is the piecewise-linear heading through the supplied samples.
    Its Lipschitz constant is the steepest segment. The same half-interval
    remainder used for tangent and look-at yaw covers the gaps between samples.
    ``samples`` are already in normalized path parameter, so lap count does not
    scale this slope.
    """
    if isinstance(count, bool) or not isinstance(count, int) or count < 2:
        raise ValueError("heading sample count must be an integer of at least 2")
    parameters = np.asarray([sample.parameter for sample in samples], dtype=float)
    heading = np.radians(
        np.asarray([sample.yaw_deg for sample in samples], dtype=float)
    )
    slope = float(np.max(np.abs(np.diff(heading) / np.diff(parameters))))
    normalized = np.linspace(0.0, 1.0, count)
    truth = np.interp(normalized, parameters, heading)
    predicted = np.asarray([profile.evaluate(float(value))[0] for value in normalized])
    point_error = float(np.max(np.abs(predicted - truth)))
    lipschitz = slope + profile.derivative_norm_bound(1)
    return point_error + 0.5 * lipschitz / (count - 1)


def plan_explicit_yaw(samples) -> YawTrack:
    """Fit a heading to requested samples and accept it inside the 2 degree tube."""
    parameters = np.asarray([sample.parameter for sample in samples], dtype=float)
    heading = np.radians(
        np.asarray([sample.yaw_deg for sample in samples], dtype=float)
    )
    count = HEADING_SAMPLE_LADDER[-1]
    normalized = np.linspace(0.0, 1.0, count)
    requested = np.interp(normalized, parameters, heading)
    best_error = math.inf
    for intervals in HEADING_INTERVAL_LADDER:
        profile = _fit_heading(normalized, requested, intervals)
        error = explicit_heading_error_bound(profile, samples, count=count)
        best_error = min(best_error, error)
        if error <= MAXIMUM_HEADING_ERROR_RAD:
            return YawTrack(
                profile=profile,
                error_bound_rad=error,
                bounds=PathBounds.of(profile),
                loops=1,
            )
    raise PlanningError(
        "explicit heading cannot be followed within "
        f"{math.degrees(MAXIMUM_HEADING_ERROR_RAD):.1f} degrees; best certified "
        f"error is {math.degrees(best_error):.2f} degrees",
        limiting_constraint="yaw_tracking",
    )


def plan_look_at_yaw(
    curve: BSplinePath,
    target: tuple[float, float, float],
    *,
    loops: int = 1,
) -> YawTrack:
    """Fit a heading that looks at one fixed point, within the same 2 degree tube."""
    slope = look_at_slope_bound(curve, target)

    def heading_at(path: BSplinePath, parameter: float) -> float:
        return look_at_heading(path, parameter, target)

    heading_at.target = target  # type: ignore[attr-defined]
    return _plan_heading(
        curve,
        loops=loops,
        heading_at=heading_at,
        slope=slope,
        failure="look-at heading",
    )
