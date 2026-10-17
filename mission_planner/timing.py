"""Locally time-optimal timing for verified drawn paths.

A single global time law has to be slow enough everywhere for the worst point
on the path, so a mission that is mostly straight still pays for its tightest
corner and for the peak-to-mean ratio of whatever smooth ramp it uses. This
module instead represents the path parameter as a monotone quintic schedule in
normalized time, which keeps the exact-recomputation property the verifier
relies on while letting the pace vary along the path.

Timing is split into a proposal stage and an acceptance stage. The proposal
stage builds a schedule shape from a pointwise pace ceiling, and its quality
affects only how fast the mission is. The acceptance stage derives the duration
from certified per-span convex-hull bounds on both the path and the schedule,
so a bad proposal produces a slow mission rather than an unsafe one.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .geometry import (
    BSplinePath,
    GeometryPlan,
    interval_norm_bound,
    span_scalar_bounds,
)


SCHEDULE_DEGREE = 5
CONSTRAINT_NAMES = ("speed", "acceleration", "jerk")
# The planner inverts its certified bounds for the duration, and the verifier
# recomputes those bounds at that duration. Both roundings are far below this
# factor, so reporting a slightly longer duration keeps the recomputed bounds
# strictly inside the limits at a cost of nanoseconds of flight time.
DURATION_SAFETY_FACTOR = 1.0 + 1e-9
# Repeating the leading and trailing control values this many times forces the
# schedule and its first three derivatives to vanish at the endpoints, because
# a B-spline derivative at a clamped end depends only on that many controls.
STATIONARY_CONTROLS = 4
# A pace that vanishes like s**(3/4) at each end makes the path parameter leave
# and reach its endpoints like t**4, which is exactly the order needed for
# velocity, acceleration, and jerk to vanish there while duration stays finite.
RAMP_EXPONENT = 0.75
MIN_SCHEDULE_INTERVALS = 8
MAX_SCHEDULE_INTERVALS = 2_000
# Candidate grids. Every candidate is scored by the same certified acceptance
# requirement, so these only decide how fast the accepted mission is.
SCHEDULE_INTERVAL_LADDER = (128, 256, 512)
PACE_WINDOW_LADDER = (3.0, 6.0, 12.0)
ACCELERATION_SHARE_LADDER = (0.3, 0.5, 0.7)
EQUALIZATION_PASSES = 12
EQUALIZATION_DAMPING = 0.5
EQUALIZATION_FLOOR = 0.25
RAMP_FRACTION_RANGE = (0.01, 0.4)
SHAPE_RAMP_SAMPLES = 400
SHAPE_CRUISE_SAMPLES = 2_000
PACE_SAMPLES = 2_000
PACE_SMOOTHING_PASSES = 3


@dataclass(frozen=True)
class PathBounds:
    """Certified per-span bounds on the path derivatives."""

    breakpoints: tuple[float, ...]
    first: tuple[float, ...]
    second: tuple[float, ...]
    third: tuple[float, ...]

    @classmethod
    def of(cls, curve: BSplinePath) -> "PathBounds":
        breakpoints, first = curve.span_norm_bounds(1)
        _, second = curve.span_norm_bounds(2)
        _, third = curve.span_norm_bounds(3)
        return cls(
            breakpoints=breakpoints,
            first=first,
            second=second,
            third=third,
        )

    def over(self, lower: float, upper: float) -> tuple[float, float, float]:
        return (
            interval_norm_bound(self.breakpoints, self.first, lower, upper),
            interval_norm_bound(self.breakpoints, self.second, lower, upper),
            interval_norm_bound(self.breakpoints, self.third, lower, upper),
        )


@dataclass(frozen=True)
class TimeLawReport:
    schedule_intervals: int
    control_point_count: int
    pace_window_spans: float
    cruise_acceleration_share: float
    ramp_fraction: float
    equalization_passes: int
    candidates_evaluated: int
    minimum_requirement_s: float
    limiting_constraint: str
    limiting_normalized_time: float
    speed_limit_m_s: float
    acceleration_limit_m_s2: float
    jerk_limit_m_s3: float
    straight_line_duration_s: float


@dataclass(frozen=True)
class TimeLaw:
    """Monotone path-parameter schedule s(t / duration_s) with its evidence."""

    schedule: BSplinePath
    report: TimeLawReport

    def sample(
        self,
        normalized_time: float,
    ) -> tuple[float, float, float, float]:
        """Return the path parameter and its first three tau derivatives."""
        parameter = min(max(self.schedule.evaluate(normalized_time)[0], 0.0), 1.0)
        return (
            parameter,
            self.schedule.evaluate(normalized_time, 1)[0],
            self.schedule.evaluate(normalized_time, 2)[0],
            self.schedule.evaluate(normalized_time, 3)[0],
        )


def _positive(value: float, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be finite and positive")
    return result


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


def _span_values(
    breakpoints: tuple[float, ...],
    values: tuple[float, ...],
    positions: np.ndarray,
) -> np.ndarray:
    index = np.searchsorted(np.asarray(breakpoints), positions, side="right") - 1
    return np.asarray(values)[np.clip(index, 0, len(values) - 1)]


def _cumulative_trapezoid(axis: np.ndarray, values: np.ndarray) -> np.ndarray:
    return np.concatenate((
        np.zeros(1),
        np.cumsum(0.5 * np.diff(axis) * (values[:-1] + values[1:])),
    ))


def _windowed(values: np.ndarray, window: int, reduce) -> np.ndarray:
    pad = window // 2
    padded = np.pad(values, pad, mode="edge")
    strided = np.lib.stride_tricks.sliding_window_view(padded, 2 * pad + 1)
    return reduce(strided, axis=1)


@dataclass(frozen=True)
class PaceProfile:
    """A smooth, conservative ds/dt ceiling sampled over the path parameter."""

    positions: np.ndarray
    ceiling: np.ndarray

    def at(self, positions: np.ndarray) -> np.ndarray:
        return np.interp(positions, self.positions, self.ceiling)


def _pace_blur(schedule_intervals: int, window_spans: float) -> int:
    """Return a blur width measured in whole schedule spans.

    Smoothing narrower than a schedule span cannot be represented by the fitted
    schedule, and smoothing much wider than the pace varies throws away pace
    the bounds do allow, so the width is tied to the span count.
    """
    return max(
        3, int(round(PACE_SAMPLES * window_spans / schedule_intervals))
    )


def _pace_profile(
    bounds: PathBounds,
    *,
    blur: int,
    speed_limit_m_s: float,
    lateral_acceleration_m_s2: float,
    lateral_jerk_m_s3: float,
) -> PaceProfile:
    """Return a smoothed ds/dt ceiling that never exceeds the raw ceiling.

    The raw ceiling is piecewise constant because it comes from per-span
    bounds, and fitting a schedule through its steps leaves third-derivative
    noise that acceptance charges for heavily. Taking a running minimum over a
    window at least as wide as the whole blur support before blurring keeps
    every smoothed value at or below the raw ceiling at the same parameter, so
    smoothing cannot invent pace the bounds do not allow.
    """
    positions = np.linspace(0.0, 1.0, PACE_SAMPLES + 1)
    first = _span_values(bounds.breakpoints, bounds.first, positions)
    second = _span_values(bounds.breakpoints, bounds.second, positions)
    third = _span_values(bounds.breakpoints, bounds.third, positions)
    with np.errstate(divide="ignore", invalid="ignore"):
        ceiling = np.minimum.reduce([
            speed_limit_m_s / first,
            np.where(
                second > 0.0,
                np.sqrt(lateral_acceleration_m_s2 / second),
                np.inf,
            ),
            np.where(
                third > 0.0, np.cbrt(lateral_jerk_m_s3 / third), np.inf
            ),
        ])
    if not np.all(np.isfinite(ceiling)) or np.any(ceiling <= 0.0):
        raise ValueError("path derivative bounds admit no positive pace")
    smoothed = _windowed(ceiling, PACE_SMOOTHING_PASSES * blur + 1, np.min)
    for _ in range(PACE_SMOOTHING_PASSES):
        smoothed = _windowed(smoothed, blur, np.mean)
    return PaceProfile(positions=positions, ceiling=smoothed)


def _equalize(
    pace: PaceProfile,
    schedule: BSplinePath,
    requirements: tuple[float, ...],
    blur: int,
) -> PaceProfile:
    """Slow the pace wherever acceptance charges more than it does elsewhere.

    The duration is the worst span requirement, so it falls only when the
    binding spans are slowed and the slack spans take up the difference. Each
    parameter is therefore scaled down by the worst requirement of any schedule
    span that can reach it, which drives the requirements towards each other
    instead of towards the pointwise ceiling the first pass assumed.
    """
    _, lower, upper = span_scalar_bounds(schedule)
    reference = max(requirements) * EQUALIZATION_FLOOR
    weight = np.full_like(pace.ceiling, reference)
    for value, left, right in zip(requirements, lower, upper, strict=True):
        window = (pace.positions >= left) & (pace.positions <= right)
        weight[window] = np.maximum(weight[window], max(value, reference))
    factor = (float(np.min(weight)) / weight) ** EQUALIZATION_DAMPING
    factor = _windowed(factor, PACE_SMOOTHING_PASSES * blur + 1, np.min)
    for _ in range(PACE_SMOOTHING_PASSES):
        factor = _windowed(factor, blur, np.mean)
    return PaceProfile(
        positions=pace.positions,
        ceiling=pace.ceiling * factor,
    )


def _shape_samples(
    pace: PaceProfile,
    ramp_fraction: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return monotone (normalized time, path parameter) shape samples.

    The elapsed-time integrand diverges where the pace ramp vanishes, so each
    ramp is integrated in a substituted variable that cancels the singularity
    exactly instead of being sampled through it.
    """
    ramp = ramp_fraction
    # Writing the ramp parameter as r * x**power makes the ramp factor exactly
    # x**(power - 1), which cancels the Jacobian and leaves a bounded
    # integrand, so the elapsed-time quadrature never samples the singularity.
    power = 1.0 / (1.0 - RAMP_EXPONENT)
    axis = np.linspace(0.0, 1.0, SHAPE_RAMP_SAMPLES + 1)

    entry_parameter = ramp * axis**power
    entry_time = _cumulative_trapezoid(
        axis, power * ramp / pace.at(entry_parameter)
    )

    cruise_parameter = np.linspace(ramp, 1.0 - ramp, SHAPE_CRUISE_SAMPLES + 1)
    cruise_time = entry_time[-1] + _cumulative_trapezoid(
        cruise_parameter, 1.0 / pace.at(cruise_parameter)
    )

    exit_parameter = 1.0 - ramp * (1.0 - axis) ** power
    exit_time = cruise_time[-1] + _cumulative_trapezoid(
        axis, power * ramp / pace.at(exit_parameter)
    )

    parameter = np.concatenate((
        entry_parameter, cruise_parameter[1:], exit_parameter[1:]
    ))
    elapsed = np.concatenate((entry_time, cruise_time[1:], exit_time[1:]))
    if not np.all(np.isfinite(elapsed)) or elapsed[-1] <= 0.0:
        raise ValueError("shape integration produced a degenerate time scale")
    parameter = np.maximum.accumulate(np.clip(parameter, 0.0, 1.0))
    elapsed = np.maximum.accumulate(elapsed)
    return elapsed / elapsed[-1], parameter


def _fit_schedule(
    normalized_time: np.ndarray,
    parameter: np.ndarray,
    intervals: int,
) -> BSplinePath:
    """Fit a monotone clamped quintic schedule with stationary endpoints."""
    knots = _clamped_knots(intervals)
    count = intervals + SCHEDULE_DEGREE
    abscissae = _greville(knots, count)
    values = np.interp(abscissae, normalized_time, parameter)
    values[:STATIONARY_CONTROLS] = 0.0
    values[-STATIONARY_CONTROLS:] = 1.0
    values = np.clip(np.maximum.accumulate(values), 0.0, 1.0)
    return BSplinePath(
        control_points=tuple((float(value),) for value in values),
        knots=tuple(float(value) for value in knots),
        degree=SCHEDULE_DEGREE,
        periodic=False,
    )


def schedule_defects(schedule: BSplinePath) -> tuple[str, ...]:
    """Return every reason a schedule is not an acceptable time law.

    The time law is planner output rather than a fixed formula, so acceptance
    has to confirm its structural claims instead of assuming them. Monotonicity
    and the [0, 1] range follow from the control values alone, because on each
    span the value is a convex combination of that span's local controls.
    """
    defects: list[str] = []
    if schedule.degree != SCHEDULE_DEGREE:
        defects.append(f"degree must be {SCHEDULE_DEGREE}")
    if schedule.periodic:
        defects.append("schedule must not be periodic")
    if schedule.dimension != 1:
        defects.append("schedule must be scalar")
    knots = np.asarray(schedule.knots, dtype=float)
    multiplicity = schedule.degree + 1
    if not (
        np.all(knots[:multiplicity] == 0.0)
        and np.all(knots[-multiplicity:] == 1.0)
    ):
        defects.append("schedule must be clamped at both ends")
    values = np.asarray(schedule.control_points, dtype=float)[:, 0]
    if len(values) < 2 * STATIONARY_CONTROLS:
        defects.append("schedule has too few control values")
        return tuple(defects)
    if np.any(np.diff(values) < 0.0):
        defects.append("schedule control values must be nondecreasing")
    if np.any(values[:STATIONARY_CONTROLS] != 0.0) or np.any(
        values[-STATIONARY_CONTROLS:] != 1.0
    ):
        defects.append("schedule must be stationary through jerk at both ends")
    return tuple(defects)


def _span_terms(bounds: PathBounds, schedule: BSplinePath) -> np.ndarray:
    """Return per-span duration-free numerators for speed, acceleration, jerk.

    Each schedule span is paired with the path bounds over only the parameter
    range that span can reach, so a straight stretch is never charged for a
    corner it does not visit. Dividing the three rows by the duration, its
    square, and its cube gives certified bounds at any duration, which is why
    the planner and the independent verifier can share this one computation.
    """
    _, lower, upper = span_scalar_bounds(schedule)
    _, first_rate = schedule.span_norm_bounds(1)
    _, second_rate = schedule.span_norm_bounds(2)
    _, third_rate = schedule.span_norm_bounds(3)
    reach = np.asarray(bounds.breakpoints)
    overlapping = (reach[None, 1:] >= np.asarray(lower)[:, None]) & (
        reach[None, :-1] <= np.asarray(upper)[:, None]
    )
    if not np.all(np.any(overlapping, axis=1)):
        raise ValueError("schedule reaches outside the path domain")

    def reachable(values: tuple[float, ...]) -> np.ndarray:
        return np.max(
            np.where(overlapping, np.asarray(values)[None, :], -np.inf), axis=1
        )

    path_first = reachable(bounds.first)
    path_second = reachable(bounds.second)
    path_third = reachable(bounds.third)
    rate = np.asarray(first_rate)
    acceleration = np.asarray(second_rate)
    jerk = np.asarray(third_rate)
    terms = np.stack([
        path_first * rate,
        path_second * rate**2 + path_first * acceleration,
        path_third * rate**3
        + 3.0 * path_second * rate * acceleration
        + path_first * jerk,
    ])
    if not np.all(np.isfinite(terms)) or np.any(terms < 0.0):
        raise ValueError("schedule admits no finite derivative bounds")
    return terms


def certified_dynamics(
    bounds: PathBounds,
    schedule: BSplinePath,
    duration_s: float,
) -> tuple[float, float, float]:
    """Return certified speed, acceleration, and jerk bounds at a duration."""
    duration = _positive(duration_s, "duration_s")
    terms = _span_terms(bounds, schedule)
    powers = np.asarray([duration, duration**2, duration**3])
    return tuple(
        float(value) for value in np.max(terms / powers[:, None], axis=1)
    )


def duration_requirements(
    bounds: PathBounds,
    schedule: BSplinePath,
    *,
    speed_limit_m_s: float,
    acceleration_limit_m_s2: float,
    jerk_limit_m_s3: float,
) -> tuple[tuple[float, ...], tuple[str, ...]]:
    """Return the shortest duration each schedule span can be flown in.

    Inverting each certified bound for the duration that makes it meet its
    limit turns the span numerators into durations, and the span that needs the
    longest sets the mission duration.
    """
    terms = _span_terms(bounds, schedule)
    limits = np.asarray([
        speed_limit_m_s, acceleration_limit_m_s2, jerk_limit_m_s3
    ])
    roots = np.asarray([1.0, 1.0 / 2.0, 1.0 / 3.0])
    stacked = (terms / limits[:, None]) ** roots[:, None]
    if not np.all(np.isfinite(stacked)):
        raise ValueError("schedule admits no finite duration requirement")
    # Exact ties resolve to the earliest name so reports stay reproducible.
    selected = np.argmax(stacked, axis=0)
    requirements = stacked[selected, np.arange(stacked.shape[1])]
    if float(np.max(requirements)) <= 0.0:
        raise ValueError("schedule admits no positive duration requirement")
    return (
        tuple(float(value) for value in requirements),
        tuple(CONSTRAINT_NAMES[index] for index in selected),
    )


def _worst_span(
    requirements: tuple[float, ...],
    limiting: tuple[str, ...],
    schedule: BSplinePath,
) -> tuple[float, str, float]:
    """Return the binding requirement, its constraint, and where it binds."""
    breakpoints, _, _ = span_scalar_bounds(schedule)
    index = max(
        range(len(requirements)), key=lambda value: requirements[value]
    )
    return (
        requirements[index] * DURATION_SAFETY_FACTOR,
        limiting[index],
        0.5 * (breakpoints[index] + breakpoints[index + 1]),
    )


def duration_requirement(
    bounds: PathBounds,
    schedule: BSplinePath,
    *,
    speed_limit_m_s: float,
    acceleration_limit_m_s2: float,
    jerk_limit_m_s3: float,
) -> tuple[float, str, float]:
    """Return the duration, binding constraint, and where it binds."""
    requirements, limiting = duration_requirements(
        bounds,
        schedule,
        speed_limit_m_s=speed_limit_m_s,
        acceleration_limit_m_s2=acceleration_limit_m_s2,
        jerk_limit_m_s3=jerk_limit_m_s3,
    )
    return _worst_span(requirements, limiting, schedule)


@dataclass(frozen=True)
class _Candidate:
    requirement_s: float
    intervals: int
    window_spans: float
    share: float
    ramp_fraction: float
    limiting: str
    limiting_normalized_time: float
    schedule: BSplinePath

    def key(self) -> tuple:
        return (
            self.requirement_s,
            self.intervals,
            self.window_spans,
            self.share,
        )


def plan_time_law(
    geometry: GeometryPlan,
    *,
    speed_limit_m_s: float,
    acceleration_limit_m_s2: float,
    jerk_limit_m_s3: float,
    schedule_intervals: tuple[int, ...] | None = None,
    pace_window_spans: tuple[float, ...] | None = None,
    cruise_acceleration_shares: tuple[float, ...] | None = None,
    equalization_passes: int = EQUALIZATION_PASSES,
) -> TimeLaw:
    """Search a fixed grid of schedule shapes and keep the fastest feasible one.

    Each candidate is scored by the same certified per-span requirement that
    acceptance uses, so widening the grid can only find a faster mission and
    never a less conservative one.
    """
    speed_limit = _positive(speed_limit_m_s, "speed_limit_m_s")
    acceleration_limit = _positive(
        acceleration_limit_m_s2, "acceleration_limit_m_s2"
    )
    jerk_limit = _positive(jerk_limit_m_s3, "jerk_limit_m_s3")
    if geometry.curve.periodic:
        raise ValueError(
            "periodic execution requires a separately planned entry and exit"
        )
    if isinstance(equalization_passes, bool) or not isinstance(
        equalization_passes, int
    ):
        raise ValueError("equalization_passes must be an integer")
    if not 1 <= equalization_passes <= 100:
        raise ValueError("equalization_passes must be in [1, 100]")
    intervals_grid = (
        SCHEDULE_INTERVAL_LADDER
        if schedule_intervals is None
        else schedule_intervals
    )
    window_grid = (
        PACE_WINDOW_LADDER if pace_window_spans is None else pace_window_spans
    )
    share_grid = (
        ACCELERATION_SHARE_LADDER
        if cruise_acceleration_shares is None
        else cruise_acceleration_shares
    )
    if not intervals_grid or not window_grid or not share_grid:
        raise ValueError("timing search grids must not be empty")
    for value in intervals_grid:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("schedule_intervals must contain integers")
        if not MIN_SCHEDULE_INTERVALS <= value <= MAX_SCHEDULE_INTERVALS:
            raise ValueError(
                "schedule_intervals must be in "
                f"[{MIN_SCHEDULE_INTERVALS}, {MAX_SCHEDULE_INTERVALS}]"
            )
    for value in window_grid:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0.0
        ):
            raise ValueError("pace_window_spans must be finite and positive")
    for value in share_grid:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0.0 < value < 1.0
        ):
            raise ValueError("cruise_acceleration_shares must be in (0, 1)")

    bounds = PathBounds.of(geometry.curve)
    length = geometry.smoothing.curve_length_m
    limits = {
        "speed_limit_m_s": speed_limit,
        "acceleration_limit_m_s2": acceleration_limit,
        "jerk_limit_m_s3": jerk_limit,
    }
    best: _Candidate | None = None
    evaluated = 0
    for share in share_grid:
        # The ramp must be long enough in path parameter to reach cruise speed
        # under the share of acceleration reserved for changing speed, or the
        # schedule asks for a step that acceptance simply charges for.
        ramp_fraction = min(
            max(
                speed_limit**2 / (acceleration_limit * share * length),
                RAMP_FRACTION_RANGE[0],
            ),
            RAMP_FRACTION_RANGE[1],
        )
        for window_spans in window_grid:
            for intervals in intervals_grid:
                blur = _pace_blur(intervals, window_spans)
                pace = _pace_profile(
                    bounds,
                    blur=blur,
                    speed_limit_m_s=speed_limit,
                    lateral_acceleration_m_s2=acceleration_limit
                    * (1.0 - share),
                    lateral_jerk_m_s3=jerk_limit * (1.0 - share),
                )
                for _ in range(equalization_passes):
                    schedule = _fit_schedule(
                        *_shape_samples(pace, ramp_fraction), intervals
                    )
                    requirements, limiting_names = duration_requirements(
                        bounds, schedule, **limits
                    )
                    worst, limiting, normalized_time = _worst_span(
                        requirements, limiting_names, schedule
                    )
                    candidate = _Candidate(
                        requirement_s=worst,
                        intervals=intervals,
                        window_spans=window_spans,
                        share=share,
                        ramp_fraction=ramp_fraction,
                        limiting=limiting,
                        limiting_normalized_time=normalized_time,
                        schedule=schedule,
                    )
                    evaluated += 1
                    if best is None or candidate.key() < best.key():
                        best = candidate
                    pace = _equalize(pace, schedule, requirements, blur)
    return TimeLaw(
        schedule=best.schedule,
        report=TimeLawReport(
            schedule_intervals=best.intervals,
            control_point_count=len(best.schedule.control_points),
            pace_window_spans=best.window_spans,
            cruise_acceleration_share=best.share,
            ramp_fraction=best.ramp_fraction,
            equalization_passes=equalization_passes,
            candidates_evaluated=evaluated,
            minimum_requirement_s=best.requirement_s,
            limiting_constraint=best.limiting,
            limiting_normalized_time=best.limiting_normalized_time,
            speed_limit_m_s=speed_limit,
            acceleration_limit_m_s2=acceleration_limit,
            jerk_limit_m_s3=jerk_limit,
            straight_line_duration_s=length / speed_limit,
        ),
    )
