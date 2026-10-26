"""Deterministic smooth geometry for user-drawn LOCAL_NED polylines."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .contracts import PathIntent
from .ordered_fidelity import ordered_fidelity


MAX_CONTROL_POINTS = 20_000
MAX_EXACT_CONTROLS = 512
MAX_VERIFICATION_SAMPLES = 250_000
SMOOTHING_LADDER_STEPS = 6
FIDELITY_REFINEMENT_STEPS = 4
ARC_LENGTH_NODES = 8


def _finite_positive(value: float, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{field} must be finite and positive")
    return result


def _as_tuple(point: np.ndarray) -> tuple[float, ...]:
    return tuple(float(value) for value in point)


def _derivative_curve(
    control_points: np.ndarray,
    knots: np.ndarray,
    degree: int,
) -> tuple[np.ndarray, np.ndarray, int]:
    if degree <= 0:
        return np.zeros((1, control_points.shape[1]), dtype=float), knots, 0
    denominator = knots[degree + 1 : degree + len(control_points)]
    denominator = denominator - knots[1 : len(control_points)]
    if np.any(denominator <= 0.0):
        raise RuntimeError("invalid B-spline derivative knot interval")
    differences = np.diff(control_points, axis=0)
    derivative = degree * differences / denominator[:, None]
    return derivative, knots[1:-1], degree - 1


def _span_index(
    knots: np.ndarray,
    degree: int,
    count: int,
    parameter: float,
) -> tuple[int, float]:
    """Return the knot span owning a parameter, clamped to the curve domain."""
    lower = knots[degree]
    upper = knots[count]
    if parameter <= lower:
        return degree, float(lower)
    if parameter >= upper:
        return count - 1, float(upper)
    span = int(np.searchsorted(knots, parameter, side="right") - 1)
    return min(max(span, degree), count - 1), parameter


def _span_breakpoints(
    knots: np.ndarray,
    degree: int,
    count: int,
) -> np.ndarray:
    """Return the distinct parameter breakpoints spanning the curve domain."""
    return np.unique(knots[degree : count + 1])


def _local_controls(
    control_points: np.ndarray,
    knots: np.ndarray,
    degree: int,
    parameter: float,
) -> np.ndarray:
    """Return the control points whose hull contains the owning span."""
    span, _ = _span_index(knots, degree, len(control_points), parameter)
    return control_points[span - degree : span + 1]


def _local_control_stack(
    control_points: np.ndarray,
    knots: np.ndarray,
    degree: int,
    breakpoints: np.ndarray,
) -> np.ndarray:
    """Return the local control points of every span as one stacked array."""
    middles = 0.5 * (breakpoints[:-1] + breakpoints[1:])
    spans = np.clip(
        np.searchsorted(knots, middles, side="right") - 1,
        degree,
        len(control_points) - 1,
    )
    offsets = np.arange(degree + 1)
    return control_points[spans[:, None] - degree + offsets[None, :]]


def _hull_norm_upper(points: np.ndarray) -> float:
    """Return a valid upper bound for the norm over a convex hull."""
    return float(np.max(np.linalg.norm(points, axis=1)))


def _hull_norm_lower(points: np.ndarray) -> float:
    """Return a valid lower bound for the norm over a convex hull.

    Any unit direction that separates the hull from the origin certifies a
    lower bound, because every hull member projects at least as far along it.
    The hull centroid supplies that direction when one exists; a nonpositive
    result simply means this certificate found none.
    """
    centroid = points.mean(axis=0)
    magnitude = float(np.linalg.norm(centroid))
    if magnitude <= 0.0:
        return 0.0
    direction = centroid / magnitude
    return max(0.0, float(np.min(points @ direction)))


def _de_boor(
    control_points: np.ndarray,
    knots: np.ndarray,
    degree: int,
    parameter: float,
) -> np.ndarray:
    count = len(control_points)
    span, parameter = _span_index(knots, degree, count, parameter)
    work = np.array(
        control_points[span - degree : span + 1],
        dtype=float,
        copy=True,
    )
    for level in range(1, degree + 1):
        for index in range(degree, level - 1, -1):
            left_index = span - degree + index
            denominator = knots[left_index + degree - level + 1] - knots[left_index]
            alpha = (
                0.0
                if denominator <= 0.0
                else (parameter - knots[left_index]) / denominator
            )
            work[index] = (1.0 - alpha) * work[index - 1] + alpha * work[index]
    return work[degree]


@dataclass(frozen=True)
class BSplinePath:
    """A clamped or periodic B-spline over normalized parameter s in [0, 1].

    The control points may have any positive dimension so that a scalar
    schedule, such as a yaw profile, reuses the same validated evaluation and
    convex-hull bounding code as a three-dimensional path.
    """

    control_points: tuple[tuple[float, ...], ...]
    knots: tuple[float, ...]
    degree: int
    periodic: bool

    def __post_init__(self) -> None:
        controls = np.asarray(self.control_points, dtype=float)
        knots = np.asarray(self.knots, dtype=float)
        if controls.ndim != 2 or controls.shape[1] < 1:
            raise ValueError("control_points must be an N by D array with D >= 1")
        if not np.all(np.isfinite(controls)) or not np.all(np.isfinite(knots)):
            raise ValueError("B-spline data must be finite")
        if not 1 <= self.degree < len(controls):
            raise ValueError("B-spline degree is inconsistent with controls")
        if len(knots) != len(controls) + self.degree + 1:
            raise ValueError("B-spline knot count is inconsistent with controls")
        if np.any(np.diff(knots) < 0.0):
            raise ValueError("B-spline knots must be nondecreasing")
        if not math.isclose(knots[self.degree], 0.0, abs_tol=1e-12):
            raise ValueError("B-spline parameter domain must begin at zero")
        if not math.isclose(knots[len(controls)], 1.0, abs_tol=1e-12):
            raise ValueError("B-spline parameter domain must end at one")

    @property
    def dimension(self) -> int:
        return len(self.control_points[0])

    def evaluate(
        self,
        parameter: float,
        derivative: int = 0,
    ) -> tuple[float, ...]:
        if isinstance(parameter, bool) or not isinstance(parameter, (int, float)):
            raise ValueError("parameter must be numeric")
        parameter = float(parameter)
        if not math.isfinite(parameter) or not 0.0 <= parameter <= 1.0:
            raise ValueError("parameter must be finite and in [0, 1]")
        if isinstance(derivative, bool) or not isinstance(derivative, int):
            raise ValueError("derivative must be an integer")
        if not 0 <= derivative <= 3:
            raise ValueError("derivative must be in [0, 3]")
        if derivative > self.degree:
            return (0.0,) * self.dimension

        controls = np.asarray(self.control_points, dtype=float)
        knots = np.asarray(self.knots, dtype=float)
        degree = self.degree
        for _ in range(derivative):
            controls, knots, degree = _derivative_curve(controls, knots, degree)
        return _as_tuple(_de_boor(controls, knots, degree, parameter))

    def derivative_norm_bound(self, derivative: int = 1) -> float:
        """Return a convex-hull upper bound for a derivative norm."""
        _, bounds = self.span_norm_bounds(derivative)
        return max(bounds)

    def span_norm_bounds(
        self,
        derivative: int = 1,
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Return breakpoints and a per-span bound for a derivative norm.

        A derivative curve restricted to one knot span lies in the convex hull
        of that span's local control points, so the largest local control norm
        bounds the derivative over that span alone. Localizing the bound this
        way is what lets timing be fast where the path is straight instead of
        everywhere as slow as its worst corner.
        """
        if not 1 <= derivative <= self.degree:
            raise ValueError("derivative must be within the spline degree")
        controls = np.asarray(self.control_points, dtype=float)
        knots = np.asarray(self.knots, dtype=float)
        breakpoints = _span_breakpoints(knots, self.degree, len(controls))
        degree = self.degree
        for _ in range(derivative):
            controls, knots, degree = _derivative_curve(controls, knots, degree)
        local = _local_control_stack(controls, knots, degree, breakpoints)
        bounds = np.max(np.linalg.norm(local, axis=2), axis=1)
        return (
            tuple(float(value) for value in breakpoints),
            tuple(float(value) for value in bounds),
        )


@dataclass(frozen=True)
class PathFidelityReport:
    accepted: bool
    maximum_allowed_deviation_m: float
    curve_to_stroke_upper_bound_m: float
    stroke_to_curve_upper_bound_m: float
    symmetric_upper_bound_m: float
    curve_sample_count: int
    stroke_sample_count: int
    verification_resolution_m: float


@dataclass(frozen=True)
class GeometryPlan:
    curve: BSplinePath
    source_points: tuple[tuple[float, float, float], ...]
    fidelity: PathFidelityReport
    smoothing: SmoothingReport


@dataclass(frozen=True)
class SmoothingReport:
    requested_control_spacing_m: float
    selected_control_spacing_m: float
    deviation_utilization: float
    deviation_budget_m: float
    control_point_count: int
    source_length_m: float
    curve_length_m: float
    heading_slope_bound_rad: float | None
    soft_target_met: bool = False
    candidates_evaluated: int = 0
    selection_policy: str = "soft-fidelity-target-then-smoothness"
    fallback: str = "none"
    ordered_curve_bound_m: float | None = None


def _deduplicate_consecutive(
    points: np.ndarray,
    tolerance: float,
) -> np.ndarray:
    kept = [points[0]]
    for point in points[1:]:
        if np.linalg.norm(point - kept[-1]) > tolerance:
            kept.append(point)
    return np.asarray(kept, dtype=float)


def _densify_segment(start: np.ndarray, end: np.ndarray, spacing: float) -> list:
    distance = float(np.linalg.norm(end - start))
    intervals = max(1, math.ceil(distance / spacing))
    return [start + (end - start) * (index / intervals) for index in range(intervals)]


def _densify(
    points: np.ndarray,
    spacing: float,
    *,
    periodic: bool,
) -> np.ndarray:
    dense: list[np.ndarray] = []
    pairs = list(zip(points[:-1], points[1:], strict=True))
    if periodic:
        pairs.append((points[-1], points[0]))
    for start, finish in pairs:
        dense.extend(_densify_segment(start, finish, spacing))
    if not periodic:
        dense.append(points[-1])
    result = np.asarray(dense, dtype=float)
    if len(result) > MAX_CONTROL_POINTS:
        raise ValueError(f"normalized path exceeds {MAX_CONTROL_POINTS} control points")
    return result


def _build_spline(
    control_points: np.ndarray,
    *,
    periodic: bool,
    parameters: np.ndarray | None = None,
) -> BSplinePath:
    degree = 5
    if len(control_points) <= degree:
        raise ValueError("quintic geometry requires at least six control points")
    if periodic:
        if parameters is not None:
            raise ValueError("periodic splines use intrinsic uniform parameters")
        original_count = len(control_points)
        # Center the periodic basis at the requested launch seam, then enforce
        # its position while updating repeated controls together. This keeps
        # all periodic derivatives continuous instead of shifting the launch.
        unique = np.roll(control_points, degree // 2, axis=0)
        extended = np.vstack((unique, unique[:degree]))
        knots = (
            np.arange(len(extended) + degree + 1, dtype=float) - degree
        ) / original_count
        weights = _basis_row(knots, degree, len(extended), 0.0)
        folded = np.array(weights[:original_count], copy=True)
        folded[:degree] += weights[original_count:]
        delta = control_points[0] - weights @ extended
        unique = unique + folded[:, None] * delta / float(folded @ folded)
        controls = np.vstack((unique, unique[:degree]))
    else:
        controls = control_points
        if parameters is None:
            chord_lengths = np.linalg.norm(np.diff(controls, axis=0), axis=1)
            parameters = np.concatenate(
                (
                    np.zeros(1),
                    np.cumsum(chord_lengths) / np.sum(chord_lengths),
                )
            )
        elif len(parameters) != len(controls):
            raise ValueError("parameter count must match the control count")
        internal = np.asarray(
            [
                np.mean(parameters[index : index + degree])
                for index in range(1, len(controls) - degree)
            ]
        )
        knots = np.concatenate(
            (
                np.zeros(degree + 1),
                internal,
                np.ones(degree + 1),
            )
        )
    return BSplinePath(
        control_points=tuple(_as_tuple(point) for point in controls),
        knots=tuple(float(value) for value in knots),
        degree=degree,
        periodic=periodic,
    )


def _point_to_segments(point: np.ndarray, polyline: np.ndarray) -> float:
    starts = polyline[:-1]
    vectors = polyline[1:] - starts
    lengths_squared = np.einsum("ij,ij->i", vectors, vectors)
    offsets = point - starts
    fractions = np.divide(
        np.einsum("ij,ij->i", offsets, vectors),
        lengths_squared,
        out=np.zeros_like(lengths_squared),
        where=lengths_squared > 0.0,
    )
    fractions = np.clip(fractions, 0.0, 1.0)
    closest = starts + fractions[:, None] * vectors
    return float(np.min(np.linalg.norm(closest - point, axis=1)))


def _sample_polyline(polyline: np.ndarray, resolution: float) -> np.ndarray:
    samples: list[np.ndarray] = []
    for start, finish in zip(polyline[:-1], polyline[1:], strict=True):
        samples.extend(_densify_segment(start, finish, resolution))
    samples.append(polyline[-1])
    return np.asarray(samples, dtype=float)


def _minimum_sample_distance(
    point: np.ndarray,
    samples: np.ndarray,
    *,
    chunk_size: int = 20_000,
) -> float:
    result = math.inf
    for offset in range(0, len(samples), chunk_size):
        chunk = samples[offset : offset + chunk_size]
        result = min(result, float(np.min(np.linalg.norm(chunk - point, axis=1))))
    return result


def measure_path_fidelity(
    curve: BSplinePath,
    source_points: tuple[tuple[float, float, float], ...],
    *,
    maximum_deviation_m: float,
    verification_resolution_m: float,
) -> PathFidelityReport:
    """Bound sampled symmetric deviation using curve/source motion bounds."""
    resolution = _finite_positive(
        verification_resolution_m, "verification_resolution_m"
    )
    if maximum_deviation_m < 0.0 or not math.isfinite(maximum_deviation_m):
        raise ValueError("maximum_deviation_m must be finite and nonnegative")
    source = np.asarray(source_points, dtype=float)
    if curve.periodic:
        source = np.vstack((source, source[0]))

    speed_bound = curve.derivative_norm_bound()
    intervals = max(1, math.ceil(speed_bound / resolution))
    if intervals + 1 > MAX_VERIFICATION_SAMPLES:
        raise ValueError("curve fidelity check exceeds the sample safety limit")
    parameters = np.linspace(0.0, 1.0, intervals + 1)
    curve_samples = np.asarray([curve.evaluate(value) for value in parameters])
    curve_measured = max(_point_to_segments(point, source) for point in curve_samples)
    curve_upper = curve_measured + speed_bound / (2.0 * intervals)

    source_samples = _sample_polyline(source, resolution)
    if len(source_samples) > MAX_VERIFICATION_SAMPLES:
        raise ValueError("stroke fidelity check exceeds the sample safety limit")
    stroke_measured = max(
        _minimum_sample_distance(point, curve_samples) for point in source_samples
    )
    stroke_upper = stroke_measured + resolution / 2.0
    symmetric = max(curve_upper, stroke_upper)
    return PathFidelityReport(
        accepted=symmetric <= maximum_deviation_m + 1e-12,
        maximum_allowed_deviation_m=maximum_deviation_m,
        curve_to_stroke_upper_bound_m=curve_upper,
        stroke_to_curve_upper_bound_m=stroke_upper,
        symmetric_upper_bound_m=symmetric,
        curve_sample_count=len(curve_samples),
        stroke_sample_count=len(source_samples),
        verification_resolution_m=resolution,
    )


def span_scalar_bounds(
    curve: BSplinePath,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Return breakpoints and per-span value ranges of a scalar spline.

    On one knot span the value is a convex combination of that span's local
    control values, because the basis functions are nonnegative and sum to one
    there, so the local minimum and maximum bracket the value.
    """
    if curve.dimension != 1:
        raise ValueError("scalar bounds require a one-dimensional spline")
    controls = np.asarray(curve.control_points, dtype=float)
    knots = np.asarray(curve.knots, dtype=float)
    breakpoints = _span_breakpoints(knots, curve.degree, len(controls))
    local = _local_control_stack(controls, knots, curve.degree, breakpoints)[:, :, 0]
    return (
        tuple(float(value) for value in breakpoints),
        tuple(float(value) for value in np.min(local, axis=1)),
        tuple(float(value) for value in np.max(local, axis=1)),
    )


def interval_norm_bound(
    breakpoints: tuple[float, ...],
    bounds: tuple[float, ...],
    lower: float,
    upper: float,
) -> float:
    """Return the per-span bound covering every span meeting [lower, upper]."""
    if lower > upper:
        raise ValueError("interval bounds must be ordered")
    selected = [
        bound
        for bound, left, right in zip(
            bounds, breakpoints[:-1], breakpoints[1:], strict=True
        )
        if right >= lower and left <= upper
    ]
    if not selected:
        raise ValueError("interval lies outside the curve domain")
    return max(selected)


def curve_length(curve: BSplinePath, nodes: int = ARC_LENGTH_NODES) -> float:
    """Return the arc length of a curve by per-span Gauss-Legendre quadrature.

    The smoothed curve is shorter than the polyline it replaces, because
    rounding a corner cuts across it, so the flown distance has to be measured
    on the curve rather than inherited from the stroke. Integrating span by
    span keeps the speed norm smooth over each panel even though it is not a
    polynomial, which is what makes so few nodes sufficient.
    """
    if not isinstance(nodes, int) or isinstance(nodes, bool) or nodes < 2:
        raise ValueError("nodes must be an integer of at least two")
    breakpoints = _span_breakpoints(
        np.asarray(curve.knots, dtype=float),
        curve.degree,
        len(curve.control_points),
    )
    abscissae, weights = np.polynomial.legendre.leggauss(nodes)
    left = breakpoints[:-1, None]
    widths = np.diff(breakpoints)[:, None]
    samples = left + 0.5 * widths * (abscissae[None, :] + 1.0)
    speeds = np.asarray(
        [np.linalg.norm(curve.evaluate(float(value), 1)) for value in samples.ravel()]
    ).reshape(samples.shape)
    return float(np.sum(0.5 * widths * weights[None, :] * speeds))


def tangent_heading_slope_bounds(
    curve: BSplinePath,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Bound |d(heading)/ds| on each knot span of a 3-D curve.

    The heading of the horizontal tangent obeys
    ``psi' = (n' e'' - e' n'') / (n'^2 + e'^2)``, so a per-span upper bound on
    the horizontal tangent and curvature terms combined with a per-span lower
    bound on horizontal speed bounds the whole quotient. Both bounds come from
    the convex hull of the local control points of the derivative curves, which
    contains the derivative over that span. A span whose horizontal speed
    cannot be separated from zero is rejected, because the heading itself is
    undefined wherever the horizontal tangent vanishes.
    """
    if curve.dimension != 3:
        raise ValueError("tangent heading requires a three-dimensional curve")
    if curve.degree < 2:
        raise ValueError("tangent heading slope requires a second derivative")
    controls = np.asarray(curve.control_points, dtype=float)
    knots = np.asarray(curve.knots, dtype=float)
    breakpoints = _span_breakpoints(knots, curve.degree, len(controls))
    first = _derivative_curve(controls, knots, curve.degree)
    second = _derivative_curve(*first)
    bounds = []
    for lower, upper in zip(breakpoints[:-1], breakpoints[1:], strict=True):
        middle = 0.5 * (float(lower) + float(upper))
        tangent = _local_controls(*first, middle)[:, :2]
        curvature = _local_controls(*second, middle)[:, :2]
        speed_lower = _hull_norm_lower(tangent)
        if speed_lower <= 0.0:
            raise ValueError(
                "horizontal speed cannot be bounded away from zero near "
                f"parameter {middle:.6f}; tangent yaw is undefined there"
            )
        bounds.append(
            _hull_norm_upper(tangent) * _hull_norm_upper(curvature) / speed_lower**2
        )
    return (
        tuple(float(value) for value in breakpoints),
        tuple(bounds),
    )


def tangent_heading_slope_bound(curve: BSplinePath) -> float:
    """Return the largest per-span bound on |d(heading)/ds|."""
    _, bounds = tangent_heading_slope_bounds(curve)
    return max(bounds)


def _heading_slope_or_none(curve: BSplinePath) -> float | None:
    try:
        return tangent_heading_slope_bound(curve)
    except ValueError:
        return None


def _basis_row(
    knots: np.ndarray,
    degree: int,
    count: int,
    parameter: float,
) -> np.ndarray:
    row = np.zeros(count)
    controls = np.zeros((count, 1))
    for index in range(count):
        controls[index, 0] = 1.0
        row[index] = _de_boor(controls, knots, degree, parameter)[0]
        controls[index, 0] = 0.0
    return row


def _greville(knots: np.ndarray, degree: int, index: int) -> float:
    return float(np.mean(knots[index + 1 : index + 1 + degree]))


def _enforce_exact(
    curve: BSplinePath,
    parameters: tuple[float, ...],
    values: np.ndarray,
) -> BSplinePath:
    """Move the controls so the curve passes through each requested point.

    The fit stays as close as possible to the original controls. Endpoint
    parameters are included by the caller so a clamped curve cannot leave its
    start or finish while an interior point is being pulled onto the stroke.
    """
    controls = np.asarray(curve.control_points, dtype=float)
    knots = np.asarray(curve.knots, dtype=float)
    count, dimension = controls.shape
    basis = np.vstack(
        [_basis_row(knots, curve.degree, count, parameter) for parameter in parameters]
    )
    constraints = len(parameters)
    system = np.block(
        [
            [np.eye(count), basis.T],
            [basis, np.zeros((constraints, constraints))],
        ]
    )
    updated = np.array(controls, copy=True)
    for axis in range(dimension):
        right_hand = np.concatenate((controls[:, axis], values[:, axis]))
        try:
            solution = np.linalg.solve(system, right_hand)
        except np.linalg.LinAlgError as exc:
            raise ValueError("exact fly-through constraints are singular") from exc
        updated[:, axis] = solution[:count]
        if np.max(np.abs(basis @ solution[:count] - values[:, axis])) > 1e-8:
            raise ValueError("exact fly-through constraints were not met")
    return BSplinePath(
        control_points=tuple(_as_tuple(point) for point in updated),
        knots=curve.knots,
        degree=curve.degree,
        periodic=curve.periodic,
    )


def _densify_marked(
    points: np.ndarray,
    spacing: float,
    *,
    periodic: bool,
    exact: np.ndarray,
) -> tuple[np.ndarray, tuple[int, ...]]:
    dense: list[np.ndarray] = []
    marks: list[int] = []
    pairs = list(zip(points[:-1], points[1:], strict=True))
    if periodic:
        pairs.append((points[-1], points[0]))
    for index, (start, finish) in enumerate(pairs):
        if exact[index]:
            marks.append(len(dense))
        dense.extend(_densify_segment(start, finish, spacing))
    if not periodic:
        if exact[-1]:
            marks.append(len(dense))
        dense.append(points[-1])
    if len(dense) > MAX_CONTROL_POINTS:
        raise ValueError(f"normalized path exceeds {MAX_CONTROL_POINTS} control points")
    return np.asarray(dense, dtype=float), tuple(marks)


def build_smooth_path(
    path: PathIntent,
    *,
    control_spacing_m: float,
    verification_resolution_m: float | None = None,
    duplicate_tolerance_m: float = 1e-6,
    deviation_utilization: float = 0.25,
) -> GeometryPlan:
    """Normalize a drawn polyline and fail if smoothing violates its tolerance.

    Search a bounded family, including finer fits than the requested spacing.
    Minimize excess above the soft fidelity goal before favoring a smoother
    candidate. A missed soft goal is explicitly reported; the hard allowance
    is never relaxed. Timing must subsequently verify the chosen geometry.
    """
    if path.kind != "polyline":
        raise ValueError("smooth-path baseline currently accepts polyline intent only")
    if path.closure == "open_depart":
        raise ValueError("open_depart execution is not supported by the baseline")
    spacing = _finite_positive(control_spacing_m, "control_spacing_m")
    duplicate_tolerance = _finite_positive(
        duplicate_tolerance_m, "duplicate_tolerance_m"
    )
    utilization = _finite_positive(deviation_utilization, "deviation_utilization")
    if utilization > 1.0:
        raise ValueError("deviation_utilization must not exceed one")
    internal_hard = [point.point_id for point in path.points[1:-1] if point.hard]
    if internal_hard:
        raise ValueError(
            "internal hard points are not supported by the smoothing baseline: "
            + ", ".join(internal_hard)
        )
    interior_exact = [point.point_id for point in path.points[1:-1] if point.exact]
    if interior_exact and path.closure == "periodic":
        raise ValueError(
            "exact fly-through points are not supported on a periodic path: "
            + ", ".join(interior_exact)
        )
    points = np.asarray(
        [(point.north_m, point.east_m, point.down_m) for point in path.points],
        dtype=float,
    )
    exact_flags = np.asarray([point.exact for point in path.points], dtype=bool)
    original_points = np.array(points, copy=True)
    kept = [0]
    for index in range(1, len(points)):
        if np.linalg.norm(points[index] - points[kept[-1]]) > duplicate_tolerance:
            kept.append(index)
        elif exact_flags[index]:
            raise ValueError("an exact fly-through point duplicates its neighbor")
    points = points[kept]
    exact_flags = exact_flags[kept]
    periodic = path.closure == "periodic"
    if periodic and len(points) > 1:
        if np.linalg.norm(points[-1] - points[0]) <= duplicate_tolerance:
            points = points[:-1]
    if len(points) < 2:
        raise ValueError("path has fewer than two distinct points")
    segment_lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if periodic:
        segment_lengths = np.append(
            segment_lengths,
            np.linalg.norm(points[0] - points[-1]),
        )
    if np.any(segment_lengths <= duplicate_tolerance):
        raise ValueError("path contains unresolved zero-length segments")
    total_length = float(np.sum(segment_lengths))
    resolution = verification_resolution_m
    if resolution is None:
        if path.maximum_deviation_m <= 0.0:
            raise ValueError("zero path deviation requires an exact geometry planner")
        resolution = min(spacing / 20.0, path.maximum_deviation_m / 20.0)
    budget = path.maximum_deviation_m * utilization
    source = tuple(_as_tuple(point) for point in original_points)
    attempt = None
    best = None
    evaluated = 0
    previous_spacing = math.inf
    for step in range(SMOOTHING_LADDER_STEPS, -FIDELITY_REFINEMENT_STEPS - 1, -1):
        candidate_spacing = min(spacing * 2.0 ** (step / 2.0), total_length / 5.0)
        if candidate_spacing >= previous_spacing:
            continue
        previous_spacing = candidate_spacing
        # Densification and the downstream checker have their own hard caps.
        # Do not allocate a candidate that cannot fit the control budget.
        if total_length / candidate_spacing + len(points) + 12 > MAX_CONTROL_POINTS:
            continue
        if (
            interior_exact
            and total_length / candidate_spacing + len(points) + 12 > MAX_EXACT_CONTROLS
        ):
            continue
        dense, marks = _densify_marked(
            points,
            candidate_spacing,
            periodic=periodic,
            exact=exact_flags,
        )
        curve = _build_spline(dense, periodic=periodic)
        interior = [mark for mark in marks if mark not in (0, len(dense) - 1)]
        if interior:
            knots = np.asarray(curve.knots, dtype=float)
            parameters = [0.0]
            values = [dense[0]]
            for mark in interior:
                parameters.append(_greville(knots, curve.degree, mark))
                values.append(dense[mark])
            parameters.append(1.0)
            values.append(dense[-1])
            try:
                curve = _enforce_exact(
                    curve,
                    tuple(parameters),
                    np.asarray(values, dtype=float),
                )
            except ValueError:
                continue
        fidelity = measure_path_fidelity(
            curve,
            source,
            maximum_deviation_m=path.maximum_deviation_m,
            verification_resolution_m=resolution,
        )
        attempt = (candidate_spacing, curve, fidelity)
        evaluated += 1
        if fidelity.accepted:
            try:
                ordered = ordered_fidelity(
                    curve,
                    source,
                    path.maximum_deviation_m,
                    sampling_tolerance=max(budget, path.maximum_deviation_m * 0.25),
                )
            except ValueError:
                continue
            if not ordered["accepted"]:
                continue
            curve_bound = max(
                fidelity.symmetric_upper_bound_m, ordered["upper_bound_m"]
            )
            excess = max(0.0, curve_bound - budget)
            key = (excess, -candidate_spacing)
            if best is None or key < best[0]:
                best = (key, attempt, curve_bound)
            # Spacings strictly decrease. Once excess is zero, every later
            # candidate has key >= (0, -candidate_spacing), so cannot win.
            # Stop only after both symmetric and ordered fidelity passed.
            if excess == 0.0:
                break
    if attempt is None:
        raise ValueError("exact fly-through could not be satisfied")
    candidate_spacing, curve, fidelity = best[1] if best is not None else attempt
    if not fidelity.accepted or best is None:
        raise ValueError(
            "smoothed path exceeds maximum deviation: "
            f"{fidelity.symmetric_upper_bound_m:.6f} m > "
            f"{path.maximum_deviation_m:.6f} m, or traversal order could not be certified within the bounded search"
        )
    return GeometryPlan(
        curve=curve,
        source_points=source,
        fidelity=fidelity,
        smoothing=SmoothingReport(
            requested_control_spacing_m=spacing,
            selected_control_spacing_m=candidate_spacing,
            deviation_utilization=utilization,
            deviation_budget_m=budget,
            control_point_count=len(curve.control_points),
            source_length_m=total_length,
            curve_length_m=curve_length(curve),
            heading_slope_bound_rad=_heading_slope_or_none(curve),
            soft_target_met=best[2] <= budget,
            candidates_evaluated=evaluated,
            fallback=(
                "none"
                if best[2] <= budget
                else "hard-bound-only; mark a corner stop or revise the route for a closer fit"
            ),
            ordered_curve_bound_m=best[2],
        ),
    )
