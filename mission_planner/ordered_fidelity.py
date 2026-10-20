"""Bounded, order-aware geometric acceptance with a separate spline evaluator.

No planner geometry/timing primitives are imported. Cox basis evaluation and
derivative-control hulls construct a polygonal approximation with a continuous
error bound. A monotone discrete Frechet coupling certifies traversal order.
This is a second geometric method, not independent aircraft-model validation.
"""

from bisect import bisect_right
import csv
import math

import numpy as np

MAX_NODES = 4096
MAX_CELLS = 4_000_000
SEARCH_PASSES = 7


class SplineAudit:
    def __init__(self, curve):
        self.controls = tuple(tuple(float(x) for x in p) for p in curve.control_points)
        self.knots = tuple(float(x) for x in curve.knots)
        self.degree = curve.degree
        if not all(math.isfinite(x) for p in self.controls for x in p):
            raise ValueError("non-finite spline audit input")

    def value(self, u):
        """Evaluate via nonzero Cox basis functions, rather than de Boor points."""
        p, knots = self.degree, self.knots
        span = min(len(self.controls) - 1, max(p, bisect_right(knots, u) - 1))
        basis = [1.0] + [0.0] * p
        left, right = [0.0] * (p + 1), [0.0] * (p + 1)
        for j in range(1, p + 1):
            left[j], right[j] = u - knots[span + 1 - j], knots[span + j] - u
            saved = 0.0
            for r in range(j):
                den = right[r + 1] + left[j - r]
                term = basis[r] / den if den else 0.0
                basis[r], saved = saved + right[r + 1] * term, left[j - r] * term
            basis[j] = saved
        return tuple(
            sum(basis[j] * self.controls[span - p + j][axis] for j in range(p + 1))
            for axis in range(len(self.controls[0]))
        )

    def second_derivative_bound(self):
        controls, knots, degree = self.controls, self.knots, self.degree
        for _ in range(2):
            if degree == 0:
                return 0.0
            derivative = []
            for i in range(len(controls) - 1):
                den = knots[i + degree + 1] - knots[i + 1]
                derivative.append(
                    tuple(
                        degree * (b - a) / den if den else 0.0
                        for a, b in zip(controls[i], controls[i + 1], strict=True)
                    )
                )
            controls, knots, degree = tuple(derivative), knots[1:-1], degree - 1
        return max(math.sqrt(sum(v * v for v in p)) for p in controls) * (1 + 1e-10)


def _curve_polygon(audit, tolerance):
    second = audit.second_derivative_bound()
    nodes = {0.0: audit.value(0.0), 1.0: audit.value(1.0)}
    stack = [(0.0, 1.0, 0)]
    error = 0.0
    while stack:
        a, b, depth = stack.pop()
        remainder = second * (b - a) ** 2 / 8.0
        if (
            math.dist(nodes[a], nodes[b]) <= tolerance * 0.5
            and remainder <= tolerance * 0.1
        ):
            error = max(error, remainder)
            continue
        if len(nodes) >= MAX_NODES or depth >= 24:
            raise ValueError(
                "ordered fidelity refinement budget exceeded; simplify the route"
            )
        middle = (a + b) / 2.0
        nodes[middle] = audit.value(middle)
        stack.extend(((middle, b, depth + 1), (a, middle, depth + 1)))
    return np.asarray([nodes[u] for u in sorted(nodes)]), error


def _source_polygon(source, spacing):
    result = []
    for a, b in zip(source, source[1:], strict=False):
        intervals = max(1, math.ceil(math.dist(a, b) / spacing))
        if len(result) + intervals + 1 > MAX_NODES:
            raise ValueError(
                "ordered fidelity source budget exceeded; simplify the route"
            )
        result.extend(
            tuple(x + (y - x) * i / intervals for x, y in zip(a, b, strict=True))
            for i in range(intervals)
        )
    result.append(tuple(source[-1]))
    return np.asarray(result)


def coupling_within(distances, radius):
    """Monotone coupling decision using vectorized horizontal reachability."""
    columns = np.arange(distances.shape[1])
    previous = np.zeros(distances.shape[1], dtype=bool)
    for i, row in enumerate(distances):
        valid = row <= radius
        seeds = previous.copy()
        seeds[1:] |= previous[:-1]
        if i == 0:
            seeds[0] = True
        run_start = np.maximum.accumulate(np.where(~valid, columns + 1, 0))
        last_seed = np.maximum.accumulate(np.where(valid & seeds, columns, -1))
        previous = valid & (last_seed >= run_start)
        if not previous.any():
            return False
    return bool(previous[-1])


def ordered_fidelity(curve, source, tolerance, *, sampling_tolerance=None):
    if not math.isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("ordered fidelity requires a positive finite allowance")
    source = tuple(tuple(float(x) for x in p) for p in source)
    if len(source) < 2 or any(not math.isfinite(x) for p in source for x in p):
        raise ValueError("ordered fidelity requires a finite source polyline")
    if curve.periodic and math.dist(source[0], source[-1]) > 1e-12:
        source = (*source, source[0])
    sampling = tolerance if sampling_tolerance is None else sampling_tolerance
    if not math.isfinite(sampling) or sampling <= 0.0:
        raise ValueError("ordered fidelity requires a positive sampling allowance")
    polygon, error = _curve_polygon(SplineAudit(curve), sampling)
    original = _source_polygon(source, sampling * 0.5)
    if len(polygon) * len(original) > MAX_CELLS:
        raise ValueError(
            "ordered fidelity coupling budget exceeded; simplify the route"
        )
    # Allocate a bounded matrix without a second n*m*3 temporary.
    distances = np.empty((len(polygon), len(original)))
    for i, point in enumerate(polygon):
        distances[i] = np.linalg.norm(original - point, axis=1)
    upper = max(0.0, tolerance - error)
    accepted = coupling_within(distances, upper)
    if accepted:
        low = 0.0
        for _ in range(SEARCH_PASSES):
            middle = (low + upper) / 2.0
            if coupling_within(distances, middle):
                upper = middle
            else:
                low = middle
    return dict(
        accepted=accepted,
        upper_bound_m=upper + error if accepted else None,
        curve_approximation_bound_m=error,
        curve_nodes=len(polygon),
        source_nodes=len(original),
        cells=len(polygon) * len(original),
        method="monotone discrete Frechet + bounded continuous spline approximation",
        checker_scope="geometry and order; shared aircraft-model assumptions remain",
    )


def emitted_geometry_agrees(curve, candidate, tolerance=1e-7):
    """Compare every emitted position using the separate basis implementation."""
    audit = SplineAudit(curve)
    for point in candidate.points:
        u = point.path_parameter
        if not math.isfinite(u):
            return False
        if curve.periodic:
            u = 1.0 if u == candidate.summary.loop_count else u % 1.0
        if not 0.0 <= u <= 1.0:
            return False
        actual = (point.north_m, point.east_m, point.down_m)
        if (
            any(not math.isfinite(v) for v in actual)
            or math.dist(audit.value(u), actual) > tolerance
        ):
            return False
    return bool(candidate.points)


def audit_controller_csv(path, candidate):
    """Re-read the serialized artifact; check every field, count and clock."""
    fields = {
        "time": "time_s",
        "x": "north_m",
        "y": "east_m",
        "z": "down_m",
        "yaw": "yaw_rad",
        "vx": "north_velocity_m_s",
        "vy": "east_velocity_m_s",
        "vz": "down_velocity_m_s",
        "yaw_rate": "yaw_rate_rad_s",
        "ax": "north_acceleration_m_s2",
        "ay": "east_acceleration_m_s2",
        "az": "down_acceleration_m_s2",
        "jx": "north_jerk_m_s3",
        "jy": "east_jerk_m_s3",
        "jz": "down_jerk_m_s3",
        "yaw_acceleration": "yaw_acceleration_rad_s2",
        "yaw_jerk": "yaw_jerk_rad_s3",
    }
    with open(path, encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        if set(reader.fieldnames or ()) != {*fields, "planner_contract_version"}:
            raise ValueError(
                "serialized trajectory fields disagree with the checked reference"
            )
        previous = -math.inf
        count = 0
        for count, row in enumerate(reader, start=1):
            if count > len(candidate.points):
                raise ValueError("serialized trajectory has extra rows")
            point = candidate.points[count - 1]
            if any(
                not math.isfinite(float(row[k]))
                or float(row[k]) != getattr(point, name)
                for k, name in fields.items()
            ):
                raise ValueError(
                    "serialized trajectory differs from the checked reference"
                )
            if float(row["planner_contract_version"]) != 1 or point.time_s <= previous:
                raise ValueError("serialized trajectory version or clock is invalid")
            previous = point.time_s
        if count != len(candidate.points) or count < 2:
            raise ValueError("serialized trajectory is missing reference rows")
    return dict(
        accepted=True,
        rows=count,
        method="re-read CSV; all reference fields and strict timestamps checked",
    )
