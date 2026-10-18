"""Hashed occupancy grid, inflation, and a deterministic corridor.

The map is an input artifact. GPS and IMU do not create one. Clearance is
certified by a sampled distance to occupied cells minus a Lipschitz remainder,
after the grid has been kept at least the declared clearance and localization
radius away from those cells. A corridor is a 4-connected path on that
inflated grid, so every segment between cell centers stays inside free cells.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math

import numpy as np

from .contracts import CoordinateFrame, canonical_sha256
from .explain import PlanningError
from .geometry import BSplinePath


MAX_MAP_CELLS = 128 * 128


def _number(value, field: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    if minimum is not None and result < minimum:
        raise ValueError(f"{field} must be at least {minimum}")
    return result


@dataclass(frozen=True)
class OccupancyMap:
    schema_version: int
    map_id: str
    frame: CoordinateFrame
    origin_north_m: float
    origin_east_m: float
    resolution_m: float
    rows: int
    columns: int
    occupied: tuple[bool, ...]
    localization_radius_m: float

    @classmethod
    def from_dict(cls, raw: dict) -> "OccupancyMap":
        if not isinstance(raw, dict):
            raise ValueError("occupancy map must be an object")
        allowed = {
            "schema_version",
            "map_id",
            "frame",
            "origin_north_m",
            "origin_east_m",
            "resolution_m",
            "rows",
            "columns",
            "occupied",
            "localization_radius_m",
        }
        unknown = set(raw) - allowed
        if unknown:
            raise ValueError(
                "occupancy map has unknown fields: " + ", ".join(sorted(unknown))
            )
        missing = allowed - set(raw)
        if missing:
            raise ValueError(
                "occupancy map is missing fields: " + ", ".join(sorted(missing))
            )
        if raw["schema_version"] != 1:
            raise ValueError("occupancy map schema_version must be 1")
        rows = raw["rows"]
        columns = raw["columns"]
        if (
            isinstance(rows, bool)
            or not isinstance(rows, int)
            or isinstance(columns, bool)
            or not isinstance(columns, int)
            or rows < 1
            or columns < 1
            or rows * columns > MAX_MAP_CELLS
        ):
            raise ValueError("occupancy map must be an integer grid of at most 16384 cells")
        occupied_text = raw["occupied"]
        if not isinstance(occupied_text, str) or len(occupied_text) != rows * columns:
            raise ValueError("occupied must be a 0/1 string covering every cell")
        if any(character not in "01" for character in occupied_text):
            raise ValueError("occupied must contain only 0 and 1")
        return cls(
            schema_version=1,
            map_id=str(raw["map_id"]),
            frame=CoordinateFrame.from_dict(raw["frame"]),
            origin_north_m=_number(raw["origin_north_m"], "origin_north_m"),
            origin_east_m=_number(raw["origin_east_m"], "origin_east_m"),
            resolution_m=_number(raw["resolution_m"], "resolution_m", minimum=1e-3),
            rows=rows,
            columns=columns,
            occupied=tuple(character == "1" for character in occupied_text),
            localization_radius_m=_number(
                raw["localization_radius_m"],
                "localization_radius_m",
                minimum=0.0,
            ),
        )

    def to_dict(self) -> dict:
        return {
            "schema_version": self.schema_version,
            "map_id": self.map_id,
            "frame": {
                "name": self.frame.name,
                "units": self.frame.units,
                "origin_id": self.frame.origin_id,
                "altitude_datum": self.frame.altitude_datum,
            },
            "origin_north_m": self.origin_north_m,
            "origin_east_m": self.origin_east_m,
            "resolution_m": self.resolution_m,
            "rows": self.rows,
            "columns": self.columns,
            "occupied": "".join("1" if cell else "0" for cell in self.occupied),
            "localization_radius_m": self.localization_radius_m,
        }

    @property
    def sha256(self) -> str:
        return canonical_sha256(self)

    def grid(self) -> np.ndarray:
        return np.asarray(self.occupied, dtype=bool).reshape(self.rows, self.columns)

    def contains(self, north: float, east: float) -> bool:
        row = (north - self.origin_north_m) / self.resolution_m
        column = (east - self.origin_east_m) / self.resolution_m
        return 0.0 <= row < self.rows and 0.0 <= column < self.columns

    def cell(self, north: float, east: float) -> tuple[int, int]:
        if not self.contains(north, east):
            raise PlanningError(
                "a corridor point lies outside the occupancy map",
                limiting_constraint="map",
            )
        return (
            int((north - self.origin_north_m) / self.resolution_m),
            int((east - self.origin_east_m) / self.resolution_m),
        )

    def center(self, row: int, column: int) -> tuple[float, float]:
        return (
            self.origin_north_m + (row + 0.5) * self.resolution_m,
            self.origin_east_m + (column + 0.5) * self.resolution_m,
        )


def _rectangle_distance(
    north: np.ndarray,
    east: np.ndarray,
    row: np.ndarray,
    column: np.ndarray,
    *,
    origin_north: float,
    origin_east: float,
    resolution: float,
) -> np.ndarray:
    north0 = origin_north + row * resolution
    east0 = origin_east + column * resolution
    north_gap = np.maximum(north0 - north, 0.0) + np.maximum(
        north - (north0 + resolution), 0.0
    )
    east_gap = np.maximum(east0 - east, 0.0) + np.maximum(
        east - (east0 + resolution), 0.0
    )
    return np.hypot(north_gap, east_gap)


def distance_to_occupied(
    occupancy: OccupancyMap,
    north: np.ndarray,
    east: np.ndarray,
) -> np.ndarray:
    """Return the distance from each query point to the nearest occupied cell."""
    grid = occupancy.grid()
    rows, columns = np.nonzero(grid)
    if len(rows) == 0:
        return np.full(len(north), math.inf)
    distances = _rectangle_distance(
        north[:, None],
        east[:, None],
        rows[None, :],
        columns[None, :],
        origin_north=occupancy.origin_north_m,
        origin_east=occupancy.origin_east_m,
        resolution=occupancy.resolution_m,
    )
    return np.min(distances, axis=1)


def clearance_margin_m(
    curve: BSplinePath,
    occupancy: OccupancyMap,
    *,
    clearance_m: float,
) -> float:
    """Lower-bound the clearance remaining after localization error.

    Samples are spaced by the curve's speed bound, and half a sample step of
    that bound is subtracted, so an accepted margin is a continuous claim.
    """
    if clearance_m < 0.0 or not math.isfinite(clearance_m):
        raise ValueError("clearance must be finite and nonnegative")
    speed = curve.derivative_norm_bound(1)
    step = min(occupancy.resolution_m, 0.25)
    intervals = max(1, math.ceil(speed / step))
    if intervals + 1 > 20_000:
        raise ValueError("map clearance check exceeds the sample safety limit")
    parameters = np.linspace(0.0, 1.0, intervals + 1)
    samples = np.asarray([curve.evaluate(float(value)) for value in parameters])
    distances = distance_to_occupied(occupancy, samples[:, 0], samples[:, 1])
    continuous = float(np.min(distances)) - speed / (2.0 * intervals)
    return continuous - clearance_m - occupancy.localization_radius_m


def _inflated(occupancy: OccupancyMap, radius: float) -> np.ndarray:
    grid = occupancy.grid()
    blocked = np.array(grid, copy=True)
    rows, columns = np.nonzero(~grid)
    if len(rows) == 0 or radius <= 0.0:
        return blocked
    centers_n = occupancy.origin_north_m + (rows + 0.5) * occupancy.resolution_m
    centers_e = occupancy.origin_east_m + (columns + 0.5) * occupancy.resolution_m
    distances = distance_to_occupied(occupancy, centers_n, centers_e)
    # A free cell is usable only when its whole rectangle stays outside the
    # radius. The center is half a diagonal inside the rectangle.
    half_diagonal = math.hypot(occupancy.resolution_m, occupancy.resolution_m) / 2.0
    blocked[rows, columns] = distances < radius + half_diagonal - 1e-12
    return blocked


def corridor_polyline(
    occupancy: OccupancyMap,
    *,
    start: tuple[float, float],
    goal: tuple[float, float],
    clearance_m: float,
    deviation_m: float,
) -> tuple[tuple[float, float], ...]:
    """Return a deterministic 4-connected path that stays clear of obstacles.

    Neighbor order and the heap key are fixed, so the same map and endpoints
    always produce the same polyline.
    """
    if clearance_m < 0.0 or deviation_m < 0.0:
        raise ValueError("clearance and deviation must be nonnegative")
    radius = clearance_m + deviation_m + occupancy.localization_radius_m
    blocked = _inflated(occupancy, radius)
    start_cell = occupancy.cell(*start)
    goal_cell = occupancy.cell(*goal)
    if blocked[start_cell] or blocked[goal_cell]:
        raise PlanningError(
            "corridor start or goal is inside the inflated obstacle",
            limiting_constraint="map",
        )
    neighbors = ((1, 0), (0, 1), (-1, 0), (0, -1))
    heap = [(0.0, start_cell[0], start_cell[1], start_cell)]
    best = {start_cell: 0.0}
    previous: dict[tuple[int, int], tuple[int, int] | None] = {start_cell: None}
    while heap:
        cost, row, column, cell = heapq.heappop(heap)
        if cost > best.get(cell, math.inf):
            continue
        if cell == goal_cell:
            break
        for d_row, d_column in neighbors:
            nxt = (row + d_row, column + d_column)
            if not (0 <= nxt[0] < occupancy.rows and 0 <= nxt[1] < occupancy.columns):
                continue
            if blocked[nxt]:
                continue
            step = occupancy.resolution_m
            updated = cost + step
            if updated + 1e-12 < best.get(nxt, math.inf):
                best[nxt] = updated
                previous[nxt] = cell
                heapq.heappush(heap, (updated, nxt[0], nxt[1], nxt))
    if goal_cell not in previous:
        raise PlanningError(
            "no corridor connects the launch point to the goal",
            limiting_constraint="map",
        )
    cells = []
    cursor: tuple[int, int] | None = goal_cell
    while cursor is not None:
        cells.append(cursor)
        cursor = previous[cursor]
    cells.reverse()
    points = [start]
    for cell in cells[1:-1]:
        points.append(occupancy.center(*cell))
    if points[-1] != goal:
        points.append(goal)
    if len(points) < 2:
        raise PlanningError(
            "corridor goal must differ from the launch point",
            limiting_constraint="map",
        )
    return tuple(points)
