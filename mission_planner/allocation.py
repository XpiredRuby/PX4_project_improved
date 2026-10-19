"""Conservative wrench allocation for parallel-axis, fixed-pitch multicopters.

An explicit right inverse maps [upward thrust, roll, pitch, yaw] into individual
rotor thrusts. Interval arithmetic checks the entire wrench box, including
simultaneous roll/pitch reserve. This sufficient test can reject feasible
allocations; it does not reproduce PX4's desaturating control allocator.
Positions and moments use body FRD about the loaded center of mass.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from functools import lru_cache
import math

import numpy as np

from .flatness import thrust_interval_n


@dataclass(frozen=True)
class Rotor:
    x_m: float
    y_m: float
    yaw_torque_per_thrust_m: float
    minimum_thrust_n: float
    maximum_thrust_n: float

    @classmethod
    def from_dict(cls, raw):
        from .contracts import _number, _object, _strict
        raw = _object(raw, "rotor")
        _strict(raw, required=set(cls.__dataclass_fields__), field="rotor")
        values = {key: _number(raw[key], key) for key in cls.__dataclass_fields__}
        if not 0 <= values["minimum_thrust_n"] < values["maximum_thrust_n"]:
            raise ValueError("rotor thrust bounds must satisfy 0 <= minimum < maximum")
        if values["yaw_torque_per_thrust_m"] == 0:
            raise ValueError("each rotor needs a signed yaw torque coefficient")
        return cls(**values)

    def to_dict(self):
        return asdict(self)


@lru_cache(maxsize=128)
def right_inverse(rotors: tuple[Rotor, ...]):
    if not 4 <= len(rotors) <= 16:
        raise ValueError("parallel-rotor allocation requires 4–16 rotors")
    matrix = np.array([[1., -r.y_m, r.x_m, r.yaw_torque_per_thrust_m]
                       for r in rotors], dtype=float).T
    if np.linalg.matrix_rank(matrix) != 4:
        raise ValueError("rotor geometry cannot independently control thrust, roll, pitch and yaw")
    inverse = np.linalg.pinv(matrix)
    if not np.allclose(matrix @ inverse, np.eye(4), atol=1e-10, rtol=1e-10):
        raise ValueError("rotor allocation is numerically ill-conditioned")
    inverse.setflags(write=False)
    return inverse


def allocation_margin(rotors, thrust_low_n, thrust_high_n, yaw_torque_nm,
                      roll_pitch_reserve_nm):
    """Minimum normalized actuator margin across a continuous wrench box."""
    if not (0 <= thrust_low_n <= thrust_high_n and yaw_torque_nm >= 0
            and roll_pitch_reserve_nm >= 0):
        raise ValueError("invalid wrench interval")
    center = np.array([(thrust_low_n + thrust_high_n) / 2., 0., 0., 0.])
    radius = np.array([(thrust_high_n - thrust_low_n) / 2.,
                       roll_pitch_reserve_nm, roll_pitch_reserve_nm, yaw_torque_nm])
    if not np.isfinite(center).all() or not np.isfinite(radius).all():
        raise ValueError("wrench interval must be finite")
    inverse = right_inverse(tuple(rotors))
    midpoint = inverse @ center
    spread = abs(inverse) @ radius
    lower = np.array([r.minimum_thrust_n for r in rotors])
    upper = np.array([r.maximum_thrust_n for r in rotors])
    return float(np.minimum((midpoint - spread - lower) / (upper - lower),
                            (upper - midpoint - spread) / (upper - lower)).min())


def vehicle_rotor_duration_s(vehicle, acceleration_numerator,
                             yaw_acceleration_numerator):
    if vehicle.schema_version == 1:
        from .rotors import rotor_duration_s
        return rotor_duration_s(
            acceleration_numerator=acceleration_numerator,
            yaw_acceleration_numerator=yaw_acceleration_numerator,
            mass_kg=vehicle.mass_kg, yaw_inertia_kg_m2=vehicle.yaw_inertia_kg_m2,
            rotor_thrust_minimum_n=vehicle.rotor_thrust_minimum_n,
            rotor_thrust_maximum_n=vehicle.rotor_thrust_maximum_n,
            yaw_torque_per_thrust_m=vehicle.yaw_torque_per_thrust_m)
    hover_lo, hover_hi = thrust_interval_n(vehicle.disturbance_acceleration_m_s2, vehicle.mass_kg)
    if allocation_margin(vehicle.rotors, hover_lo, hover_hi, 0.,
                         vehicle.roll_pitch_torque_reserve_nm) <= 1e-9:
        raise ValueError("loaded vehicle cannot hover with the declared roll/pitch torque reserve")

    def feasible(duration):
        lo, hi = thrust_interval_n(acceleration_numerator / duration**2 + vehicle.disturbance_acceleration_m_s2, vehicle.mass_kg)
        yaw = vehicle.yaw_inertia_kg_m2 * yaw_acceleration_numerator / duration**2
        return allocation_margin(vehicle.rotors, lo, hi, yaw,
                                 vehicle.roll_pitch_torque_reserve_nm) >= 0.

    low, high = 1e-6, 1.
    for _ in range(40):
        if feasible(high):
            break
        high *= 2.
    else:
        raise ValueError("rotor allocation remains infeasible at every duration")
    for _ in range(60):
        middle = (low + high) / 2.
        if feasible(middle):
            high = middle
        else:
            low = middle
    if not math.isfinite(high):
        raise ValueError("non-finite rotor duration")
    return high, "rotor_allocation"
