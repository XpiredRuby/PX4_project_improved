"""Certified tilt, thrust, and body-rate bounds for a multicopter.

Position acceleration and jerk are flat outputs. In NED the rotors must
produce the specific force ``(a_north, a_east, g - a_down)``. Every
acceleration inside a ball of radius ``A`` therefore lands in a ball of
radius ``A`` around the hover thrust ``(0, 0, g)``. The tangent from the
origin to that ball is the largest tilt, ``asin(A / g)``, and the thrust
magnitude stays inside ``m * (g - A)`` and ``m * (g + A)``.

The thrust direction can rotate no faster than ``jerk / (g - A)``. Adding
the planned yaw rate bounds the body rate. These are bounds, so a trajectory
they accept is inside the model; a trajectory they reject might still be
flyable along some particular direction, and is refused anyway.
"""

from __future__ import annotations

import math


GRAVITY_M_S2 = 9.80665


def maximum_tilt_rad(acceleration_m_s2: float) -> float:
    """Return the largest tilt an acceleration ball of this radius can demand."""
    if acceleration_m_s2 < 0.0:
        raise ValueError("acceleration magnitude must be nonnegative")
    if acceleration_m_s2 >= GRAVITY_M_S2:
        return math.pi / 2.0
    return math.asin(acceleration_m_s2 / GRAVITY_M_S2)


def thrust_interval_n(
    acceleration_m_s2: float,
    mass_kg: float,
) -> tuple[float, float]:
    """Return the thrust range implied by an acceleration ball."""
    if mass_kg <= 0.0:
        raise ValueError("mass must be positive")
    if acceleration_m_s2 < 0.0:
        raise ValueError("acceleration magnitude must be nonnegative")
    return (
        mass_kg * max(0.0, GRAVITY_M_S2 - acceleration_m_s2),
        mass_kg * (GRAVITY_M_S2 + acceleration_m_s2),
    )


def body_rate_bound_rad_s(
    acceleration_m_s2: float,
    jerk_m_s3: float,
    yaw_rate_rad_s: float,
) -> float:
    """Bound body rate from the thrust-direction rate plus yaw rate."""
    if acceleration_m_s2 < 0.0 or jerk_m_s3 < 0.0 or yaw_rate_rad_s < 0.0:
        raise ValueError("dynamic magnitudes must be nonnegative")
    remaining = GRAVITY_M_S2 - acceleration_m_s2
    if remaining <= 0.0:
        return math.inf
    return jerk_m_s3 / remaining + yaw_rate_rad_s


def attitude_duration_s(
    *,
    acceleration_numerator: float,
    jerk_numerator: float,
    yaw_rate_numerator: float,
    mass_kg: float,
    tilt_limit_rad: float,
    minimum_thrust_n: float,
    maximum_thrust_n: float,
    maximum_body_rate_rad_s: float,
    disturbance_acceleration_m_s2: float = 0.,
    disturbance_jerk_m_s3: float = 0.,
) -> tuple[float, str]:
    """Return the shortest duration that keeps the attitude bounds inside the model.

    The numerators are the certified acceleration, jerk, and yaw rate at a
    duration of one second. Stretching the same schedule divides them by
    ``T**2``, ``T**3``, and ``T``.
    """
    if acceleration_numerator < 0.0 or jerk_numerator < 0.0:
        raise ValueError("dynamic numerators must be nonnegative")
    if yaw_rate_numerator < 0.0:
        raise ValueError("yaw-rate numerator must be nonnegative")
    if not 0.0 < tilt_limit_rad <= math.pi / 2.0:
        raise ValueError("maximum tilt must be in (0, 90] degrees")
    hover = mass_kg * GRAVITY_M_S2
    if maximum_thrust_n <= hover:
        raise ValueError(
            "maximum thrust cannot hold hover; "
            f"hover needs {hover:.3f} N and the model allows {maximum_thrust_n:.3f} N"
        )
    if minimum_thrust_n >= hover:
        raise ValueError("minimum thrust is at or above hover thrust")

    def state(duration: float) -> tuple[float, float, float]:
        return (
            acceleration_numerator / duration**2 + disturbance_acceleration_m_s2,
            jerk_numerator / duration**3 + disturbance_jerk_m_s3,
            yaw_rate_numerator / duration,
        )

    def failures(duration: float) -> tuple[str, ...]:
        acceleration, jerk, yaw_rate = state(duration)
        reasons = []
        if maximum_tilt_rad(acceleration) > tilt_limit_rad + 1e-12:
            reasons.append("tilt")
        lower, upper = thrust_interval_n(acceleration, mass_kg)
        if upper > maximum_thrust_n + 1e-9 or lower + 1e-9 < minimum_thrust_n:
            reasons.append("thrust")
        if (
            body_rate_bound_rad_s(acceleration, jerk, yaw_rate)
            > maximum_body_rate_rad_s + 1e-12
        ):
            reasons.append("body_rate")
        return tuple(reasons)

    # Zero motion satisfies every attitude limit. A one-second schedule is the
    # numerator itself, and doubling until it fits brackets the answer.
    low = 1e-6
    high = 1.0
    high_failures = failures(high)
    for _ in range(40):
        if not high_failures:
            break
        high *= 2.0
        high_failures = failures(high)
    else:
        if high_failures:
            raise ValueError(
                "attitude limits stay infeasible at every duration; active "
                + ", ".join(high_failures)
            )
    if not failures(low):
        return low, "none"
    binding = failures(low)[0]
    for _ in range(60):
        middle = 0.5 * (low + high)
        reasons = failures(middle)
        if reasons:
            binding = reasons[0]
            low = middle
        else:
            high = middle
    return high, binding
