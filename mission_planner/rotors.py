"""Yaw-torque check for a quadrotor with two rotors of each spin.

The Gazebo x500 base link has a yaw inertia of 0.04 kg m^2. PX4 v1.17's
x500 airframe gives each rotor a yaw-torque coefficient of magnitude 0.05 m,
with two rotors of each sign. Those numbers are simulator data, not an
identified aircraft.

With that pairing, a collective thrust ``T`` and a yaw torque ``tau`` are
feasible when each pair's thrust sum lies inside twice the per-rotor limit.
That holds exactly when ``|tau|`` is at most ``coefficient * slack(T)``, where
``slack`` is the room between ``T`` and the collective rotor limits.
"""

from __future__ import annotations

from .flatness import thrust_interval_n


def yaw_torque_available_nm(
    thrust_n: float,
    *,
    rotor_thrust_minimum_n: float,
    rotor_thrust_maximum_n: float,
    yaw_torque_per_thrust_m: float,
) -> float:
    """Return the largest yaw torque the paired rotors can add at this thrust."""
    if thrust_n < 0.0:
        raise ValueError("thrust must be nonnegative")
    collective_minimum = 4.0 * rotor_thrust_minimum_n
    collective_maximum = 4.0 * rotor_thrust_maximum_n
    slack = min(thrust_n - collective_minimum, collective_maximum - thrust_n)
    if slack <= 0.0:
        return 0.0
    return yaw_torque_per_thrust_m * slack


def rotor_duration_s(
    *,
    acceleration_numerator: float,
    yaw_acceleration_numerator: float,
    mass_kg: float,
    yaw_inertia_kg_m2: float,
    rotor_thrust_minimum_n: float,
    rotor_thrust_maximum_n: float,
    yaw_torque_per_thrust_m: float,
) -> tuple[float, str]:
    """Return the shortest duration whose thrust ball fits the rotor yaw torque.

    The numerators are the certified acceleration and yaw acceleration of a
    one-second schedule. Stretching the schedule divides both by ``T**2``.
    """
    if acceleration_numerator < 0.0 or yaw_acceleration_numerator < 0.0:
        raise ValueError("dynamic numerators must be nonnegative")
    if yaw_inertia_kg_m2 <= 0.0 or yaw_torque_per_thrust_m <= 0.0:
        raise ValueError("yaw inertia and torque coefficient must be positive")

    def failures(duration: float) -> bool:
        acceleration = acceleration_numerator / duration**2
        low, high = thrust_interval_n(acceleration, mass_kg)
        torque = yaw_inertia_kg_m2 * yaw_acceleration_numerator / duration**2
        available = min(
            yaw_torque_available_nm(
                low,
                rotor_thrust_minimum_n=rotor_thrust_minimum_n,
                rotor_thrust_maximum_n=rotor_thrust_maximum_n,
                yaw_torque_per_thrust_m=yaw_torque_per_thrust_m,
            ),
            yaw_torque_available_nm(
                high,
                rotor_thrust_minimum_n=rotor_thrust_minimum_n,
                rotor_thrust_maximum_n=rotor_thrust_maximum_n,
                yaw_torque_per_thrust_m=yaw_torque_per_thrust_m,
            ),
        )
        collective_minimum = 4.0 * rotor_thrust_minimum_n
        collective_maximum = 4.0 * rotor_thrust_maximum_n
        if high > collective_maximum + 1e-9 or low + 1e-9 < collective_minimum:
            return True
        return torque > available + 1e-12

    low = 1e-6
    high = 1.0
    for _ in range(40):
        if not failures(high):
            break
        high *= 2.0
    else:
        if failures(high):
            raise ValueError("rotor yaw torque stays infeasible at every duration")
    if not failures(low):
        return low, "none"
    for _ in range(60):
        middle = 0.5 * (low + high)
        if failures(middle):
            low = middle
        else:
            high = middle
    return high, "rotor"
