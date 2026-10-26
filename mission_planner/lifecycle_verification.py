"""Independent continuous bounds for the added rest-to-rest lifecycle phases."""

from functools import lru_cache
import math

from .flatness import maximum_tilt_rad, thrust_interval_n, body_rate_bound_rad_s
from .allocation import allocation_margin
from .rotors import yaw_torque_available_nm


@lru_cache(maxsize=4)
def derivative_bound(order):
    # Restrict the power polynomial to short intervals and use the Bernstein
    # convex hull. This is a continuous upper bound, not a sampled maximum.
    coefficients = [0.0, 0.0, 0.0, 0.0, 35.0, -84.0, 70.0, -20.0]
    for _ in range(order):
        coefficients = [i * coefficients[i] for i in range(1, len(coefficients))]
    degree = len(coefficients) - 1
    bound = 0.0
    for piece in range(64):
        a, width = piece / 64, 1 / 64
        power = [
            sum(
                coefficients[k] * math.comb(k, j) * a ** (k - j) * width**j
                for k in range(j, degree + 1)
            )
            for j in range(degree + 1)
        ]
        bernstein = [
            sum(power[j] * math.comb(i, j) / math.comb(degree, j) for j in range(i + 1))
            for i in range(degree + 1)
        ]
        bound = max(bound, *map(abs, bernstein))
    return bound * (1 + 1e-10)


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def verify_phase(phase, vehicle, environment):
    if not isinstance(phase, dict):
        raise ValueError("Phase must be an object")
    duration = phase.get("duration_s")
    start, end = phase.get("start"), phase.get("end")
    speed_limit = phase.get("speed_limit_m_s")
    if (
        not isinstance(start, (list, tuple))
        or not isinstance(end, (list, tuple))
        or len(start) != 4
        or len(end) != 4
        or not all(_finite_number(v) for v in (*start, *end, duration, speed_limit))
        or duration <= 0
        or speed_limit <= 0
    ):
        raise ValueError(
            "Phase requires finite poses, positive duration and positive speed limit"
        )
    distance = math.dist(start[:3], end[:3])
    turn = abs(end[3] - start[3])
    bounds = [derivative_bound(i) / duration**i for i in (1, 2, 3)]
    speed, acceleration, jerk = [distance * b for b in bounds]
    yaw_rate, yaw_acceleration, yaw_jerk = [turn * b for b in bounds]
    disturbance_a = getattr(vehicle, "disturbance_acceleration_m_s2", 0.0)
    disturbance_j = getattr(vehicle, "disturbance_jerk_m_s3", 0.0)
    total_a, total_j = acceleration + disturbance_a, jerk + disturbance_j
    low, high = thrust_interval_n(total_a, vehicle.mass_kg)
    torque = vehicle.yaw_inertia_kg_m2 * yaw_acceleration
    if vehicle.schema_version == 1:
        authority = min(
            yaw_torque_available_nm(
                t,
                rotor_thrust_minimum_n=vehicle.rotor_thrust_minimum_n,
                rotor_thrust_maximum_n=vehicle.rotor_thrust_maximum_n,
                yaw_torque_per_thrust_m=vehicle.yaw_torque_per_thrust_m,
            )
            for t in (low, high)
        )
        allocation_ok = authority + 1e-10 >= torque
    else:
        allocation_ok = (
            allocation_margin(
                vehicle.rotors, low, high, torque, vehicle.roll_pitch_torque_reserve_nm
            )
            >= -1e-10
        )
    conditions = {
        "speed": speed <= min(speed_limit, 0.9 * vehicle.maximum_speed_m_s) + 1e-9,
        "acceleration": acceleration <= 0.9 * vehicle.maximum_acceleration_m_s2 + 1e-9,
        "jerk": jerk <= 0.9 * vehicle.maximum_jerk_m_s3 + 1e-9,
        "yaw_rate": yaw_rate
        <= 0.9 * math.radians(vehicle.maximum_yaw_rate_deg_s) + 1e-9,
        "yaw_acceleration": yaw_acceleration
        <= 0.9 * math.radians(vehicle.maximum_yaw_acceleration_deg_s2) + 1e-9,
        "yaw_jerk": yaw_jerk
        <= 0.9 * math.radians(vehicle.maximum_yaw_jerk_deg_s3) + 1e-9,
        "tilt": maximum_tilt_rad(total_a)
        <= 0.9 * math.radians(vehicle.maximum_tilt_deg) + 1e-9,
        "thrust": low >= vehicle.minimum_thrust_n / 0.9 - 1e-9
        and high <= 0.9 * vehicle.maximum_thrust_n + 1e-9,
        "body_rate": body_rate_bound_rad_s(total_a, total_j, yaw_rate)
        <= 0.9 * math.radians(vehicle.maximum_body_rate_deg_s) + 1e-9,
        "rotor_authority": allocation_ok,
        # q is monotone in [0,1]; every spatial point is in this segment hull.
        "ground_and_envelope": all(
            math.hypot(p[0], p[1]) <= environment.max_radius_m + 1e-9
            and -environment.max_height_above_launch_m - 1e-9 <= p[2] <= 1e-9
            for p in (start, end)
        ),
    }
    return dict(
        accepted=all(conditions.values()),
        checks=conditions,
        speed_bound_m_s=speed,
        acceleration_bound_m_s2=acceleration,
        jerk_bound_m_s3=jerk,
        yaw_rate_bound_rad_s=yaw_rate,
        method="septic Bernstein interval hulls; monotone straight segment",
    )
