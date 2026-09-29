import math

from trajectory import TrajectoryPoint


class MinimumJerkSegment:
    """Rest-to-rest quintic segment with bounded velocity/accel/jerk."""

    def __init__(self, start, finish, duration):
        if len(start) != 3 or len(finish) != 3:
            raise ValueError("minimum-jerk endpoints must be 3D")
        if duration <= 0.0:
            raise ValueError("minimum-jerk duration must be positive")
        self.start = tuple(float(value) for value in start)
        self.finish = tuple(float(value) for value in finish)
        self.duration = float(duration)

    @classmethod
    def from_limits(
        cls,
        start,
        finish,
        max_speed,
        max_accel,
        max_jerk,
        minimum_duration=0.0,
    ):
        distance = math.sqrt(
            sum((float(b) - float(a)) ** 2 for a, b in zip(start, finish))
        )
        if distance <= 1e-9:
            duration = max(0.1, float(minimum_duration))
        else:
            # Exact/upper-bound maxima for the quintic smooth-step profile.
            velocity_time = 1.875 * distance / max_speed
            acceleration_time = math.sqrt(5.774 * distance / max_accel)
            jerk_time = (60.0 * distance / max_jerk) ** (1.0 / 3.0)
            duration = max(
                float(minimum_duration),
                velocity_time,
                acceleration_time,
                jerk_time,
            )
        return cls(start, finish, duration)

    def sample(self, elapsed_s):
        elapsed_s = max(0.0, min(float(elapsed_s), self.duration))
        s = elapsed_s / self.duration

        blend = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
        blend_rate = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / self.duration
        blend_accel = (60.0 * s - 180.0 * s**2 + 120.0 * s**3) / self.duration**2
        blend_jerk = (60.0 - 360.0 * s + 360.0 * s**2) / self.duration**3

        delta = tuple(b - a for a, b in zip(self.start, self.finish))
        values = tuple(a + blend * d for a, d in zip(self.start, delta))
        velocity = tuple(blend_rate * d for d in delta)
        acceleration = tuple(blend_accel * d for d in delta)
        jerk = tuple(blend_jerk * d for d in delta)

        return TrajectoryPoint(
            time=elapsed_s,
            x=values[0],
            y=values[1],
            z=values[2],
            vx=velocity[0],
            vy=velocity[1],
            vz=velocity[2],
            ax=acceleration[0],
            ay=acceleration[1],
            az=acceleration[2],
            jx=jerk[0],
            jy=jerk[1],
            jz=jerk[2],
        )

    def finished(self, elapsed_s):
        return elapsed_s >= self.duration
