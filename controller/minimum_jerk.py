import math

import numpy as np

from trajectory import TrajectoryPoint


class MinimumJerkSegment:
    """Boundary-continuous quintic segment with verified motion limits."""

    def __init__(
        self,
        start,
        finish,
        duration,
        start_velocity=(0.0, 0.0, 0.0),
        finish_velocity=(0.0, 0.0, 0.0),
        start_acceleration=(0.0, 0.0, 0.0),
        finish_acceleration=(0.0, 0.0, 0.0),
    ):
        if len(start) != 3 or len(finish) != 3:
            raise ValueError("minimum-jerk endpoints must be 3D")
        if duration <= 0.0:
            raise ValueError("minimum-jerk duration must be positive")
        self.start = tuple(float(value) for value in start)
        self.finish = tuple(float(value) for value in finish)
        self.start_velocity = tuple(float(value) for value in start_velocity)
        self.finish_velocity = tuple(float(value) for value in finish_velocity)
        self.start_acceleration = tuple(
            float(value) for value in start_acceleration
        )
        self.finish_acceleration = tuple(
            float(value) for value in finish_acceleration
        )
        self.duration = float(duration)
        self._coefficients = tuple(
            self._axis_coefficients(
                self.start[index],
                self.finish[index],
                self.start_velocity[index],
                self.finish_velocity[index],
                self.start_acceleration[index],
                self.finish_acceleration[index],
            )
            for index in range(3)
        )

    def _axis_coefficients(self, p0, pf, v0, vf, a0, af):
        duration = self.duration
        duration_2 = duration**2
        c0 = p0
        c1 = v0
        c2 = 0.5 * a0
        c3 = (
            20.0 * (pf - p0)
            - (8.0 * vf + 12.0 * v0) * duration
            - (3.0 * a0 - af) * duration_2
        ) / (2.0 * duration**3)
        c4 = (
            30.0 * (p0 - pf)
            + (14.0 * vf + 16.0 * v0) * duration
            + (3.0 * a0 - 2.0 * af) * duration_2
        ) / (2.0 * duration**4)
        c5 = (
            12.0 * (pf - p0)
            - (6.0 * vf + 6.0 * v0) * duration
            - (a0 - af) * duration_2
        ) / (2.0 * duration**5)
        return c0, c1, c2, c3, c4, c5

    @classmethod
    def from_limits(
        cls,
        start,
        finish,
        max_speed,
        max_accel,
        max_jerk,
        minimum_duration=0.0,
        start_velocity=(0.0, 0.0, 0.0),
        finish_velocity=(0.0, 0.0, 0.0),
        start_acceleration=(0.0, 0.0, 0.0),
        finish_acceleration=(0.0, 0.0, 0.0),
    ):
        if min(max_speed, max_accel, max_jerk) <= 0.0:
            raise ValueError("motion limits must be positive")
        endpoint_speeds = (
            math.sqrt(sum(float(value) ** 2 for value in start_velocity)),
            math.sqrt(sum(float(value) ** 2 for value in finish_velocity)),
        )
        if max(endpoint_speeds) > max_speed * 1.001:
            raise ValueError("endpoint velocity exceeds segment speed limit")

        distance = math.sqrt(
            sum(
                (float(b) - float(a)) ** 2
                for a, b in zip(start, finish, strict=True)
            )
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
        # Nonzero boundary velocity changes the extrema. Increase duration
        # until the vector-norm polynomial extrema satisfy all limits.
        for _ in range(20):
            segment = cls(
                start,
                finish,
                duration,
                start_velocity=start_velocity,
                finish_velocity=finish_velocity,
                start_acceleration=start_acceleration,
                finish_acceleration=finish_acceleration,
            )
            peak_speed, peak_accel, peak_jerk = segment.peak_kinematics()
            scale = max(
                1.0,
                peak_speed / max_speed,
                math.sqrt(peak_accel / max_accel),
                (peak_jerk / max_jerk) ** (1.0 / 3.0),
            )
            if scale <= 1.0001:
                return segment
            duration *= 1.02 * scale
        raise RuntimeError("unable to satisfy minimum-jerk motion limits")

    @staticmethod
    def _evaluate(coefficients, elapsed_s):
        c0, c1, c2, c3, c4, c5 = coefficients
        t = elapsed_s
        position = (
            c0
            + c1 * t
            + c2 * t**2
            + c3 * t**3
            + c4 * t**4
            + c5 * t**5
        )
        velocity = (
            c1
            + 2.0 * c2 * t
            + 3.0 * c3 * t**2
            + 4.0 * c4 * t**3
            + 5.0 * c5 * t**4
        )
        acceleration = (
            2.0 * c2
            + 6.0 * c3 * t
            + 12.0 * c4 * t**2
            + 20.0 * c5 * t**3
        )
        jerk = 6.0 * c3 + 24.0 * c4 * t + 60.0 * c5 * t**2
        return position, velocity, acceleration, jerk

    def sample(self, elapsed_s):
        elapsed_s = max(0.0, min(float(elapsed_s), self.duration))
        samples = tuple(
            self._evaluate(coefficients, elapsed_s)
            for coefficients in self._coefficients
        )
        values = tuple(sample[0] for sample in samples)
        velocity = tuple(sample[1] for sample in samples)
        acceleration = tuple(sample[2] for sample in samples)
        jerk = tuple(sample[3] for sample in samples)

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

    @staticmethod
    def _derivative_coefficients(coefficients, order):
        values = np.asarray(coefficients, dtype=float)
        for _ in range(order):
            values = np.asarray(
                [index * values[index] for index in range(1, len(values))],
                dtype=float,
            )
        return values

    def _vector_norm_extrema_times(self, derivative_order):
        vectors = [
            self._derivative_coefficients(coefficients, derivative_order)
            * self.duration
            ** np.arange(6 - derivative_order)
            for coefficients in self._coefficients
        ]
        derivatives = [
            self._derivative_coefficients(coefficients, 1)
            for coefficients in vectors
        ]
        norm_derivative = np.asarray([0.0])
        for vector, derivative in zip(vectors, derivatives, strict=True):
            product = np.polynomial.polynomial.polymul(vector, derivative)
            if len(product) > len(norm_derivative):
                norm_derivative = np.pad(
                    norm_derivative,
                    (0, len(product) - len(norm_derivative)),
                )
            norm_derivative[: len(product)] += product

        candidates = [0.0, self.duration]
        peak_coefficient = max(abs(norm_derivative))
        if peak_coefficient > 0.0:
            normalized = np.polynomial.polynomial.polytrim(
                norm_derivative / peak_coefficient, tol=1e-12
            )
            roots = np.polynomial.polynomial.polyroots(normalized)
            candidates.extend(
                float(root.real * self.duration)
                for root in roots
                if abs(root.imag) <= 1e-8
                and -1e-9 <= root.real <= 1.0 + 1e-9
            )
        return candidates

    def peak_kinematics(self, sample_count=None):
        """Return vector-norm peaks at analytic polynomial extrema."""
        peak_speed = peak_acceleration = peak_jerk = 0.0
        candidate_sets = (
            self._vector_norm_extrema_times(1),
            self._vector_norm_extrema_times(2),
            self._vector_norm_extrema_times(3),
        )
        for quantity, candidates in enumerate(candidate_sets):
            for elapsed_s in candidates:
                point = self.sample(elapsed_s)
                vectors = (
                    (point.vx, point.vy, point.vz),
                    (point.ax, point.ay, point.az),
                    (point.jx, point.jy, point.jz),
                )
                magnitude = math.sqrt(sum(value**2 for value in vectors[quantity]))
                if quantity == 0:
                    peak_speed = max(peak_speed, magnitude)
                elif quantity == 1:
                    peak_acceleration = max(peak_acceleration, magnitude)
                else:
                    peak_jerk = max(peak_jerk, magnitude)
        return peak_speed, peak_acceleration, peak_jerk

    def finished(self, elapsed_s):
        return elapsed_s >= self.duration
