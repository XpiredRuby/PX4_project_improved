from dataclasses import dataclass
import math


def sample_times(total_time, dt):
    """Return a positive, monotonic grid that includes the exact endpoint."""
    if dt <= 0.0:
        raise ValueError("sample period must be positive")
    if total_time < 0.0:
        raise ValueError("profile duration cannot be negative")

    step_count = int(math.floor(total_time / dt + 1e-12))
    times = [index * dt for index in range(step_count + 1)]
    if not times:
        times = [0.0]

    tolerance = 1e-10 * max(1.0, total_time)
    if total_time - times[-1] > tolerance:
        times.append(total_time)
    else:
        times[-1] = total_time
    return times


@dataclass
class MotionState:
    time: float

    position: float
    velocity: float
    acceleration: float
    jerk: float = 0.0


class TrapezoidalProfile:

    def __init__(
        self,
        distance,
        start_speed,
        cruise_speed,
        end_speed,
        max_acceleration,
        max_deceleration,
    ):
        self.distance = distance

        self.v0 = start_speed
        self.vc = cruise_speed
        self.v1 = end_speed

        self.a = max_acceleration
        self.d = max_deceleration

        self._compute_profile()

    def _compute_profile(self):
        # Acceleration phase
        self.t_acc = abs(self.vc - self.v0) / self.a
        self.s_acc = (self.v0 + self.vc) * 0.5 * self.t_acc

        # Deceleration phase
        self.t_dec = max((self.vc - self.v1) / self.d, 0.0)
        self.s_dec = (self.v1 + self.vc) * 0.5 * self.t_dec

        remaining = self.distance - self.s_acc - self.s_dec

        # Triangle profile
        if remaining < 0:
            self.s_cruise = 0.0
            self.t_cruise = 0.0

            # Compute achievable peak speed
            A = 1 / self.a + 1 / self.d
            B = self.v0**2 / self.a + self.v1**2 / self.d

            vp = math.sqrt((2 * self.distance + B) / A)

            self.vc = vp

            self.t_acc = (vp - self.v0) / self.a
            self.t_dec = (vp - self.v1) / self.d

            self.s_acc = (self.v0 + vp) * 0.5 * self.t_acc
            self.s_dec = (self.v1 + vp) * 0.5 * self.t_dec
        else:
            self.s_cruise = remaining
            self.t_cruise = remaining / self.vc

        self.total_time = self.t_acc + self.t_cruise + self.t_dec

    def sample(self, dt):
        samples = []
        tolerance = 1e-10 * max(1.0, self.total_time)

        for t in sample_times(self.total_time, dt):
            if abs(t - self.total_time) <= tolerance:
                samples.append(
                    MotionState(
                        time=self.total_time,
                        position=self.distance,
                        velocity=self.v1,
                        acceleration=0.0,
                        jerk=0.0,
                    )
                )
                continue

            if t <= self.t_acc:
                a = self.a if self.vc >= self.v0 else -self.a
                v = self.v0 + a * t
                s = self.v0 * t + 0.5 * a * t * t
            elif t <= self.t_acc + self.t_cruise:
                tc = t - self.t_acc
                a = 0.0
                v = self.vc
                s = self.s_acc + self.vc * tc
            else:
                td = t - self.t_acc - self.t_cruise
                a = -self.d if self.v1 <= self.vc else self.d
                v = self.vc + a * td
                s = (
                    self.s_acc
                    + self.s_cruise
                    + self.vc * td
                    - 0.5 * self.d * td * td
                )

            samples.append(
                MotionState(
                    time=t,
                    position=s,
                    velocity=v,
                    acceleration=a,
                    jerk=0.0,
                )
            )

        return samples


class SCurveProfile:
    """
    Jerk-limited (S-curve) profile.

    Uses bang-bang jerk integration. Robust for arbitrary
    start/end speeds and short distances. If the distance is
    too small to reach the cruise speed, the peak speed is
    reduced iteratively until the motion fits.
    """

    def __init__(
        self,
        distance,
        start_speed,
        cruise_speed,
        end_speed,
        max_acceleration,
        max_deceleration,
        max_jerk,
    ):
        self.distance = distance
        self.v0 = start_speed
        self.vc = cruise_speed
        self.v1 = end_speed
        self.a = max_acceleration
        self.d = max_deceleration
        self.j = max_jerk if max_jerk and max_jerk > 0 else 1.0
        self._fit_peak_speed()

    def _distance_for_peak(self, vp):
        d_acc, t_acc = self._ramp(self.v0, vp, self.a)
        d_dec, t_dec = self._ramp(vp, self.v1, self.d)
        return d_acc + d_dec, (t_acc, t_dec)

    def _ramp(self, v_start, v_end, a_max):
        dv = abs(v_end - v_start)
        j = self.j
        if dv * j < a_max * a_max:
            a_peak = math.sqrt(dv * j)
            tj = a_peak / j
            ta = 0.0
        else:
            a_peak = a_max
            tj = a_max / j
            ta = dv / a_max - tj
        dist = self._ramp_distance(v_start, v_end, tj, ta, a_peak)
        return dist, (tj, ta, tj)

    def _ramp_distance(self, v_start, v_end, tj, ta, a_peak):
        sign = 1.0 if v_end >= v_start else -1.0
        a = sign * a_peak
        j = sign * self.j

        v1 = v_start + 0.5 * j * tj * tj
        s1 = v_start * tj + (1.0 / 6.0) * j * tj**3

        v2 = v1 + a * ta
        s2 = v1 * ta + 0.5 * a * ta * ta

        s3 = v2 * tj + 0.5 * a * tj * tj - (1.0 / 6.0) * j * tj**3

        return s1 + s2 + s3

    def _fit_peak_speed(self):
        vp = self.vc
        lo = max(self.v0, self.v1)
        d_needed, _ = self._distance_for_peak(vp)

        if d_needed > self.distance:
            hi = vp
            lo_v = lo
            for _ in range(60):
                mid = 0.5 * (lo_v + hi)
                d_mid, _ = self._distance_for_peak(mid)
                if d_mid > self.distance:
                    hi = mid
                else:
                    lo_v = mid
            vp = lo_v

        self.vp = vp

        d_acc, self._acc_phases = self._ramp(self.v0, vp, self.a)
        d_dec, self._dec_phases = self._ramp(vp, self.v1, self.d)

        self.s_acc = d_acc
        self.s_dec = d_dec
        self.t_acc = sum(self._acc_phases)
        self.t_dec = sum(self._dec_phases)

        self.s_cruise = max(self.distance - self.s_acc - self.s_dec, 0.0)
        self.t_cruise = self.s_cruise / vp if vp > 1e-9 else 0.0

        self.total_time = self.t_acc + self.t_cruise + self.t_dec

    def _integrate_ramp(self, tau, v_start, v_end, phases):
        """
        Return (position_offset, velocity, acceleration, jerk)
        at local time tau within a jerk-limited ramp.
        """
        tj, ta, tj2 = phases
        if tj + ta + tj2 <= 1e-12:
            return 0.0, v_start, 0.0, 0.0

        sign = 1.0 if v_end >= v_start else -1.0
        j = sign * self.j

        # Phase 1: jerk up (0 -> tj)
        if tau <= tj:
            t = tau
            a = j * t
            v = v_start + 0.5 * j * t * t
            s = v_start * t + (1.0 / 6.0) * j * t**3
            return s, v, a, j

        # End of phase 1
        a1 = j * tj
        v1 = v_start + 0.5 * j * tj * tj
        s1 = v_start * tj + (1.0 / 6.0) * j * tj**3

        # Phase 2: constant accel (tj -> tj + ta)
        if tau <= tj + ta:
            t = tau - tj
            a = a1
            v = v1 + a1 * t
            s = s1 + v1 * t + 0.5 * a1 * t * t
            return s, v, a, 0.0

        # End of phase 2
        v2 = v1 + a1 * ta
        s2 = s1 + v1 * ta + 0.5 * a1 * ta * ta

        # Phase 3: jerk down (tj + ta -> tj + ta + tj2)
        t = tau - tj - ta
        a = a1 - j * t
        v = v2 + a1 * t - 0.5 * j * t * t
        s = s2 + v2 * t + 0.5 * a1 * t * t - (1.0 / 6.0) * j * t**3
        return s, v, a, -j

    def sample(self, dt):
        samples = []
        tolerance = 1e-10 * max(1.0, self.total_time)

        for t in sample_times(self.total_time, dt):
            if abs(t - self.total_time) <= tolerance:
                samples.append(
                    MotionState(
                        time=self.total_time,
                        position=self.distance,
                        velocity=self.v1,
                        acceleration=0.0,
                        jerk=0.0,
                    )
                )
                continue

            if t <= self.t_acc:
                s, v, a, jerk = self._integrate_ramp(
                    t, self.v0, self.vp, self._acc_phases
                )
            elif t <= self.t_acc + self.t_cruise:
                tc = t - self.t_acc
                s = self.s_acc + self.vp * tc
                v = self.vp
                a = 0.0
                jerk = 0.0
            else:
                td = t - self.t_acc - self.t_cruise
                ds, v, a, jerk = self._integrate_ramp(
                    td, self.vp, self.v1, self._dec_phases
                )
                s = self.s_acc + self.s_cruise + ds

            samples.append(
                MotionState(
                    time=t,
                    position=s,
                    velocity=v,
                    acceleration=a,
                    jerk=jerk,
                )
            )

        return samples
