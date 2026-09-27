from dataclasses import dataclass
import math


@dataclass
class MotionState:
    time: float

    position: float
    velocity: float
    acceleration: float


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

        # ----------------------------
        # Acceleration phase
        # ----------------------------

        self.t_acc = abs(
            self.vc - self.v0
        ) / self.a

        self.s_acc = (
            (self.v0 + self.vc)
            * 0.5
            * self.t_acc
        )

        # ----------------------------
        # Deceleration phase
        # ----------------------------

        self.t_dec = max(
            (self.vc - self.v1) / self.d,
            0.0
        )

        self.s_dec = (
            (self.v1 + self.vc)
            * 0.5
            * self.t_dec
        )

        # ----------------------------
        # Cruise phase
        # ----------------------------

        remaining = (
            self.distance
            - self.s_acc
            - self.s_dec
        )

        # Triangle profile
        if remaining < 0:

            self.s_cruise = 0.0
            self.t_cruise = 0.0

            # Compute achievable peak speed
            A = (
                1/self.a
                +
                1/self.d
            )

            B = (
                self.v0**2/self.a
                +
                self.v1**2/self.d
            )

            vp = math.sqrt(
                (
                    2*self.distance
                    + B
                ) / A
            )

            self.vc = vp

            self.t_acc = (
                vp-self.v0
            ) / self.a

            self.t_dec = (
                vp-self.v1
            ) / self.d

            self.s_acc = (
                (self.v0+vp)
                *0.5
                *self.t_acc
            )

            self.s_dec = (
                (self.v1+vp)
                *0.5
                *self.t_dec
            )

        else:

            self.s_cruise = remaining

            self.t_cruise = (
                remaining
                / self.vc
            )

        self.total_time = (
            self.t_acc
            +
            self.t_cruise
            +
            self.t_dec
        )


    def sample(self, dt):

        samples = []

        t = 0.0

        while t <= self.total_time + 1e-9:

            # ------------------------
            # Acceleration
            # ------------------------
            if t <= self.t_acc:

                if self.vc >= self.v0:
                    a = self.a
                else:
                    a = -self.a


                v = self.v0 + a*t

                s = (
                    self.v0*t
                    +
                    0.5*a*t*t
                )

            # ------------------------
            # Cruise
            # ------------------------

            elif t <= self.t_acc + self.t_cruise:

                tc = t - self.t_acc

                a = 0.0

                v = self.vc

                s = (
                    self.s_acc
                    +
                    self.vc*tc
                )

            # ------------------------
            # Deceleration
            # ------------------------

            else:

                td = (
                    t
                    - self.t_acc
                    - self.t_cruise
                )

                if self.v1 <= self.vc:
                    a = -self.d
                else:
                    a = self.d


                v = self.vc + a*td
                
                s = (
                    self.s_acc
                    + self.s_cruise
                    + self.vc*td
                    -0.5*self.d*td*td
                )

            samples.append(

                MotionState(
                    time=t,
                    position=s,
                    velocity=v,
                    acceleration=a
                )
            )

            t += dt

        return samples