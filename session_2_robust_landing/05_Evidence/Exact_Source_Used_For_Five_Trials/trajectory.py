import csv
import math
from dataclasses import dataclass


@dataclass
class TrajectoryPoint:
    time: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    jx: float = math.nan
    jy: float = math.nan
    jz: float = math.nan
    yaw_acceleration: float = math.nan
    yaw_jerk: float = math.nan


class Trajectory:
    """Load and continuously interpolate the generated trajectory."""

    def __init__(self, filename):
        self.points = []
        self.index = 0
        self.finished = False

        with open(filename, "r", newline="") as file:
            reader = csv.DictReader(file)
            for row in reader:
                def value(name, default=0.0):
                    raw = row.get(name)
                    return default if raw in (None, "") else float(raw)

                self.points.append(
                    TrajectoryPoint(
                        time=value("time"),
                        x=value("x"),
                        y=value("y"),
                        z=value("z"),
                        vx=value("vx"),
                        vy=value("vy"),
                        vz=value("vz"),
                        yaw=value("yaw"),
                        yaw_rate=value("yaw_rate"),
                        ax=value("ax"),
                        ay=value("ay"),
                        az=value("az"),
                        jx=value("jx", math.nan),
                        jy=value("jy", math.nan),
                        jz=value("jz", math.nan),
                        yaw_acceleration=value("yaw_acceleration", math.nan),
                        yaw_jerk=value("yaw_jerk", math.nan),
                    )
                )

        if len(self.points) < 2:
            raise ValueError("trajectory must contain at least two points")
        if any(b.time <= a.time for a, b in zip(self.points, self.points[1:])):
            raise ValueError("trajectory time must be strictly increasing")

        self.duration = self.points[-1].time

    def reset(self):
        self.index = 0
        self.finished = False

    @staticmethod
    def _lerp(a, b, ratio):
        return a + ratio * (b - a)

    @staticmethod
    def _lerp_optional(a, b, ratio):
        if not (math.isfinite(a) and math.isfinite(b)):
            return math.nan
        return a + ratio * (b - a)

    def get_target(self, t):
        if t >= self.points[-1].time:
            self.finished = True
            self.index = len(self.points) - 1
            return self.points[-1]

        while (
            self.index < len(self.points) - 2
            and self.points[self.index + 1].time <= t
        ):
            self.index += 1

        p1 = self.points[self.index]
        p2 = self.points[self.index + 1]
        ratio = (t - p1.time) / (p2.time - p1.time)

        dyaw = p2.yaw - p1.yaw
        while dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        while dyaw < -math.pi:
            dyaw += 2.0 * math.pi

        return TrajectoryPoint(
            time=t,
            x=self._lerp(p1.x, p2.x, ratio),
            y=self._lerp(p1.y, p2.y, ratio),
            z=self._lerp(p1.z, p2.z, ratio),
            vx=self._lerp(p1.vx, p2.vx, ratio),
            vy=self._lerp(p1.vy, p2.vy, ratio),
            vz=self._lerp(p1.vz, p2.vz, ratio),
            yaw=p1.yaw + ratio * dyaw,
            yaw_rate=self._lerp(p1.yaw_rate, p2.yaw_rate, ratio),
            ax=self._lerp(p1.ax, p2.ax, ratio),
            ay=self._lerp(p1.ay, p2.ay, ratio),
            az=self._lerp(p1.az, p2.az, ratio),
            jx=self._lerp_optional(p1.jx, p2.jx, ratio),
            jy=self._lerp_optional(p1.jy, p2.jy, ratio),
            jz=self._lerp_optional(p1.jz, p2.jz, ratio),
            yaw_acceleration=self._lerp_optional(
                p1.yaw_acceleration, p2.yaw_acceleration, ratio
            ),
            yaw_jerk=self._lerp_optional(p1.yaw_jerk, p2.yaw_jerk, ratio),
        )
