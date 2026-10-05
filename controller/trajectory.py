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
            required_fields = {"time", "x", "y", "z", "vx", "vy", "vz",
                               "yaw", "yaw_rate", "ax", "ay", "az"}
            missing = required_fields - set(reader.fieldnames or [])
            if missing:
                raise ValueError("trajectory missing required columns: " + ", ".join(sorted(missing)))
            for row in reader:
                def value(name, default=0.0, row=row):
                    raw = row.get(name)
                    if name in required_fields and raw in (None, ""):
                        raise ValueError(f"trajectory has empty required field: {name}")
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
        if any(
            b.time <= a.time
            for a, b in zip(self.points, self.points[1:], strict=False)
        ):
            raise ValueError("trajectory time must be strictly increasing")
        for index, point in enumerate(self.points):
            required = (
                point.time, point.x, point.y, point.z,
                point.vx, point.vy, point.vz, point.yaw,
                point.yaw_rate, point.ax, point.ay, point.az,
            )
            if not all(math.isfinite(value) for value in required):
                raise ValueError(
                    f"trajectory point {index} has non-finite required fields"
                )

        if self.points[0].time != 0.0:
            raise ValueError("trajectory must start at time zero")
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
        if not math.isfinite(t):
            raise ValueError("trajectory query time must be finite")
        if t <= 0.0:
            self.index = 0
            self.finished = False
            return self.points[0]
        if t < self.points[self.index].time:
            self.index = 0
        self.finished = False
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
        # Repeated subtraction can never converge for very large finite
        # angles, and subtraction itself can overflow. Reduce in bounded time.
        if not math.isfinite(dyaw):
            dyaw = math.fmod(p2.yaw, math.tau) - math.fmod(p1.yaw, math.tau)
        dyaw = math.fmod(dyaw, math.tau)
        if dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        elif dyaw < -math.pi:
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
