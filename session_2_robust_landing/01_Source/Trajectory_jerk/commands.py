from dataclasses import dataclass


@dataclass
class LineCommand:
    distance: float
    heading: float      # degrees relative to current yaw
    speed: float | None = None
    acceleration: float | None = None
    jerk: float | None = None


@dataclass
class TurnCommand:
    angle: float          # degrees (+CW, -CCW)
    radius: float
    speed: float | None = None
    acceleration: float | None = None
    jerk: float | None = None


@dataclass
class RotateCommand:
    angle: float          # degrees (+CW, -CCW)
    yaw_rate: float | None = None
    yaw_acceleration: float | None = None
    yaw_jerk: float | None = None


@dataclass
class HoverCommand:
    duration: float