from dataclasses import dataclass
import math


@dataclass
class Pose:
    x: float
    y: float
    z: float
    yaw: float      # radians


@dataclass
class TrajectoryPoint:
    time: float

    x: float
    y: float
    z: float

    yaw: float

    vx: float
    vy: float
    vz: float

    yaw_rate: float

    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0

    yaw_acceleration: float = 0.0

    # Jerk
    jx: float = 0.0
    jy: float = 0.0
    jz: float = 0.0

    yaw_jerk: float = 0.0