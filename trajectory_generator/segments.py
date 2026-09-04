from dataclasses import dataclass
from pose import Pose


@dataclass
class MotionProfile:
    # Linear motion
    start_speed: float
    cruise_speed: float
    end_speed: float

    max_acceleration: float
    max_deceleration: float
    max_jerk: float | None = None

    # Angular motion
    start_yaw_rate: float = 0.0
    cruise_yaw_rate: float = 0.0
    end_yaw_rate: float = 0.0

    max_yaw_acceleration: float = 0.0
    max_yaw_jerk: float | None = None


@dataclass
class LineSegment:
    start: Pose
    end: Pose
    motion: MotionProfile


@dataclass
class ArcSegment:
    start: Pose
    end: Pose
    center: tuple[float, float]
    radius: float
    angle: float
    motion: MotionProfile


@dataclass
class RotationSegment:
    start: Pose
    end: Pose
    motion: MotionProfile


@dataclass
class HoverSegment:
    start: Pose
    end: Pose
    duration: float