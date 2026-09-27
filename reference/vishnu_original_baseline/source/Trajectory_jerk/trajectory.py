import math
from copy import deepcopy

from pose import Pose, TrajectoryPoint
from commands import (
    LineCommand,
    TurnCommand,
    RotateCommand,
    HoverCommand,
)
from segments import (
    LineSegment,
    ArcSegment,
    RotationSegment,
    HoverSegment
)
from profiles import TrapezoidalProfile, SCurveProfile


class TrajectoryGenerator:

    def __init__(
        self,
        start_pose,
        dt=0.05,
        default_speed=2.0,
        default_acceleration=1.0,
        default_yaw_rate=30.0,
        default_yaw_acceleration=1.0,
        profile_type="trapezoidal",
        default_jerk=1.0,
        default_yaw_jerk=1.0,
    ):
        self.pose = start_pose
        self.dt = dt

        self.default_speed = default_speed
        self.default_acceleration = default_acceleration
        self.default_yaw_rate = default_yaw_rate
        self.default_yaw_acceleration = default_yaw_acceleration

        self.profile_type = profile_type
        self.default_jerk = default_jerk
        self.default_yaw_jerk = default_yaw_jerk

        self.segments = []

    def _make_profile(
        self,
        distance,
        start_speed,
        cruise_speed,
        end_speed,
        max_acceleration,
        max_deceleration,
        max_jerk=None,
    ):
        if self.profile_type == "scurve":
            jerk = max_jerk if max_jerk is not None else self.default_jerk
            return SCurveProfile(
                distance=distance,
                start_speed=start_speed,
                cruise_speed=cruise_speed,
                end_speed=end_speed,
                max_acceleration=max_acceleration,
                max_deceleration=max_deceleration,
                max_jerk=jerk,
            )
        else:
            return TrapezoidalProfile(
                distance=distance,
                start_speed=start_speed,
                cruise_speed=cruise_speed,
                end_speed=end_speed,
                max_acceleration=max_acceleration,
                max_deceleration=max_deceleration,
            )

    def add_line(self, command, motion):
        start = deepcopy(self.pose)
        heading = self.pose.yaw + math.radians(command.heading)

        dx = command.distance * math.cos(heading)
        dy = command.distance * math.sin(heading)

        self.pose.x += dx
        self.pose.y += dy

        end = deepcopy(self.pose)

        self.segments.append(
            LineSegment(start=start, end=end, motion=motion)
        )

    def add_turn(self, command, motion):
        start = deepcopy(self.pose)
        angle = math.radians(command.angle)
        radius = command.radius

        sign = 1.0 if angle >= 0 else -1.0

        heading = self.pose.yaw
        cx = self.pose.x - sign * radius * math.sin(heading)
        cy = self.pose.y + sign * radius * math.cos(heading)

        theta0 = math.atan2(self.pose.y - cy, self.pose.x - cx)
        theta1 = theta0 + angle

        # Compute final position
        x_end = cx + radius * math.cos(theta1)
        y_end = cy + radius * math.sin(theta1)

        self.pose.x = x_end
        self.pose.y = y_end
        self.pose.yaw += angle

        end = deepcopy(self.pose)

        self.segments.append(
            ArcSegment(
                start=start,
                end=end,
                center=(cx, cy),
                radius=radius,
                angle=angle,
                motion=motion,
            )
        )

    def add_rotate(self, command, motion):
        start_yaw = self.pose.yaw
        yaw_change = math.radians(command.angle)
        end_yaw = start_yaw + yaw_change

        self.segments.append(
            RotationSegment(
                start=deepcopy(self.pose),
                end=Pose(
                    x=self.pose.x,
                    y=self.pose.y,
                    z=self.pose.z,
                    yaw=end_yaw,
                ),
                motion=motion,
            )
        )
        self.pose.yaw = end_yaw

    def add_hover(self, command, motion):
        self.segments.append(
            HoverSegment(
                start=deepcopy(self.pose),
                end=deepcopy(self.pose),
                duration=command.duration,
            )
        )

    def generate(self, commands, profiles):
        self.segments = []

        for command, motion in zip(commands, profiles):
            self.add_command(command, motion)

        trajectory = self.sample_segments()
        return trajectory

    def add_command(self, command, motion):
        if isinstance(command, LineCommand):
            self.add_line(command, motion)
        elif isinstance(command, TurnCommand):
            self.add_turn(command, motion)
        elif isinstance(command, RotateCommand):
            self.add_rotate(command, motion)
        elif isinstance(command, HoverCommand):
            self.add_hover(command, motion)
        else:
            raise ValueError(
                f"Unknown command type: {type(command)}"
            )

    # ---------------- Samplers ----------------

    def sample_line(self, segment):
        points = []

        dx = segment.end.x - segment.start.x
        dy = segment.end.y - segment.start.y
        dz = segment.end.z - segment.start.z

        length = math.sqrt(dx * dx + dy * dy + dz * dz)

        if length < 1e-6:
            return []

        ux = dx / length
        uy = dy / length
        uz = dz / length

        profile = self._make_profile(
            distance=length,
            start_speed=segment.motion.start_speed,
            cruise_speed=segment.motion.cruise_speed,
            end_speed=segment.motion.end_speed,
            max_acceleration=segment.motion.max_acceleration,
            max_deceleration=segment.motion.max_deceleration,
            max_jerk=segment.motion.max_jerk,
        )

        states = profile.sample(self.dt)

        for state in states:
            s = state.position / length
            s = max(0.0, min(1.0, s))

            velocity = state.velocity
            acceleration = state.acceleration
            jerk = state.jerk

            points.append(
                TrajectoryPoint(
                    time=0.0,
                    # Position interpolation
                    x=segment.start.x + s * dx,
                    y=segment.start.y + s * dy,
                    z=segment.start.z + s * dz,
                    yaw=segment.start.yaw,
                    # Velocity vector
                    vx=ux * velocity,
                    vy=uy * velocity,
                    vz=uz * velocity,
                    yaw_rate=0.0,
                    # Acceleration vector
                    ax=ux * acceleration,
                    ay=uy * acceleration,
                    az=uz * acceleration,
                    # Jerk vector
                    jx=ux * jerk,
                    jy=uy * jerk,
                    jz=uz * jerk,
                )
            )

        return points

    def sample_arc(self, segment):
        points = []

        cx, cy = segment.center
        radius = segment.radius
        sweep_angle = segment.angle

        theta0 = math.atan2(
            segment.start.y - cy,
            segment.start.x - cx,
        )

        arc_length = abs(sweep_angle) * radius

        if arc_length < 1e-6:
            return []

        profile = self._make_profile(
            distance=arc_length,
            start_speed=segment.motion.start_speed,
            cruise_speed=segment.motion.cruise_speed,
            end_speed=segment.motion.end_speed,
            max_acceleration=segment.motion.max_acceleration,
            max_deceleration=segment.motion.max_deceleration,
            max_jerk=segment.motion.max_jerk,
        )

        states = profile.sample(self.dt)

        sign = 1.0 if sweep_angle >= 0 else -1.0

        for state in states:
            arc_distance = state.position
            theta = theta0 + sign * arc_distance / radius

            x = cx + radius * math.cos(theta)
            y = cy + radius * math.sin(theta)
            z = segment.start.z

            # Yaw follows tangent
            yaw = segment.start.yaw + sign * arc_distance / radius

            # Velocity direction (tangent)
            tangent = theta + sign * math.pi / 2.0
            ux = math.cos(tangent)
            uy = math.sin(tangent)

            velocity = state.velocity
            acceleration = state.acceleration
            jerk = state.jerk

            vx = ux * velocity
            vy = uy * velocity

            ax = ux * acceleration
            ay = uy * acceleration

            jx = ux * jerk
            jy = uy * jerk

            # Angular velocity
            yaw_rate = sign * state.velocity / radius

            points.append(
                TrajectoryPoint(
                    time=0.0,
                    x=x,
                    y=y,
                    z=z,
                    yaw=yaw,
                    vx=vx,
                    vy=vy,
                    vz=0.0,
                    yaw_rate=yaw_rate,
                    ax=ax,
                    ay=ay,
                    az=0.0,
                    jx=jx,
                    jy=jy,
                    jz=0.0,
                )
            )

        return points

    def sample_rotation(self, segment):
        points = []

        yaw0 = segment.start.yaw
        yaw1 = segment.end.yaw

        yaw_change = yaw1 - yaw0

        if abs(yaw_change) < 1e-6:
            return []

        profile = self._make_profile(
            distance=abs(yaw_change),
            start_speed=segment.motion.start_yaw_rate,
            cruise_speed=segment.motion.cruise_yaw_rate,
            end_speed=segment.motion.end_yaw_rate,
            max_acceleration=segment.motion.max_yaw_acceleration,
            max_deceleration=segment.motion.max_yaw_acceleration,
            max_jerk=segment.motion.max_yaw_jerk,
        )

        states = profile.sample(self.dt)

        direction = 1.0 if yaw_change >= 0 else -1.0

        for state in states:
            yaw = yaw0 + direction * state.position

            points.append(
                TrajectoryPoint(
                    time=0.0,
                    x=segment.start.x,
                    y=segment.start.y,
                    z=segment.start.z,
                    yaw=yaw,
                    vx=0.0,
                    vy=0.0,
                    vz=0.0,
                    yaw_rate=direction * state.velocity,
                    ax=0.0,
                    ay=0.0,
                    az=0.0,
                    yaw_acceleration=direction * state.acceleration,
                    jx=0.0,
                    jy=0.0,
                    jz=0.0,
                    yaw_jerk=direction * state.jerk,
                )
            )

        return points

    def sample_hover(self, segment):
        points = []

        steps = max(int(segment.duration / self.dt), 1)

        for _ in range(steps):
            points.append(
                TrajectoryPoint(
                    time=0.0,
                    x=segment.start.x,
                    y=segment.start.y,
                    z=segment.start.z,
                    yaw=segment.start.yaw,
                    vx=0.0,
                    vy=0.0,
                    vz=0.0,
                    yaw_rate=0.0,
                    ax=0.0,
                    ay=0.0,
                    az=0.0,
                    jx=0.0,
                    jy=0.0,
                    jz=0.0,
                )
            )

        return points

    def sample_segments(self):
        trajectory = []
        current_time = 0.0

        for segment in self.segments:
            if isinstance(segment, LineSegment):
                points = self.sample_line(segment)
            elif isinstance(segment, ArcSegment):
                points = self.sample_arc(segment)
            elif isinstance(segment, RotationSegment):
                points = self.sample_rotation(segment)
            elif isinstance(segment, HoverSegment):
                points = self.sample_hover(segment)
            else:
                continue

            # Remove duplicate connection point
            if trajectory:
                points = points[1:]

            # Assign timestamps
            for p in points:
                p.time = current_time
                current_time += self.dt

            trajectory.extend(points)

        return trajectory