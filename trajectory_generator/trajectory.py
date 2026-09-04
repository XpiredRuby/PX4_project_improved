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
from profiles import TrapezoidalProfile, SCurveProfile, sample_times


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
        turn_transition_fraction=0.03,
    ):
        self.start_pose = deepcopy(start_pose)
        self.pose = deepcopy(start_pose)
        self.dt = dt

        self.default_speed = default_speed
        self.default_acceleration = default_acceleration
        self.default_yaw_rate = default_yaw_rate
        self.default_yaw_acceleration = default_yaw_acceleration

        self.profile_type = profile_type
        self.default_jerk = default_jerk
        self.default_yaw_jerk = default_yaw_jerk

        if not 0.0 < turn_transition_fraction < 0.5:
            raise ValueError(
                "turn_transition_fraction must be between 0 and 0.5"
            )
        self.turn_transition_fraction = turn_transition_fraction
        self.turn_integration_steps = 8192

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
        if len(commands) != len(profiles):
            raise ValueError("commands and profiles must have equal length")

        self.pose = deepcopy(self.start_pose)
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
                    time=state.time,
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

    @staticmethod
    def _smoothstep5(value):
        return (
            10.0 * value ** 3
            - 15.0 * value ** 4
            + 6.0 * value ** 5
        )

    @staticmethod
    def _smoothstep5_derivative(value):
        return (
            30.0 * value ** 2
            - 60.0 * value ** 3
            + 30.0 * value ** 4
        )

    @staticmethod
    def _smoothstep5_second_derivative(value):
        return (
            60.0 * value
            - 180.0 * value ** 2
            + 120.0 * value ** 3
        )

    @staticmethod
    def _smoothstep5_integral(value):
        return (
            2.5 * value ** 4
            - 3.0 * value ** 5
            + value ** 6
        )

    def _turn_shape(self, progress):
        fraction = self.turn_transition_fraction
        progress = max(0.0, min(1.0, progress))

        if progress < fraction:
            local = progress / fraction
            shape = self._smoothstep5(local)
            shape_u = (
                self._smoothstep5_derivative(local) / fraction
            )
            shape_uu = (
                self._smoothstep5_second_derivative(local)
                / (fraction ** 2)
            )
            integral = (
                fraction * self._smoothstep5_integral(local)
            )
        elif progress <= 1.0 - fraction:
            shape = 1.0
            shape_u = 0.0
            shape_uu = 0.0
            integral = progress - 0.5 * fraction
        else:
            local = (1.0 - progress) / fraction
            shape = self._smoothstep5(local)
            shape_u = (
                -self._smoothstep5_derivative(local) / fraction
            )
            shape_uu = (
                self._smoothstep5_second_derivative(local)
                / (fraction ** 2)
            )
            integral = (
                1.0
                - fraction
                - fraction * self._smoothstep5_integral(local)
            )

        return shape, shape_u, shape_uu, integral

    def _build_smooth_turn_geometry(self, segment):
        angle = segment.angle
        fraction = self.turn_transition_fraction
        steps = self.turn_integration_steps
        normalization = 1.0 - fraction

        local_x = [0.0]
        local_y = [0.0]
        previous_heading = 0.0
        previous_tx = 1.0
        previous_ty = 0.0

        for index in range(1, steps + 1):
            progress = index / steps
            _, _, _, integral = self._turn_shape(progress)
            heading = angle * integral / normalization
            tx = math.cos(heading)
            ty = math.sin(heading)
            du = 1.0 / steps
            local_x.append(
                local_x[-1] + 0.5 * (previous_tx + tx) * du
            )
            local_y.append(
                local_y[-1] + 0.5 * (previous_ty + ty) * du
            )
            previous_heading = heading
            previous_tx = tx
            previous_ty = ty

        raw_x = local_x[-1]
        raw_y = local_y[-1]
        raw_norm = math.hypot(raw_x, raw_y)

        dx = segment.end.x - segment.start.x
        dy = segment.end.y - segment.start.y
        cosine = math.cos(segment.start.yaw)
        sine = math.sin(segment.start.yaw)
        target_x = cosine * dx + sine * dy
        target_y = -sine * dx + cosine * dy
        target_norm = math.hypot(target_x, target_y)

        if raw_norm < 1e-9 or target_norm < 1e-9:
            raise ValueError(
                "Smooth turns require a nonzero endpoint chord; "
                "full-circle turns must be split into smaller turns"
            )

        alignment_error = abs(
            raw_x * target_y - raw_y * target_x
        ) / (raw_norm * target_norm)
        direction = raw_x * target_x + raw_y * target_y
        if alignment_error > 1e-6 or direction <= 0.0:
            raise ValueError(
                "Smooth-turn integration did not align with endpoint "
                f"(alignment_error={alignment_error:.3g})"
            )

        path_length = target_norm / raw_norm
        return path_length, local_x, local_y

    def _smooth_turn_position(
        self,
        segment,
        progress,
        path_length,
        local_x,
        local_y,
    ):
        progress = max(0.0, min(1.0, progress))
        if progress >= 1.0:
            return segment.end.x, segment.end.y

        scaled = progress * self.turn_integration_steps
        index = min(
            int(scaled),
            self.turn_integration_steps - 1,
        )
        ratio = scaled - index
        x_local = (
            local_x[index]
            + ratio * (local_x[index + 1] - local_x[index])
        ) * path_length
        y_local = (
            local_y[index]
            + ratio * (local_y[index + 1] - local_y[index])
        ) * path_length

        cosine = math.cos(segment.start.yaw)
        sine = math.sin(segment.start.yaw)
        return (
            segment.start.x + cosine * x_local - sine * y_local,
            segment.start.y + sine * x_local + cosine * y_local,
        )

    def sample_arc(self, segment):
        points = []
        (
            path_length,
            local_x,
            local_y,
        ) = self._build_smooth_turn_geometry(segment)

        profile = self._make_profile(
            distance=path_length,
            start_speed=segment.motion.start_speed,
            cruise_speed=segment.motion.cruise_speed,
            end_speed=segment.motion.end_speed,
            max_acceleration=segment.motion.max_acceleration,
            max_deceleration=segment.motion.max_deceleration,
            max_jerk=segment.motion.max_jerk,
        )
        states = profile.sample(self.dt)

        angle = segment.angle
        normalization = 1.0 - self.turn_transition_fraction

        for state in states:
            progress = max(
                0.0,
                min(1.0, state.position / path_length),
            )
            shape, shape_u, shape_uu, integral = self._turn_shape(
                progress
            )
            yaw_offset = angle * integral / normalization
            yaw = segment.start.yaw + yaw_offset
            x, y = self._smooth_turn_position(
                segment,
                progress,
                path_length,
                local_x,
                local_y,
            )

            tx = math.cos(yaw)
            ty = math.sin(yaw)
            nx = -ty
            ny = tx

            curvature = (
                angle * shape
                / (path_length * normalization)
            )
            curvature_s = (
                angle * shape_u
                / (path_length ** 2 * normalization)
            )
            curvature_ss = (
                angle * shape_uu
                / (path_length ** 3 * normalization)
            )

            velocity = state.velocity
            tangential_acceleration = state.acceleration
            tangential_jerk = state.jerk

            vx = tx * velocity
            vy = ty * velocity
            normal_acceleration = velocity ** 2 * curvature
            ax = (
                tx * tangential_acceleration
                + nx * normal_acceleration
            )
            ay = (
                ty * tangential_acceleration
                + ny * normal_acceleration
            )

            tangent_jerk = (
                tangential_jerk
                - velocity ** 3 * curvature ** 2
            )
            normal_jerk = (
                3.0
                * velocity
                * tangential_acceleration
                * curvature
                + velocity ** 3 * curvature_s
            )
            jx = tx * tangent_jerk + nx * normal_jerk
            jy = ty * tangent_jerk + ny * normal_jerk

            yaw_rate = curvature * velocity
            yaw_acceleration = (
                curvature * tangential_acceleration
                + curvature_s * velocity ** 2
            )
            yaw_jerk = (
                curvature * tangential_jerk
                + 3.0
                * curvature_s
                * velocity
                * tangential_acceleration
                + curvature_ss * velocity ** 3
            )

            points.append(
                TrajectoryPoint(
                    time=state.time,
                    x=x,
                    y=y,
                    z=segment.start.z,
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
                    yaw_acceleration=yaw_acceleration,
                    yaw_jerk=yaw_jerk,
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
                    time=state.time,
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

        for local_time in sample_times(segment.duration, self.dt):
            points.append(
                TrajectoryPoint(
                    time=local_time,
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
                    yaw_acceleration=0.0,
                    yaw_jerk=0.0,
                )
            )

        return points

    def sample_segments(self):
        trajectory = []

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
                raise ValueError(
                    f"Unknown segment type: {type(segment)}"
                )

            if not points:
                continue

            if trajectory:
                previous = trajectory[-1]
                connection = points[0]
                position_gap = math.sqrt(
                    (connection.x - previous.x) ** 2
                    + (connection.y - previous.y) ** 2
                    + (connection.z - previous.z) ** 2
                )
                yaw_gap = math.atan2(
                    math.sin(connection.yaw - previous.yaw),
                    math.cos(connection.yaw - previous.yaw),
                )
                if position_gap > 1e-7 or abs(yaw_gap) > 1e-7:
                    raise ValueError(
                        "Discontinuous segment geometry: "
                        f"position_gap={position_gap:.9g}, "
                        f"yaw_gap={yaw_gap:.9g}"
                    )
                segment_start_time = previous.time
                points = points[1:]
            else:
                segment_start_time = 0.0

            for point in points:
                point.time = segment_start_time + point.time
                if trajectory and point.time <= trajectory[-1].time:
                    raise ValueError(
                        "Trajectory timestamps must be strictly increasing"
                    )
                trajectory.append(point)

        if len(trajectory) < 2:
            raise ValueError("trajectory must contain at least two points")
        return trajectory
