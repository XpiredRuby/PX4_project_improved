"""GPS/IMU-only yaw initialization and launch-site recovery."""

from dataclasses import dataclass
import math
import statistics
import time


EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class LaunchReference:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    horizontal_accuracy_m: float
    vertical_accuracy_m: float
    sample_count: int


def quaternion_from_euler(roll, pitch, yaw):
    """Return a MAVLink scalar-first quaternion."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def home_from_geodetic_sample(
    local_xyz,
    current_geodetic,
    launch_reference,
):
    """Project the saved launch GPS fix into PX4's current local NED frame."""
    x, y, z = local_xyz
    latitude_deg, longitude_deg, altitude_m = current_geodetic
    mean_latitude = math.radians(
        (latitude_deg + launch_reference.latitude_deg) / 2.0
    )
    north_m = (
        math.radians(launch_reference.latitude_deg - latitude_deg)
        * EARTH_RADIUS_M
    )
    east_m = (
        math.radians(launch_reference.longitude_deg - longitude_deg)
        * EARTH_RADIUS_M
        * math.cos(mean_latitude)
    )
    down_m = altitude_m - launch_reference.altitude_m
    return x + north_m, y + east_m, z + down_m


def bootstrap_thrust(
    z,
    vz,
    target_z,
    hover_thrust,
    px4_max_thrust,
    config,
):
    """Altitude-hold thrust used only during yaw-observability bootstrap."""
    command = (
        hover_thrust
        + config.bootstrap_thrust_bias
        + config.bootstrap_vertical_kp * (z - target_z)
        + config.bootstrap_vertical_kd * vz
    )
    maximum = min(config.bootstrap_max_thrust, px4_max_thrust - 0.02)
    if maximum <= config.bootstrap_min_thrust:
        raise RuntimeError("PX4 thrust ceiling is below bootstrap minimum")
    return max(config.bootstrap_min_thrust, min(command, maximum))


def _raw_navigation_reasons(controller, snapshot):
    """Checks available before GPS horizontal fusion has initialized."""
    config = controller.config
    reasons = []
    if snapshot["position_age_s"] > controller.max_position_age_s:
        reasons.append("vertical position telemetry stale")
    if snapshot["attitude_age_s"] > controller.max_attitude_age_s:
        reasons.append("attitude telemetry stale")
    if snapshot["gps_age_s"] > config.gps_max_age_s:
        reasons.append("GPS telemetry stale")
    if snapshot["gps_source_age_s"] > config.gps_max_age_s:
        reasons.append("GPS measurement timestamp stopped advancing")
    if snapshot["gps_source_regressed"]:
        reasons.append("GPS measurement timestamp moved backwards")
    if snapshot["gps_fix_type"] < config.gps_min_fix_type:
        reasons.append("GPS has no 3D fix")
    if snapshot["gps_satellites_visible"] < config.gps_min_satellites:
        reasons.append("GPS satellite count is below the configured minimum")
    if snapshot["gps_hdop"] > config.gps_max_hdop:
        reasons.append("GPS HDOP exceeds the configured maximum")
    if snapshot["gps_vdop"] > config.gps_max_vdop:
        reasons.append("GPS VDOP exceeds the configured maximum")
    if (
        snapshot["gps_horizontal_accuracy_m"]
        > config.gps_max_horizontal_accuracy_m
    ):
        reasons.append("GPS horizontal accuracy is insufficient")
    if (
        snapshot["gps_vertical_accuracy_m"]
        > config.gps_max_vertical_accuracy_m
    ):
        reasons.append("GPS vertical accuracy is insufficient")
    if snapshot["estimator_age_s"] > config.estimator_max_age_s:
        reasons.append("estimator telemetry stale")

    # Attitude, vertical velocity, and absolute vertical position are enough
    # for the guarded attitude/thrust maneuver. Horizontal states are expected
    # to remain invalid until the GSF yaw estimator observes acceleration.
    vertical_mask = 1 | 4 | 32
    if snapshot["estimator_flags"] & vertical_mask != vertical_mask:
        reasons.append(
            f"estimator flags 0x{snapshot['estimator_flags']:x} "
            f"missing bootstrap mask 0x{vertical_mask:x}"
        )
    vertical_ratio = snapshot["estimator_pos_vert_ratio"]
    if (
        not math.isfinite(vertical_ratio)
        or vertical_ratio > config.estimator_max_test_ratio
    ):
        reasons.append("vertical position innovation is unhealthy")
    return reasons


def wait_for_bootstrap_ready(controller, timeout=None):
    timeout = controller.config.preflight_timeout_s if timeout is None else timeout
    deadline = time.monotonic() + timeout
    last_reasons = ["no bootstrap telemetry received"]
    print("Waiting for GPS, attitude, and vertical-estimator gates...")
    while time.monotonic() < deadline:
        snapshot = controller._snapshot(time.monotonic())
        last_reasons = _raw_navigation_reasons(controller, snapshot)
        if not last_reasons:
            print(
                "[bootstrap] Raw navigation ready: "
                f"fix={snapshot['gps_fix_type']} "
                f"satellites={snapshot['gps_satellites_visible']} "
                f"flags=0x{snapshot['estimator_flags']:x}"
            )
            return snapshot
        if controller.receiver_error is not None:
            raise RuntimeError(
                "MAVLink receiver failed during bootstrap readiness: "
                f"{controller.receiver_error}"
            )
        time.sleep(0.1)
    raise TimeoutError(
        "Navigation did not pass bootstrap gates: " + "; ".join(last_reasons)
    )


def capture_launch_reference(controller):
    """Average the known takeoff GPS location before the vehicle moves."""
    config = controller.config
    deadline = time.monotonic() + config.launch_reference_duration_s
    samples = []
    previous_gps_time = None
    while time.monotonic() < deadline:
        snapshot = controller._snapshot(time.monotonic())
        reasons = _raw_navigation_reasons(controller, snapshot)
        if reasons:
            raise RuntimeError(
                "Navigation degraded while capturing launch GPS: "
                + "; ".join(reasons)
            )
        if abs(snapshot["vz"]) > config.home_max_vertical_speed_m_s:
            raise RuntimeError("Vehicle moved during launch GPS capture")
        if snapshot["gps_time_usec"] != previous_gps_time:
            samples.append(
                (
                    snapshot["gps_lat_deg"],
                    snapshot["gps_lon_deg"],
                    snapshot["gps_alt_m"],
                    snapshot["gps_horizontal_accuracy_m"],
                    snapshot["gps_vertical_accuracy_m"],
                )
            )
            previous_gps_time = snapshot["gps_time_usec"]
        time.sleep(0.02)

    if len(samples) < config.launch_reference_min_samples:
        raise RuntimeError(
            f"Only {len(samples)} launch GPS samples collected; "
            f"need {config.launch_reference_min_samples}"
        )
    latitude = statistics.median(sample[0] for sample in samples)
    longitude = statistics.median(sample[1] for sample in samples)
    altitude = statistics.median(sample[2] for sample in samples)
    reference = LaunchReference(
        latitude_deg=latitude,
        longitude_deg=longitude,
        altitude_m=altitude,
        horizontal_accuracy_m=max(sample[3] for sample in samples),
        vertical_accuracy_m=max(sample[4] for sample in samples),
        sample_count=len(samples),
    )
    print(
        "[bootstrap] Saved launch GPS reference: "
        f"lat={latitude:.8f} lon={longitude:.8f} alt={altitude:.3f}m "
        f"samples={len(samples)}"
    )
    return reference


def send_attitude_target(controller, pitch_rad, thrust, yaw=None):
    snapshot = controller._snapshot(time.monotonic())
    yaw = snapshot["yaw"] if yaw is None else yaw
    quaternion = quaternion_from_euler(0.0, pitch_rad, yaw)
    with controller.mav_send_lock:
        controller.master.mav.set_attitude_target_send(
            controller._estimated_px4_boot_ms(),
            controller.master.target_system,
            controller.master.target_component,
            0b00000111,
            quaternion,
            0.0,
            0.0,
            0.0,
            float(thrust),
        )


def run_magless_yaw_bootstrap(
    controller,
    hover_thrust,
    px4_max_thrust,
):
    """Climb and accelerate until PX4's GPS-velocity yaw is observable."""
    config = controller.config
    initial = controller._snapshot(time.monotonic())
    z0 = initial["z"]
    yaw0 = initial["yaw"]
    target_z = z0 - config.bootstrap_altitude_m
    pitch_command = math.radians(config.bootstrap_pitch_deg)
    tilt_guard = math.radians(config.bootstrap_max_tilt_deg)
    deadline = time.monotonic() + config.bootstrap_timeout_s
    aligned_since = None
    last_status = 0.0

    print(
        "[bootstrap] Starting guarded GPS-velocity yaw initialization: "
        f"target_altitude={config.bootstrap_altitude_m:.1f}m "
        f"pitch={config.bootstrap_pitch_deg:.1f}deg"
    )
    while time.monotonic() < deadline:
        now = time.monotonic()
        snapshot = controller._snapshot(now)
        if not snapshot["armed"]:
            raise RuntimeError("Vehicle disarmed during yaw bootstrap")
        altitude = z0 - snapshot["z"]
        if altitude > config.bootstrap_max_altitude_m:
            raise RuntimeError(
                f"Yaw bootstrap altitude guard exceeded: {altitude:.2f}m"
            )
        if max(abs(snapshot["roll"]), abs(snapshot["pitch"])) > tilt_guard:
            raise RuntimeError("Yaw bootstrap tilt guard exceeded")

        raw_reasons = _raw_navigation_reasons(controller, snapshot)
        if raw_reasons:
            raise RuntimeError(
                "Raw navigation degraded during yaw bootstrap: "
                + "; ".join(raw_reasons)
            )

        full_reasons = controller._navigation_health_reasons(snapshot)
        if not full_reasons:
            if aligned_since is None:
                aligned_since = now
                print(
                    "[bootstrap] Horizontal GPS fusion acquired: "
                    f"flags=0x{snapshot['estimator_flags']:x}"
                )
            commanded_pitch = 0.0
            if now - aligned_since >= config.bootstrap_navigation_confirm_s:
                send_attitude_target(
                    controller,
                    0.0,
                    bootstrap_thrust(
                        snapshot["z"],
                        snapshot["vz"],
                        target_z,
                        hover_thrust,
                        px4_max_thrust,
                        config,
                    ),
                    yaw0,
                )
                print("[bootstrap] GPS/IMU yaw alignment confirmed")
                return snapshot
        else:
            aligned_since = None
            commanded_pitch = (
                pitch_command
                if altitude >= config.bootstrap_pitch_start_altitude_m
                else 0.0
            )

        thrust = bootstrap_thrust(
            snapshot["z"],
            snapshot["vz"],
            target_z,
            hover_thrust,
            px4_max_thrust,
            config,
        )
        send_attitude_target(controller, commanded_pitch, thrust, yaw0)
        if now - last_status >= 1.0:
            print(
                "[bootstrap] "
                f"alt={altitude:.2f}m vz={snapshot['vz']:.2f}m/s "
                f"flags=0x{snapshot['estimator_flags']:x} "
                f"pitch_cmd={math.degrees(commanded_pitch):.1f}deg"
            )
            last_status = now
        time.sleep(controller.control_dt)

    raise TimeoutError("GPS-velocity yaw alignment timed out")


def recover_local_home(controller, launch_reference):
    """Map the saved launch GPS fix into the post-alignment local NED frame."""
    config = controller.config
    deadline = time.monotonic() + config.home_recovery_duration_s
    estimates = []
    previous_gps_time = None
    while time.monotonic() < deadline:
        snapshot = controller._snapshot(time.monotonic())
        reasons = controller._navigation_health_reasons(snapshot)
        if reasons:
            raise RuntimeError(
                "Navigation degraded while recovering local home: "
                + "; ".join(reasons)
            )
        controller.send_velocity(0.0, 0.0, 0.0, snapshot["yaw"])
        if snapshot["gps_time_usec"] != previous_gps_time:
            home = home_from_geodetic_sample(
                (snapshot["x"], snapshot["y"], snapshot["z"]),
                (
                    snapshot["gps_lat_deg"],
                    snapshot["gps_lon_deg"],
                    snapshot["gps_alt_m"],
                ),
                launch_reference,
            )
            estimates.append((*home, snapshot["yaw"], snapshot))
            previous_gps_time = snapshot["gps_time_usec"]
        time.sleep(controller.control_dt)

    if len(estimates) < config.home_recovery_min_samples:
        raise RuntimeError(
            f"Only {len(estimates)} local-home samples collected; "
            f"need {config.home_recovery_min_samples}"
        )
    median_x = statistics.median(item[0] for item in estimates)
    median_y = statistics.median(item[1] for item in estimates)
    median_z = statistics.median(item[2] for item in estimates)
    inliers = [
        item
        for item in estimates
        if (
            math.hypot(item[0] - median_x, item[1] - median_y)
            <= config.home_max_horizontal_spread_m
            and abs(item[2] - median_z) <= config.home_max_vertical_spread_m
        )
    ]
    if len(inliers) < config.home_recovery_min_samples:
        raise RuntimeError("Recovered launch position was not stable")

    controller.x0 = statistics.fmean(item[0] for item in inliers)
    controller.y0 = statistics.fmean(item[1] for item in inliers)
    controller.z0 = statistics.fmean(item[2] for item in inliers)
    mean_sin = statistics.fmean(math.sin(item[3]) for item in inliers)
    mean_cos = statistics.fmean(math.cos(item[3]) for item in inliers)
    controller.yaw0 = math.atan2(mean_sin, mean_cos)
    controller.home_sample_count = len(inliers)
    controller.home_reference_ready = True
    final_snapshot = dict(inliers[-1][4])
    final_snapshot["gps_vertical_accuracy_m"] = max(
        item[4]["gps_vertical_accuracy_m"] for item in inliers
    )
    print(
        "[bootstrap] Recovered launch point in local NED: "
        f"x={controller.x0:.3f} y={controller.y0:.3f} "
        f"z={controller.z0:.3f} samples={len(inliers)}"
    )
    return final_snapshot
