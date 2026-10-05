from dataclasses import dataclass


@dataclass(frozen=True)
class MissionConfig:
    """Safety and motion limits for the GPS/IMU-only mission flow."""

    # The expected landing surface may be 10 m above or below the launch
    # point.  The cruise portion stays above the highest expected surface;
    # PX4 LAND performs the final descent without a supplied ground height.
    ground_offset_min_m: float = -10.0
    ground_offset_max_m: float = 10.0
    terrain_clearance_m: float = 5.0
    cruise_height_m: float = 15.0
    vertical_accuracy_multiplier: float = 2.0
    minimum_vertical_uncertainty_margin_m: float = 1.0
    altitude_tracking_margin_m: float = 1.0
    clearance_budget_hysteresis_m: float = 0.5

    takeoff_max_speed_m_s: float = 0.8
    return_max_speed_m_s: float = 2.0
    max_accel_m_s2: float = 1.0
    max_jerk_m_s3: float = 1.5
    minimum_segment_duration_s: float = 2.0
    command_xy_accel_limit_m_s2: float = 1.5
    # Faster bounded braking during navigation HOLD; normal motion is unchanged.
    navigation_hold_accel_limit_m_s2: float = 3.0
    navigation_hold_velocity_damping: float = 0.8
    command_z_accel_limit_m_s2: float = 1.0
    tracking_governor_enabled: bool = True
    tracking_slowdown_start_m: float = 0.50
    tracking_slowdown_full_m: float = 2.0
    tracking_minimum_speed_scale: float = 0.35
    tracking_scale_fall_tau_s: float = 0.30
    tracking_scale_rise_tau_s: float = 2.0
    position_prediction_enabled: bool = True
    position_prediction_max_age_s: float = 0.075
    position_prediction_max_displacement_m: float = 0.15
    # Retimed cruise reference leaves feedback room inside the normal cap.
    cruise_reference_speed_m_s: float = 1.0
    cruise_command_speed_limit_m_s: float = 1.25
    max_horizontal_speed_m_s: float = 3.0
    max_vertical_speed_m_s: float = 1.0
    max_yaw_rate_deg_s: float = 45.0
    max_yaw_acceleration_deg_s2: float = 60.0
    max_yaw_jerk_deg_s3: float = 120.0
    setpoint_watchdog_timeout_s: float = 0.10
    offboard_stream_max_gap_s: float = 0.50
    # Resending transport packets must not extend a computed command forever.
    max_control_command_age_s: float = 0.25

    # A single-antenna GPS cannot observe absolute yaw while stationary.
    # PX4's GSF yaw estimator needs a short horizontal acceleration before
    # GPS horizontal fusion can begin when the magnetometer is disabled.
    bootstrap_altitude_m: float = 2.0
    bootstrap_pitch_deg: float = -10.0
    bootstrap_pitch_start_altitude_m: float = 0.75
    bootstrap_max_altitude_m: float = 4.0
    bootstrap_max_tilt_deg: float = 25.0
    bootstrap_timeout_s: float = 18.0
    bootstrap_navigation_confirm_s: float = 1.0
    bootstrap_thrust_bias: float = 0.12
    bootstrap_vertical_kp: float = 0.12
    bootstrap_vertical_kd: float = 0.16
    bootstrap_min_thrust: float = 0.45
    bootstrap_max_thrust: float = 0.90
    launch_reference_duration_s: float = 3.0
    launch_reference_min_samples: int = 10
    home_recovery_duration_s: float = 2.0
    home_recovery_min_samples: int = 6

    max_operating_radius_m: float = 80.0
    max_height_above_launch_m: float = 30.0
    max_offboard_drop_below_launch_m: float = 2.0
    max_measured_speed_m_s: float = 8.0

    takeoff_position_tolerance_m: float = 0.20
    takeoff_speed_tolerance_m_s: float = 0.25
    takeoff_settle_s: float = 0.75

    align_xy_tolerance_m: float = 0.35
    align_z_tolerance_m: float = 0.35
    align_horizontal_speed_m_s: float = 0.25
    align_vertical_speed_m_s: float = 0.20
    align_tilt_deg: float = 8.0
    align_angular_rate_deg_s: float = 20.0
    align_hold_s: float = 1.0

    gps_min_fix_type: int = 3
    gps_min_satellites: int = 6
    gps_max_hdop: float = 2.5
    gps_max_vdop: float = 3.5
    gps_max_horizontal_accuracy_m: float = 5.0
    gps_max_vertical_accuracy_m: float = 8.0
    gps_good_hdop: float = 1.0
    gps_good_vdop: float = 1.5
    gps_good_horizontal_accuracy_m: float = 1.0
    gps_good_vertical_accuracy_m: float = 2.0
    gps_max_age_s: float = 1.5
    estimator_max_age_s: float = 1.5
    estimator_max_test_ratio: float = 1.0
    estimator_good_test_ratio: float = 0.35
    minimum_navigation_speed_scale: float = 0.25
    navigation_degraded_entry_s: float = 0.10
    # Brake promptly after persistent hard navigation failure; a long delay
    # lets a moving vehicle travel well beyond the last trusted reference.
    navigation_hold_entry_s: float = 0.20
    navigation_abort_s: float = 8.0
    # Configuration budget, not a promise of recovery or position accuracy.
    gps_outage_recovery_budget_s: float = 4.0
    navigation_recovery_s: float = 2.0
    navigation_confidence_fall_tau_s: float = 0.30
    navigation_confidence_rise_tau_s: float = 2.0

    home_sample_duration_s: float = 3.0
    home_min_samples: int = 30
    home_max_horizontal_speed_m_s: float = 0.20
    home_max_vertical_speed_m_s: float = 0.15
    home_max_horizontal_spread_m: float = 1.50
    home_max_vertical_spread_m: float = 2.00
    home_required_horizontal_accuracy_m: float = 1.50
    home_max_yaw_spread_deg: float = 15.0

    preflight_timeout_s: float = 30.0
    mission_timeout_s: float = 240.0
    return_recovery_timeout_s: float = 120.0
    # Worst bounded descent is 30 m above launch to a surface 10 m below it.
    # Native LAND must remain slow at any height, with room for settling/disarm.
    land_timeout_s: float = 180.0
    max_offboard_loss_timeout_s: float = 1.0

    def validate(self):
        import math
        from dataclasses import fields

        booleans = {"tracking_governor_enabled", "position_prediction_enabled"}
        signed = {"ground_offset_min_m", "ground_offset_max_m", "bootstrap_pitch_deg",
                  "bootstrap_thrust_bias"}
        nonnegative = {"bootstrap_min_thrust", "navigation_degraded_entry_s",
                       "navigation_hold_velocity_damping"}
        for field in fields(self):
            value = getattr(self, field.name)
            if field.name in booleans:
                if not isinstance(value, bool):
                    raise ValueError(f"{field.name} must be boolean")
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{field.name} must be numeric")
            if not math.isfinite(value):
                raise ValueError(f"{field.name} must be finite")
            if field.name not in signed:
                if value < 0 or (value == 0 and field.name not in nonnegative):
                    raise ValueError(f"{field.name} must be positive or explicitly allow zero")
        for name in ("launch_reference_min_samples", "home_recovery_min_samples",
                     "home_min_samples", "gps_min_fix_type", "gps_min_satellites"):
            if not isinstance(getattr(self, name), int):
                raise ValueError(f"{name} must be an integer")
        if not 3 <= self.gps_min_fix_type <= 8 or self.gps_min_satellites > 254:
            raise ValueError("GPS fix and satellite gates are invalid")
        if not 0 < abs(self.bootstrap_pitch_deg) < self.bootstrap_max_tilt_deg < 90:
            raise ValueError("bootstrap pitch must stay inside the tilt guard")
        if not -1 <= self.bootstrap_thrust_bias <= 1:
            raise ValueError("bootstrap thrust bias must be in [-1, 1]")
        if not (self.takeoff_max_speed_m_s <= self.max_vertical_speed_m_s
                and self.return_max_speed_m_s <= self.max_horizontal_speed_m_s):
            raise ValueError("phase speeds must fit the command envelope")
        if not isinstance(self.tracking_governor_enabled, bool):
            raise ValueError("tracking_governor_enabled must be boolean")
        if not isinstance(self.position_prediction_enabled, bool):
            raise ValueError("position_prediction_enabled must be boolean")
        if not (0 < self.position_prediction_max_age_s <= .1
                and 0 < self.position_prediction_max_displacement_m <= .25):
            raise ValueError("position prediction must remain short and bounded")
        if not (
            0 < self.tracking_slowdown_start_m < self.tracking_slowdown_full_m
            and 0 < self.tracking_minimum_speed_scale <= 1
            and self.tracking_scale_fall_tau_s > 0
            and self.tracking_scale_rise_tau_s > 0
        ):
            raise ValueError("tracking governor limits are inconsistent")
        if not (self.command_xy_accel_limit_m_s2
                <= self.navigation_hold_accel_limit_m_s2 <= 3.0):
            raise ValueError("HOLD acceleration must be between the normal limit and 3 m/s²")
        if not 0.0 <= self.navigation_hold_velocity_damping <= 1.5:
            raise ValueError("HOLD velocity damping must be in [0, 1.5]")
        if not (0.0 < self.cruise_reference_speed_m_s
                < self.cruise_command_speed_limit_m_s
                <= self.max_horizontal_speed_m_s):
            raise ValueError("Cruise reference and command speed limits are inconsistent")
        required_height = self.ground_offset_max_m + self.terrain_clearance_m
        if self.cruise_height_m < required_height:
            raise ValueError(
                "cruise_height_m must be at least "
                "ground_offset_max_m + terrain_clearance_m"
            )
        if self.cruise_height_m > self.max_height_above_launch_m:
            raise ValueError(
                "cruise_height_m exceeds max_height_above_launch_m"
            )
        if self.ground_offset_min_m > self.ground_offset_max_m:
            raise ValueError("ground uncertainty limits are reversed")
        if min(
            self.takeoff_max_speed_m_s,
            self.return_max_speed_m_s,
            self.max_accel_m_s2,
            self.max_jerk_m_s3,
            self.command_xy_accel_limit_m_s2,
            self.command_z_accel_limit_m_s2,
            self.max_horizontal_speed_m_s,
            self.max_vertical_speed_m_s,
            self.max_yaw_rate_deg_s,
            self.max_yaw_acceleration_deg_s2,
            self.max_yaw_jerk_deg_s3,
            self.setpoint_watchdog_timeout_s,
            self.offboard_stream_max_gap_s,
            self.bootstrap_altitude_m,
            self.bootstrap_pitch_start_altitude_m,
            self.bootstrap_max_altitude_m,
            self.bootstrap_max_tilt_deg,
            self.bootstrap_timeout_s,
            self.bootstrap_navigation_confirm_s,
            self.bootstrap_vertical_kp,
            self.bootstrap_vertical_kd,
            self.launch_reference_duration_s,
            self.home_recovery_duration_s,
        ) <= 0.0:
            raise ValueError("motion limits must be positive")
        if not (
            self.bootstrap_altitude_m < self.bootstrap_max_altitude_m
            and self.bootstrap_pitch_start_altitude_m
            < self.bootstrap_altitude_m
            and 0.0 <= self.bootstrap_min_thrust
            < self.bootstrap_max_thrust <= 1.0
        ):
            raise ValueError("magless bootstrap limits are inconsistent")
        if min(
            self.launch_reference_min_samples,
            self.home_recovery_min_samples,
        ) < 3:
            raise ValueError("GPS reference sample counts must be at least 3")
        if not (self.setpoint_watchdog_timeout_s < self.max_control_command_age_s
                <= self.offboard_stream_max_gap_s):
            raise ValueError("control command freshness must fit the Offboard gap budget")
        if self.setpoint_watchdog_timeout_s >= self.offboard_stream_max_gap_s:
            raise ValueError(
                "setpoint watchdog must resend before the Offboard gap limit"
            )
        if not 0.0 < self.minimum_navigation_speed_scale <= 1.0:
            raise ValueError("minimum navigation speed scale must be in (0, 1]")
        if not (
            0.0
            <= self.navigation_degraded_entry_s
            < self.navigation_hold_entry_s
            < self.navigation_abort_s
        ):
            raise ValueError("navigation supervision times are inconsistent")
        if min(
            self.navigation_recovery_s,
            self.navigation_confidence_fall_tau_s,
            self.navigation_confidence_rise_tau_s,
            self.home_sample_duration_s,
            self.altitude_tracking_margin_m,
        ) <= 0.0:
            raise ValueError(
                "navigation timing and safety margins must be positive"
            )
        if self.home_min_samples < 3:
            raise ValueError("home_min_samples must be at least 3")
        if min(
            self.vertical_accuracy_multiplier,
            self.minimum_vertical_uncertainty_margin_m,
            self.clearance_budget_hysteresis_m,
            self.home_max_horizontal_speed_m_s,
            self.home_max_vertical_speed_m_s,
            self.home_max_horizontal_spread_m,
            self.home_max_vertical_spread_m,
            self.home_required_horizontal_accuracy_m,
            self.home_max_yaw_spread_deg,
            self.return_recovery_timeout_s,
            self.max_offboard_loss_timeout_s,
        ) <= 0.0:
            raise ValueError("navigation limits and recovery timeout must be positive")
        if not (
            self.gps_good_hdop < self.gps_max_hdop
            and self.gps_good_vdop < self.gps_max_vdop
            and self.gps_good_horizontal_accuracy_m
            < self.gps_max_horizontal_accuracy_m
            and self.gps_good_vertical_accuracy_m
            < self.gps_max_vertical_accuracy_m
            and self.estimator_good_test_ratio
            < self.estimator_max_test_ratio
        ):
            raise ValueError("navigation good thresholds must be below limits")
