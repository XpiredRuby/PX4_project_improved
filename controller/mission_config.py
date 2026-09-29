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
    command_z_accel_limit_m_s2: float = 1.0
    max_horizontal_speed_m_s: float = 3.0
    max_vertical_speed_m_s: float = 1.0
    setpoint_watchdog_timeout_s: float = 0.10
    offboard_stream_max_gap_s: float = 0.50

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
    navigation_degraded_entry_s: float = 0.25
    navigation_hold_entry_s: float = 1.0
    navigation_abort_s: float = 8.0
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
    land_timeout_s: float = 120.0
    max_offboard_loss_timeout_s: float = 1.0

    def validate(self):
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
            self.setpoint_watchdog_timeout_s,
            self.offboard_stream_max_gap_s,
        ) <= 0.0:
            raise ValueError("motion limits must be positive")
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
