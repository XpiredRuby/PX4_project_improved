"""Slow reference progress when the vehicle falls behind; retain feedback authority."""

import math


class TrackingGovernor:
    def __init__(self, config):
        self.config = config
        self.scale = 1.0
        self.error_m = 0.0

    def update(self, horizontal_error_m, dt):
        if not math.isfinite(horizontal_error_m) or horizontal_error_m < 0:
            raise ValueError("tracking error must be finite and nonnegative")
        if not math.isfinite(dt) or dt < 0:
            raise ValueError("tracking interval must be finite and nonnegative")
        self.error_m = horizontal_error_m
        cfg = self.config
        if not cfg.tracking_governor_enabled:
            self.scale = 1.0
            return self.scale
        fraction = min(1.0, max(0.0, (
            horizontal_error_m - cfg.tracking_slowdown_start_m
        ) / (cfg.tracking_slowdown_full_m - cfg.tracking_slowdown_start_m)))
        target = 1.0 - fraction * (1.0 - cfg.tracking_minimum_speed_scale)
        tau = (cfg.tracking_scale_fall_tau_s if target < self.scale
               else cfg.tracking_scale_rise_tau_s)
        # Cap elapsed time: a stalled loop must not produce an abrupt recovery.
        alpha = -math.expm1(-min(dt, 0.1) / tau)
        self.scale += alpha * (target - self.scale)
        return self.scale
