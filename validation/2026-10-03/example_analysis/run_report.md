# PX4 Instrumented Run Report

Archive: `/home/xpire/PX4-runs/terrain-final-20261003T062325Z/offset-yaw-final/batch/offset-yaw-final`

## Safety audit

**Overall: PASS**

| Result | Check | Evidence |
|---|---|---|
| PASS | Nominal phase sequence | TAKEOFF -> TRAJECTORY -> RETURN_HOME -> ALIGN -> HANDOFF -> PX4_LAND |
| PASS | Automatic disarm confirmed | final armed=False |
| PASS | PX4 touchdown confirmed | final landed_state=1 |
| PASS | Command envelope respected | checked 3512/3512 commands |
| PASS | Offboard stream continuity | maximum gap=0.109s, limit=0.500s |
| PASS | No telemetry time regression | regression flags=0, unreadable=0 |
| PASS | No navigation loss | LOST samples=0 |
| PASS | Touchdown dynamics within limits | horizontal=0.006m/s (limit=0.500), vertical=0.003m/s (limit=0.500), tilt=0.01deg (limit=10.00) |
| PASS | Touchdown position within limit | error=0.111m (limit=1.500) |
| PASS | No post-touchdown bounce | post-touchdown non-ground samples=0 |
| PASS | Automatic disarm timing | delay=2.648s (limit=5.000) |
| PASS | Propulsion outputs zero after disarm | max_abs_output=0.0000, actuator_age=0.012s |

## Tracking metrics

| Signal | Units | RMSE | MAE | Max abs error | Bias |
|---|---|---|---|---|---|
| x | m | 0.07366 | 0.03543 | 0.42675 | 0.00436 |
| y | m | 0.06630 | 0.02971 | 0.45346 | -0.00129 |
| z | m | 0.00834 | 0.00678 | 0.02372 | -0.00020 |
| planned_vx_vs_actual | m/s | 0.07326 | 0.02857 | 0.49843 | -0.00392 |
| commanded_vx_vs_actual | m/s | 0.09051 | 0.03566 | 0.74778 | -0.00059 |
| planned_vy_vs_actual | m/s | 0.07062 | 0.02537 | 0.52114 | 0.00015 |
| commanded_vy_vs_actual | m/s | 0.08881 | 0.03396 | 0.76816 | -0.00146 |
| planned_vz_vs_actual | m/s | 0.00874 | 0.00710 | 0.02536 | 0.00018 |
| commanded_vz_vs_actual | m/s | 0.00306 | 0.00239 | 0.01356 | -0.00002 |
| yaw | rad | 0.03379 | 0.01607 | 0.17765 | -0.01545 |
| px4_attitude_roll_vs_actual | rad | 0.00126 | 0.00091 | 0.00772 | 0.00001 |
| px4_attitude_pitch_vs_actual | rad | 0.00652 | 0.00261 | 0.05615 | -0.00002 |
| px4_attitude_yaw_vs_actual | rad | 0.02978 | 0.01421 | 0.16020 | -0.01359 |
| px4_body_rate_p_vs_actual | rad/s | 0.00272 | 0.00179 | 0.01988 | -0.00026 |
| px4_body_rate_q_vs_actual | rad/s | 0.00488 | 0.00254 | 0.06509 | -0.00000 |
| px4_body_rate_r_vs_actual | rad/s | 0.01733 | 0.00529 | 0.18208 | -0.00140 |
| retimed_reference_ax_vs_derived_actual | m/s^2 | 0.11773 | 0.04023 | 1.00726 | -0.00002 |
| retimed_reference_ay_vs_derived_actual | m/s^2 | 0.11638 | 0.03670 | 1.02996 | -0.00016 |
| retimed_reference_az_vs_derived_actual | m/s^2 | 0.01577 | 0.01270 | 0.05453 | 0.00008 |

The `planned_*` signals are trajectory feedforward. The `cmd_*` signals are the final PX4 outer-loop velocity commands after PID correction.

## Tracking lag

| Signal | Lag (s) | Correlation |
|---|---|---|
| vx | 0.450 | 0.9976 |
| vy | 0.450 | 0.9962 |
| vz | n/a | n/a |
| yaw_rate | 0.500 | 0.9946 |

Positive lag means the measured response follows the desired signal.

## Controller timing

- Median loop interval: 50.000 ms
- 99th-percentile interval: 50.095 ms
- Maximum interval: 516.952 ms
- Intervals above 75 ms: 7
- Total skipped periods: 12
- Maximum MAVLink setpoint gap: 108.900 ms
- 99th-percentile MAVLink setpoint gap: 50.443 ms
- Watchdog setpoint resends: 6
- Trajectory clock limited samples: 3

## Phase timing

| Phase | Samples | Start (s) | End (s) | Duration (s) |
|---|---|---|---|---|
| TAKEOFF | 878 | 0.000 | 43.850 | 43.850 |
| TRAJECTORY | 2536 | 43.900 | 171.250 | 127.350 |
| RETURN_HOME | 41 | 171.300 | 173.300 | 2.000 |
| ALIGN | 38 | 173.350 | 175.200 | 1.850 |
| HANDOFF | 19 | 175.250 | 176.150 | 0.900 |
| PX4_LAND | 581 | 176.200 | 206.201 | 30.001 |

## Plot files

- `01_position_tracking.png`
- `02_velocity_tracking.png`
- `03_outer_loop_terms.png`
- `04_yaw_tracking.png`
- `05_tracking_errors.png`
- `06_attitude_rates.png`
- `07_timing_telemetry.png`
- `08_path_3d.png`
- `09_inner_loop_signals.png`
- `10_trajectory_derivatives.png`
- `11_outer_inner_loop_tracking.png`
- `12_acceleration_tracking.png`

## Interpretation guardrails

- Yaw error is wrapped to +/- pi; the yaw plot is unwrapped only for visual continuity.
- IMU acceleration is rotated from body to local NED and gravity compensated before comparison. A centered five-sample mean is shown beside the raw derived signal.
- Reference acceleration comparisons differentiate emitted feedforward against elapsed time. Raw planned_a/j columns are original path-clock derivatives. Finite differences do not bridge phase/navigation changes or gaps over 0.25 s; command smoothness is assessed separately.
- Real-flight comparison must use the same metrics after time, origin, yaw, and coordinate-frame alignment.
