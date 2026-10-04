# PX4 Research Controller

**SITL research candidate. The requested 95+ readiness target is not established.**
Combined gusts and a 12-second GPS outage still fail touchdown position limits.
See [validation status](docs/validation_summary.md) and [current evidence](validation/2026-10-04).

GPS/IMU mission control for smooth takeoff, trajectory tracking, return, and
landing without a supplied ground height. PX4 owns the final descent and
automatic disarm.

- Smooth reference motion and bounded velocity-command changes
- GPS/estimator health checks, last-trusted-position hold, and failsafe handoff
- Short, bounded position prediction for feedback; raw measurements govern safety
- Runtime touchdown checks, followed by ground, disarm, and zero-output confirmation
- Recorded configuration, source hashes, telemetry, and independent SITL truth audits

| Directory | Contents |
|---|---|
| `controller/` | Current mission controller |
| `trajectory_generator/` | Trajectory generation |
| `analysis/` | Tracking, touchdown, and command-smoothness audits |
| `tests/` | Regression and MAVLink replay tests |
| `validation/` | SITL evidence and reproduction details |
| `reference/original_baseline/` | Original implementation |
| `session_2_robust_landing/` | Earlier validated implementation and evidence |

## Configuration

Motion, navigation, and terrain bounds are in `controller/mission_config.py`.
The default ground-height range is -10 to +10 m relative to takeoff.
It is a bounded terrain assumption; GPS/IMU cannot identify a clear landing site.
The current SITL configuration and propulsion checks use the x500 quadrotor.

Horizontal velocity commands normally change at no more than 1.5 m/s².
During navigation HOLD, the default bound is 3.0 m/s² to brake the inherited
cruise command sooner; vertical slew remains 1.0 m/s². Fresh local velocity
damping (gain 0.8) reduces momentum and oscillation during HOLD. This is a braking
tradeoff, not an increase to reference-trajectory acceleration or speed caps.
The path clock stays frozen in HOLD and the last trusted reference is retained.
The cruise reference is retimed to at most 1.0 m/s horizontally, with a
1.25 m/s normal command cap. HOLD retains the confidence-limited feedback
authority inside the existing 3.0 m/s hard envelope. The slower cruise trades
mission duration for braking distance. The October 3 trials verify the short
GPS-outage cases; combined wind and prolonged GPS loss remain a failed case.

For PX4 v1.17, configure `EKF2_GPS_CTRL=7` (15 with dual-GPS heading),
`EKF2_HGT_REF=1`, `EKF2_MAG_TYPE=5`, and disable `EKF2_BARO_CTRL`,
`EKF2_OF_CTRL`, `EKF2_EV_CTRL`, `EKF2_RNG_CTRL`, and `EKF2_AGP_CTRL`.
Reboot after estimator changes. Single-GPS yaw initialization requires movement.
The runner also checks Offboard-loss landing, automatic landing disarm, and
ordered battery thresholds with a native landing response.

The runner reads and checks `MPC_LAND_SPEED` and `MPC_LAND_CRWL`; it does not
change them during a mission. Both must be positive, crawl must not exceed
landing speed, and landing speed must be at most 0.5 m/s. The final x500 SITL
trials use 0.4 and 0.3 m/s respectively. A 180 s native-landing timeout covers
the configured 40 m maximum descent plus settling and disarm allowance.
`EKF2_NOAID_TOUT` must remain inside its supported 0.5–10 s range; the final
tests use 5 s. Extending it to 10 s did not solve the wind/GPS-loss failure.

Touchdown quality includes a frozen XY reference (1.5 m maximum displacement),
0.5 m/s horizontal and vertical speed limits, and 10° combined body tilt.
The runner retains any detected contact violation while waiting for PX4 ground,
automatic disarm, and fresh zero propulsion outputs, then reports the failure.
Detecting a violation is evidence of a failed landing, not proof it was avoided.

## Validation

```bash
python -m pip install -r requirements-dev.txt
python -m ruff check controller analysis scripts tests trajectory_generator
python scripts/validate.py
```

`scripts/validate.py` generates the trajectory, compiles the modules, and runs
all regression tests. For an already configured, isolated x500 SITL instance,
the mission entrypoint is `python controller/offboard_runner.py`; it uses
MAVLink UDP 14540. Fault and surface tests additionally use UDP 14601, so only
one monitor/injector may own that receiver during a trial. Do not run these
fault scripts against an aircraft.

`python analysis/analyze_run.py RUN_DIRECTORY` produces the tracking plots and
safety audit. Acceleration tracking uses derivatives of the emitted, retimed
velocity reference against actual elapsed time; raw path-clock derivatives
remain available separately. `analysis/sitl_ground_truth.py` projects simulator
truth into the estimator's contemporaneous local frame for offline checking.
Simulator truth never enters the control loop.

[Validation results](docs/validation_summary.md) describe the tested cases and
remaining gaps. SITL evidence does not establish unattended physical-flight readiness.
