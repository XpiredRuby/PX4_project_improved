# PX4 Research Controller

**SITL research candidate. The requested 95+ flight-readiness target is not established.**
The validated software envelope includes a four-second GNSS-outage recovery budget;
combined gusts and a 12-second outage still fail touchdown position limits.
See [validation status](docs/validation_summary.md) and [current evidence](validation/2026-10-04).
The latest bounded-recovery and lifecycle evidence is in
[`validation/2026-10-04-code-logic`](validation/2026-10-04-code-logic).

The RELLIS continuation prioritizes the reusable software layer. Computed-command
expiry, arming/control ownership, stale shutdown evidence, pre-arm plan validation,
and bounded asynchronous evidence writing are described in
[code logic](docs/code-logic.md). Aircraft and sensor integration are deferred.

GPS/IMU mission control for smooth takeoff, trajectory tracking, return, and
landing without a supplied ground height. PX4 owns the final descent and
automatic disarm.

- Strict local-NED waypoint missions with named stops and optional dwell times
- C3 stop boundaries: zero velocity, acceleration, and jerk at every stop
- Analytic retiming against horizontal, vertical, acceleration, jerk, and yaw limits
- Smooth reference motion and bounded velocity-command changes
- GPS/estimator health and source-clock checks, last-trusted-position hold, and failsafe handoff
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
mission duration for braking distance. Four independent October 4 gust/outage
trials verify recovery at the declared four-second boundary; combined wind and
prolonged GPS loss remain a failed case.

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
`EKF2_NOAID_TOUT` must remain inside its supported 0.5–10 s range. For the
default four-second recovery budget, the runner requires at least 6 s: the outage,
PX4's post-outage GNSS health window, and one second of sampling/fusion reserve.
The final recovery trials use 6 s. This timing budget is not a position guarantee;
an earlier 10 s experiment did not solve the 12-second gust/outage failure.

Touchdown quality includes a frozen XY reference (1.5 m maximum displacement),
0.5 m/s horizontal and vertical speed limits, and 10° combined body tilt.
The runner retains any detected contact violation while waiting for PX4 ground,
automatic disarm, and fresh zero propulsion outputs, then reports the failure.
Detecting a violation is evidence of a failed landing, not proof it was avoided.

## Waypoint missions

Waypoint plans use a strict, versioned JSON contract. Coordinates are metres in
`LOCAL_NED`: north, east, and down. At runtime the first stop is aligned to the
recovered launch XY, selected cruise altitude, and captured heading, so the flown
geometry and yaw are defined by offsets from that first stop. Each stop has a unique name, yaw, and
optional hold time. The compiler rejects unknown or missing fields, non-finite
values, coincident stops, envelope violations, and missions whose retimed
duration exceeds the declared budget.

```bash
python trajectory_generator/plan_waypoint_mission.py \
  --plan missions/five_stop_example.json \
  --output five_stop.csv \
  --summary five_stop.summary.json

python controller/offboard_runner.py --trajectory five_stop.csv

python analysis/waypoint_mission_audit.py \
  --log RUN/research_log_ID.csv \
  --summary five_stop.summary.json \
  --manifest RUN/run_manifest_ID.json \
  --output RUN/waypoint_audit.json
```

Every transit is a seventh-order stop-to-stop polynomial. Its position,
velocity, acceleration, and jerk join continuously to a stationary hold, and
the exact endpoint is included. Segment duration is chosen analytically so all
declared translation and yaw limits are respected. The controller independently
rechecks the compiled CSV against its own operating radius, height, speed,
acceleration, jerk, and terrain-clearance limits before any transport connection
or arming request.

This is route planning over explicitly supplied local coordinates, not obstacle
avoidance or terrain mapping. GPS/IMU alone does not provide an obstacle map;
clearance between stops remains the operator's responsibility.

The post-run audit fails unless every named stop has stationary-reference
evidence, bounded 95th-percentile position error and vehicle speed, the entire
route clock was consumed, and touchdown, automatic disarm, and shutdown were
recorded successfully.

## Validation commands

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
truth into the estimator's contemporaneous local frame for offline checking. It
rejects long interpolation gaps, treats estimator-origin changes as discrete, and
independently gates touchdown and every HOLD episode. Simulator truth never enters
the control loop.

[Validation results](docs/validation_summary.md) describe the tested cases and
remaining gaps. SITL evidence does not establish unattended physical-flight readiness.
