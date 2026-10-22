# PX4 Trajectory Studio and Research Controller

Draw a route or answer the mission interview, complete a reusable aircraft
profile, independently verify the trajectory, inspect its interpretation and
confirm an exact-artifact Gazebo rehearsal, with a controlled simulator-only cancellation request if needed. Start with the
[studio quickstart](docs/trajectory_studio_quickstart.md) and
[aircraft setup](docs/aircraft_profiles.md). The separate
[physical release guide](docs/physical_flight_release.md) documents the local
qualification, prepared bundle, supervised launch and abort CLI.

Planning supports 4–16 parallel rotors; the current PX4 execution adapter
supports 4–12. Fixed-wing, VTOL and other autopilots are explicitly unsupported.
Only x500 has connected simulator evidence; no physical aircraft is qualified
by the checked-in examples.

The validated software envelope includes a four-second GNSS-outage recovery budget;
combined gusts and a 12-second outage still fail touchdown position limits.
See [validation status](docs/validation_summary.md) and
[five-stop mission evidence](validation/2026-10-05-waypoint).
The latest bounded-recovery and lifecycle evidence is in
[`validation/2026-10-04-code-logic`](validation/2026-10-04-code-logic).
The researched architecture, requirements, trade study, and staged delivery plan
for drawn and parameter-driven missions are in the
[physics-constrained trajectory planner plan](docs/trajectory_planner_architecture.md).
Milestone 0 contract coverage is tracked in the
[planner requirements traceability matrix](docs/trajectory_planner_traceability.md).

Computed-command
expiry, arming/control ownership, stale shutdown evidence, pre-arm plan validation,
and bounded asynchronous evidence writing are described in
[code logic](docs/code-logic.md). Physical identification and bench/flight
qualification remain required for each loaded aircraft configuration.

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

## Trajectory and simulation acceptance

The default studio uses the configured reference simulator so you can draw and
rehearse without completing an aircraft questionnaire. Advanced profiles remain
optional. The [acceptance record](validation/2026-10-19-trajectory-acceptance/README.md)
contains the expanded offline/Gazebo/browser results; follow the
[manual trial](docs/trajectory_simulation_acceptance.md#manual-trial) to test your own drawings.

## Configuration

Motion, navigation, and terrain bounds are in `controller/mission_config.py`.
The default ground-height range is -10 to +10 m relative to takeoff.
It is a bounded terrain assumption; GPS/IMU cannot identify a clear landing site.
The connected reference simulator is the x500 quadrotor. Reusable profiles
and registered simulator bindings support other multicopter configurations.
Physical output mapping and disarmed values come from reviewed bench evidence.

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

## Drawn-path research baseline

The first `SITL_ONLY` general-planner slice accepts a strict `LOCAL_NED`
polyline, constructs clamped quintic B-spline geometry, proves a conservative
symmetric deviation bound against the source stroke, applies analytic
speed/acceleration/jerk retiming, independently verifies the candidate, and
exports the existing controller CSV format.

```bash
python -m mission_planner.plan_drawn_path \
  --intent missions/drawn_path_example.json \
  --vehicle config/planner/x500_sitl_vehicle_model.json \
  --environment config/planner/empty_world_environment.json \
  --trajectory drawn-path.csv \
  --report drawn-path.verification.json \
  --summary drawn-path.summary.json

python controller/offboard_runner.py \
  --trajectory drawn-path.csv \
  --planner-summary drawn-path.summary.json \
  --sitl-acknowledgement drawn-path.acknowledgement.json
```

The acknowledgement must be issued for the exact displayed CSV/report after
review. The studio manages that SITL review flow; the low-level runner is a
research/SITL entrypoint. Use the separate physical-release workflow for a real
aircraft.

The compiler flies open paths and closed laps rest-to-rest. A periodic curve
starts and stops at its seam; extra laps repeat that loop. Interior hard points
are full stops. Yaw is either a fixed heading or a tangent profile whose error
against the true horizontal heading is certified. Tilt, thrust, and body rate
are bounded from the acceleration the timing already certified, and the duration
is stretched until those bounds fit. An interview for waypoints, a circle, or a
survey lawnmower compiles through the same path. The review page shows the
route and the binding constraint and states that the artifact is not a clearance
to fly.

Look-at yaw, exact fly-through points, a hashed occupancy map, and a paired-rotor
yaw-torque check are included. A drawn-path execution requires an unexpired
SITL acknowledgement, which is not a flight clearance. A circle interview can
add an entry, an exit, or both: the laps are one open path, the lead-in meets
the seam without stopping, and the path stops at the exit or back on the seam.
The speed on a flown-through seam is the certified schedule. Yaw can follow an
explicit heading profile, certified to the requested samples on one
rest-to-rest piece. A path that ends while still moving remains unsupported.
Profile-based power/time/reserve budgets, registered local Gazebo campaigns
and a separate human-qualified physical-release workflow are implemented.
The x500 model is a simulator configuration, not an identified physical aircraft.
The compiler binds the expected controller cruise height into its summary.
The runner checks both that summary's trajectory hash and the live
uncertainty-adjusted cruise height before execution; a mismatch is rejected.

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

python controller/offboard_runner.py \
  --trajectory five_stop.csv \
  --planner-summary five_stop.summary.json

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

The checked-in five-stop example completed an isolated x500 SITL mission on
October 5, 2026. All five stop audits passed; estimated trajectory XY error was
0.109 m p95, independent simulator-truth XY error was 0.194 m p95, and physical
touchdown error was 0.062 m. PX4 then confirmed ground contact, automatic
disarm, and zero propulsion. These results validate the named-stop route in
SITL, not obstacle avoidance or unattended physical flight.

## Validation commands

```bash
python -m pip install -r requirements-dev.txt
python -m ruff check controller mission_planner analysis scripts tests trajectory_generator
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
