# Physics-constrained trajectory planner: architecture and delivery plan

Status: proposed architecture, 17 October 2026. This document is a research and
engineering plan, not a flight authorization or a claim of physical-aircraft
readiness.

## 1. Decision summary

Build a deterministic, model-based planner around the controller that already
exists in this repository. Do not place a neural network in the safety-critical
path.

The recommended production pipeline is:

1. Convert either a drawing or a structured interview into one versioned
   `MissionIntent`.
2. Resolve coordinate frame, altitude interpretation, yaw policy, hard versus
   soft geometry, vehicle model, environment, and operating limits before
   planning.
3. Fit a clamped smooth spatial path within an explicit maximum deviation from
   the user's stroke.
4. Construct collision-free corridors only when a trusted obstacle map exists.
   Without one, label the plan obstacle-unverified and prohibit autonomous
   physical launch.
5. Use a conservative spline plus path retiming first. Add a MINCO/GCOPTER-style
   joint space-time optimizer only after the baseline is independently verified.
6. Verify the optimizer's output in an independent acceptance module against
   geometry, geofence, kinematics, flatness-derived thrust/tilt/body-rate limits,
   timing, energy reserve, estimator assumptions, and numeric validity.
7. Run deterministic checks, Monte Carlo SITL, and required fault scenarios.
   Present the predicted path, uncertainty tube, limiting constraints, and
   simulation evidence to the operator.
8. Require an explicit, expiring operator approval of the exact immutable
   artifact hash. Revalidate live state and environment immediately before arm.
9. Execute through the existing PX4 Offboard controller while preserving its
   command-expiry, navigation supervision, failsafe handoff, native landing, and
   evidence pipeline.

Time is a soft objective, not a hard constraint by default. The lexicographic
objective is:

1. satisfy legal and safety constraints;
2. satisfy vehicle and environment constraints;
3. minimize deviation from accepted user intent;
4. preserve tracking margin and energy reserve;
5. minimize duration.

If the user requests a deadline below the minimum feasible duration, the planner
must reject it and report the minimum feasible duration and active constraints.
It must never silently relax a safety limit.

## 2. What exists and what does not

The current production path is:

```text
strict LOCAL_NED waypoint JSON
  -> seventh-order stop-to-stop compiler
  -> endpoint-inclusive trajectory CSV
  -> pymavlink Offboard controller at 20 Hz
  -> PX4 v1.17 / Gazebo x500
  -> telemetry, independent truth alignment, and post-run audits
```

Existing strengths that should be retained:

- strict, versioned waypoint input and rejection of unknown/non-finite fields;
- analytic speed, acceleration, jerk, yaw-rate, yaw-acceleration, and yaw-jerk
  retiming;
- stationary C3 boundaries at named stops;
- independent pre-arm revalidation by the controller;
- computed-command expiration, Offboard stream supervision, and PX4 loss action;
- navigation `HEALTHY/DEGRADED/HOLD/LOST` handling;
- return, alignment, native PX4 landing, automatic disarm, and zero-output checks;
- deterministic tests, captured MAVLink replay, fault injection, and checked-in
  SITL evidence.

Current boundaries:

- only straight stop-to-stop geometry is supported by the documented planner;
- there is no drawing UI, map/altitude interpretation, obstacle map, safe
  corridor, online replanning, or general continuous-curve planner;
- physics limits are point-mass derivative limits, not rotor-level feasibility;
- the controller uses pymavlink directly; ROS 2 and MAVSDK are not dependencies;
- GPS/IMU cannot identify obstacles, landing-site clearance, or arbitrary terrain;
- four-second GNSS recovery has bounded SITL evidence, while combined gusts and a
  twelve-second outage produced an unsafe touchdown;
- no physical airframe has been qualified.

The existing five-stop SITL result is useful regression evidence, but it does not
establish obstacle avoidance or physical-flight readiness.

## 3. Precise interpretation of the two user flows

### 3.1 Drawn-path flow

A screen stroke is not a flyable trajectory. It lacks at least a coordinate
frame, scale, altitude interpretation, timing, yaw, clearance, and endpoint
state. The UI must collect or visibly default each of these:

1. **Drawing plane:** map/horizontal plane, vertical plane, or true 3-D editor.
2. **Frame and origin:** `LOCAL_NED` relative to launch for the first release;
   geodetic input must be converted once with a recorded datum and transform.
3. **Scale:** metres per screen/map unit.
4. **Altitude:** fixed altitude, explicit altitude profile, terrain-relative
   height using an approved terrain source, or 3-D stroke.
5. **Stroke fidelity:** a maximum deviation tube in metres, plus optional hard
   points that must be interpolated.
6. **Start/end state:** position, velocity, acceleration, yaw, and whether each
   end is a stop, pass-through, or periodic closure.
7. **Yaw policy:** fixed, tangent-following, look-at target, explicit profile, or
   unconstrained.
8. **Speed policy:** target cruise speed or fastest comfortable speed, always
   capped by the verified vehicle envelope.
9. **Environment:** geofence, obstacle map version, vehicle radius, localization
   uncertainty, and tracking margin.

Processing is deterministic:

- reject invalid coordinates and duplicate/non-finite samples;
- remove input-device jitter with a declared filter;
- arc-length resample;
- simplify only within a recorded error bound;
- fit a clamped quintic B-spline inside the fidelity tube;
- preserve hard points;
- classify corners or cusps that require a stop, a geometric rounding tolerance,
  or rejection;
- solve timing and then run the independent verifier.

The UI must overlay both the original stroke and accepted trajectory. It must
show where and why the planner deviated.

### 3.2 Structured-interview flow

Ask only questions that remain unresolved and distinguish preferences from hard
constraints:

1. Where should the aircraft start and finish, and in which coordinate frame?
2. Is the route specified by waypoints, a shape, a target region, or a survey
   pattern?
3. What altitude or altitude band is required? Is it relative to launch, mean sea
   level, or terrain?
4. Which points are mandatory, and how far may the route deviate elsewhere?
5. Is velocity a target, a maximum, or a required value at a particular point?
6. Should the aircraft stop at points, pass through them, or fly a periodic loop?
7. What yaw behavior is desired? Users normally specify yaw, not roll and pitch.
8. Is arrival time a maximum, exact value, range, or preference?
9. What payload, all-up mass, battery state/health, and vehicle configuration are
   in use?
10. What map, geofence, terrain, wind, visibility, GNSS, and communications
    assumptions apply?
11. What should happen on loss of position, command link, RC, battery margin, or
    obstacle-map validity?
12. Which operator approval and legal operating basis apply?

Roll and pitch should not normally be direct mission inputs. For a multicopter,
they are states required to produce acceleration and are constrained by tilt,
thrust, body-rate, and control-authority limits. Expose them only in an expert
maneuver mode that uses a full vehicle model and a separately qualified envelope.

### 3.3 Closed curves and circles

A circle at nonzero constant speed has no discrete final turn. Its velocity
direction changes continuously and requires centripetal acceleration:

```text
a_c = v^2 / r
```

The planner must distinguish:

- **periodic loop:** position, velocity, acceleration, jerk, yaw, and yaw rate
  match at the seam;
- **one lap with a stop:** a braking segment reaches zero velocity at the end;
- **one lap then depart:** the departure tangent and state are explicit.

For a level circle with available horizontal acceleration `a_margin`, speed is
bounded by `sqrt(a_margin * r)`. `a_margin` must reserve control authority for
wind rejection and tracking; using the aircraft's absolute acceleration maximum
for nominal centripetal acceleration is not acceptable.

The baseline implements the stopped lap, including repeats of that lap. The
time law is stationary through jerk at the seam, so velocity, acceleration, and
jerk match there because they are all zero.

A circle interview can also name an exit. The ring is written once per
requested lap as a single open polyline. Each return to the launch point is an
exact fly-through, and the exit is appended. The time law is stationary only at
the launch and at the end of the exit, which is where the runner lands. Speed
on the seam is the speed of that certified schedule. It is not a separate
cruise command, and the path does not end in motion.

A periodic seam whose velocity stays nonzero, and an `open_depart` ending, are
not implemented. The current schedule can represent a rest-to-rest parameter
law only.

## 4. Requirements baseline

Each implementation and verification artifact must reference requirement IDs.

### Intent and contracts

- **INT-001:** One versioned schema shall represent both drawn and prompted
  missions.
- **INT-002:** Every spatial input shall name its frame, units, origin, and
  altitude datum.
- **INT-003:** Every input shall be classified as hard, soft with weight, or
  informational.
- **INT-004:** Every default shall be displayed and serialized; no hidden
  operational defaults are permitted.
- **INT-005:** The system shall reject NaN, infinity, unknown fields, duplicate
  identifiers, invalid transforms, and contradictory constraints.
- **INT-006:** A 2-D drawing shall not imply altitude without an explicit,
  visible interpretation.
- **INT-007:** The accepted mission shall record a maximum geometric deviation
  from the user input.

### Vehicle and environment

- **VEH-001:** A versioned vehicle model shall include mass and inertia, thrust
  curve, rotor limits, arm geometry, drag model or conservative bound, battery
  limits, tilt/body-rate limits, and controller envelope.
- **VEH-002:** Planner limits shall be no larger than independently approved
  controller and PX4 limits.
- **ENV-001:** The mission shall bind to a geofence and, when collision avoidance
  is claimed, a versioned obstacle/terrain map.
- **ENV-002:** Clearance shall include vehicle radius, map error, localization
  uncertainty, tracking error bound, and a configured residual margin.
- **ENV-003:** A missing or stale obstacle map shall block autonomous physical
  launch for any mission that relies on obstacle clearance.
- **ENV-004:** Wind and GNSS assumptions shall be part of the operational design
  domain (ODD), not informal test notes.

### Planning and feasibility

- **PLN-001:** The planner shall produce continuous position, velocity, and
  acceleration; normal missions shall also have continuous jerk.
- **PLN-002:** Yaw and yaw derivatives shall be planned independently of path
  tangent unless tangent-following is selected.
- **PLN-003:** Closed periodic paths shall satisfy seam continuity for all
  derivatives consumed by the controller/verifier.
- **PLN-004:** The planner shall enforce velocity, acceleration, jerk, yaw, tilt,
  thrust, body-rate, rotor, geofence, clearance, and mission-duration bounds
  applicable to its model.
- **PLN-005:** Optimization failure, timeout, infeasibility, or non-finite output
  shall fail closed.
- **PLN-006:** All stochastic search shall record seed and budget. Acceptance
  shall remain deterministic.
- **PLN-007:** A requested duration shall be rejected if infeasible; the minimum
  feasible duration and limiting constraints shall be reported.
- **PLN-008:** The objective and priority order shall be serialized with the
  mission.

### Independent acceptance

- **VER-001:** The acceptance verifier shall not share optimizer success flags or
  penalty values as evidence of feasibility.
- **VER-002:** It shall check the continuous trajectory using analytic,
  Bernstein/interval, or adaptively converged bounds where available. Fixed
  sparse sampling alone is insufficient.
- **VER-003:** It shall check coordinate transforms, path bounds, derivative
  bounds, flatness-derived vehicle states/inputs, clearance, geofence, timing,
  numeric conditioning, and endpoint/periodic conditions.
- **VER-004:** It shall emit a signed or hashed, immutable `PlanArtifact` with
  input, configuration, software, map, model, trajectory, and report digests.
- **VER-005:** The runtime shall revalidate the artifact against live vehicle
  state, battery, PX4 parameters, estimator health, map/geofence version, weather
  limits, and expiry before arming.

### Simulation and execution

- **SIM-001:** Every candidate shall pass deterministic unit/property tests and
  nominal SITL before operator presentation.
- **SIM-002:** Physical-flight candidates shall pass a scenario manifest covering
  boundary values, wind, sensor error, timing jitter, packet loss, process loss,
  estimator degradation, battery actions, geofence, and Offboard loss.
- **SIM-003:** Monte Carlo campaigns shall use predeclared distributions, seeds,
  sample count, and acceptance thresholds. A statistical pass is not a proof of
  untested conditions.
- **SIM-004:** Simulation shall produce actual-versus-reference plots, minimum
  clearance, constraint margins, tracking distribution, mode timeline, failsafe
  outcome, touchdown quality, disarm, and post-disarm output.
- **EXE-001:** The operator shall approve the exact artifact digest; any mission,
  software, model, map, parameter, or environment change invalidates approval.
- **EXE-002:** PX4 shall receive Offboard proof-of-life above its documented 2 Hz
  threshold; this project should retain its 20 Hz stream and stricter watchdog.
- **EXE-003:** A separate, tested operator intervention path shall provide hold,
  return, land, and emergency action appropriate to the aircraft and site.
- **EXE-004:** No simulator truth or post-processing signal shall enter the
  physical control loop.

### Human factors and records

- **HMI-001:** The review screen shall show route, altitude, yaw, speed, time,
  uncertainty/clearance tube, active constraints, assumptions, and abort actions.
- **HMI-002:** “Simulation passed” shall not be displayed as “safe to fly.”
- **HMI-003:** Warnings shall not be dismissible when they represent failed hard
  constraints.
- **LOG-001:** Inputs, outputs, decisions, telemetry, simulator configuration,
  random seeds, software/configuration hashes, and operator approval shall be
  retained as one auditable run.

## 5. Planner trade study

Scores are 1 (poor) through 5 (strong). Weighted totals are comparative design
guidance, not safety probabilities.

| Method | Assurance 25% | Determinism 15% | Stroke fidelity 15% | Dynamics 15% | Compute 10% | Timing 10% | Maturity 10% | Weighted |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| Existing stop-to-stop polynomial | 3 | 5 | 2 | 2 | 5 | 2 | 5 | 3.35 |
| Minimum-snap polynomial | 3 | 5 | 4 | 3 | 5 | 2 | 5 | 3.75 |
| B-spline/Bézier path | 4 | 5 | 5 | 3 | 5 | 2 | 4 | 4.05 |
| TOPP-RA retiming | 4 | 5 | 5 | 4 | 4 | 5 | 4 | 4.40 |
| MINCO/GCOPTER | 5 | 4 | 4 | 5 | 4 | 4 | 4 | 4.40 |
| Generic direct optimal control | 5 | 4 | 3 | 5 | 2 | 5 | 4 | 4.15 |
| Online NMPC/MPCC | 5 | 4 | 3 | 5 | 3 | 5 | 4 | 4.25 |
| Sampling-based planner | 4 | 3 | 2 | 4 | 1 | 3 | 5 | 3.25 |
| Learned planner/policy | 2 | 1 | 3 | 3 | 5 | 4 | 2 | 2.65 |

These rows solve different subproblems:

- **Clamped quintic B-spline:** canonical drawn-path representation. Local
  support makes edits predictable; convex-hull/Bernstein forms support
  conservative continuous bounds.
- **TOPP-RA:** preferred timing engine for a fixed accepted path when constraints
  fit its path-parameterized form. It preserves geometry and computes a
  time-optimal or specified-duration parameterization. It does not make an unsafe
  path safe and is not, by itself, rotor-level feasibility proof.
- **MINCO/GCOPTER:** later primary joint spatial-temporal optimizer for corridor
  missions and fuller multicopter dynamics. It is nonconvex and uses penalties;
  independent verification remains mandatory.
- **Minimum snap:** deterministic fallback/baseline for uncongested waypoint
  missions, with conservative global time scaling.
- **Graph/sampling search:** topology discovery in clutter, before deterministic
  smoothing. Seeded sampling is acceptable as a proposal generator, not as the
  final safety authority.
- **NMPC/MPCC:** possible future tracker for aggressive flight or moving
  constraints. It does not replace global route planning and requires a
  deadline-enforced deterministic fallback.
- **Machine learning:** acceptable for stroke classification, preference
  suggestions, perception proposals, warm starts, or bounded residual dynamics.
  It shall not certify clearance, feasibility, arming, or final acceptance.

### Initial implementation choice

The first release should not begin with GCOPTER, ROS 2, or a learned model.
Extend the current Python pipeline with:

1. a strict `MissionIntent` schema;
2. deterministic polyline cleanup and clamped spline fitting;
3. curvature-aware conservative retiming;
4. a separate continuous-trajectory verifier;
5. conversion to the existing endpoint-inclusive trajectory representation;
6. the existing controller and evidence system.

This delivers the user-facing capability with the smallest change to validated
execution code. Add a compiled optimizer service only after the contracts and
verifier are stable.

## 6. Proposed component boundaries

```text
Drawing UI / interview
        |
        v
MissionIntent v1  -------- VehicleModel + EnvironmentSnapshot
        |                              |
        v                              v
Intent normalizer -> geometric path -> corridor/topology (when mapped)
                             |
                             v
                 timing or joint optimizer
                             |
                             v
                    CandidateTrajectory
                             |
                             v
                 independent acceptance verifier
                             |
                             v
                       PlanArtifact
                      /            \
             Gazebo campaign    operator review
                      \            /
                       approval hash
                             |
                   live pre-arm revalidation
                             |
                 existing PX4 Offboard runner
                             |
                 flight evidence and audit
```

Suggested repository layout:

```text
mission_planner/
  contracts.py              # versioned intent/model/environment/artifact schemas
  normalize.py              # frames, units, stroke cleanup, hard/soft constraints
  geometry.py               # arc length, splines, seam and curvature handling
  retime.py                 # baseline conservative time law, later TOPP-RA adapter
  optimize.py               # later MINCO/GCOPTER service adapter
  verify.py                 # independent continuous and dynamic checks
  explain.py                # active constraints and deviation report
simulation/
  scenario_manifest.py      # fixed scenario and Monte Carlo definitions
  campaign.py               # Gazebo orchestration and evidence collation
ui/
  ...                       # separate process; never owns arming authority
```

Do not replace the current strict waypoint schema. Treat it as a supported
`MissionIntent` subtype and regression oracle.

### Core data products

`MissionIntent`:

- schema/version and creation metadata;
- frame/origin/units/datum;
- source stroke or waypoints;
- hard points, deviation tube, and endpoint/closure semantics;
- altitude, yaw, speed, time, and objective policies;
- vehicle-model, environment, geofence, and ODD references;
- abort/failsafe policy references.

`CandidateTrajectory`:

- piecewise curve representation and valid parameter interval;
- time law;
- position/yaw derivatives through snap/yaw acceleration as required;
- optimizer status, initialization, seed, versions, tolerances, and objective;
- no claim of acceptance.

`VerificationReport`:

- pass/fail for every requirement;
- global extrema and margins with where/when they occur;
- deviation from user geometry;
- active constraints and infeasibility explanation;
- numerical conditioning and bound method;
- verifier version and configuration.

`PlanArtifact`:

- immutable references to all three products;
- sampled controller trajectory;
- simulation scenario manifest and results;
- all content digests;
- approval identity/time/expiry;
- explicit `SITL_ONLY`, `HITL_CANDIDATE`, or `FLIGHT_CANDIDATE` release class.

## 7. Optimization formulation

Let the accepted geometric path be `p(s)` and yaw be `psi(s)`, for
`s in [0, 1]`. Timing supplies monotonic `s(t)`.

Hard constraints include:

- start/end or periodic state;
- safe corridor and geofence;
- altitude band;
- speed, acceleration, jerk, and optional snap;
- yaw rate and acceleration;
- total thrust, minimum thrust, tilt, body rate, and rotor authority;
- timing feasibility and battery reserve.

Use a lexicographic solve rather than a single opaque weighted sum:

1. find a feasible solution with prescribed margins;
2. keep deviation from the user's path inside the declared tube;
3. maximize minimum dynamic/clearance margin or minimize control effort;
4. minimize duration.

A weighted objective may be used within a tier, but all weights and normalized
units must be recorded. Hard constraints must never be represented only by a
finite soft penalty in the acceptance decision.

### 7.1 Spending the deviation tube

Tiers 2 and 3 genuinely conflict, and the conflict is not an implementation
detail. A path that hugs the stroke exactly reproduces every corner the user
drew, including corners whose radius is far below what the aircraft can fly at
any useful speed; a path that cuts those corners is less faithful but carries a
much larger dynamic margin at the same speed. Minimizing deviation therefore
maximizes cost, and the user's `maximum_deviation_m` is the only statement of
how much fidelity they are willing to trade.

The planner resolves this with one explicit, serialized number:
`deviation_utilization`, the share of the declared tube that smoothing is
allowed to consume. The baseline walks a fixed geometric ladder of control
spacings from coarse to fine and keeps the first spacing whose independently
measured symmetric deviation bound fits that share. Coarser spacing widens
corners, which lowers curvature, which lowers the acceleration and jerk
demanded at a given speed. The unused remainder of the tube is headroom for the
independent verifier's own measurement and for controller discretization, so
the verifier can still reject a plan that the geometry stage accepted.

Because this is a policy choice and not a derived quantity, it is recorded in
the plan summary alongside the requested and selected spacing, the resulting
deviation bound, and the budget it was checked against.

### 7.2 Timing: the path-parameter schedule

A single global time law such as a seventh-order smootherstep is slow enough
everywhere for the worst point on the path. A mission that is mostly straight
still pays twice over: once for the peak-to-mean ratio of the ramp itself, and
again for the one tightest corner, which raises the curvature bound used for
the entire path. On the example drawn mission those two factors together cost
roughly 3.6x the duration of flying the same curve at the speed limit.

Timing is therefore represented as its own curve: `s(tau)` for
`tau = t / T in [0, 1]`, a monotone clamped quintic B-spline. The alternative
considered was the TOPP formulation, which works in the `(s, s_dot^2)` plane.
That representation was rejected for this baseline because a polynomial profile
in `W = s_dot^2` can only vanish to integer order at the endpoints, which
forces a choice between an acceleration step at `t = 0` and an infinite
duration. Keeping the schedule in normalized time instead gives exact endpoint
stationarity through jerk, needs no quadrature or profile inversion at sample
time, makes uniform-period sampling trivial, and leaves the schedule as plain
spline data that the independent verifier can recompute from exactly.

Monotonicity and endpoint stationarity are both structural rather than
numerical. Control values that never decrease force `s(tau)` to never decrease,
because on each span the value is a convex combination of that span's local
control values; repeating the first and last control value four times forces
the schedule and its first three derivatives to vanish at the ends, because a
clamped-end derivative depends only on that many controls. Acceptance checks
these properties directly instead of trusting that the planner produced them.

Timing is split into two stages with different standing:

- The **proposal** stage chooses the schedule's shape. It builds a pointwise
  pace ceiling `g(s) = min(v_max/R1, sqrt(a_lat/R2), cbrt(j_lat/R3))` from the
  per-span path derivative bounds `R1, R2, R3`, tapers it at both ends so the
  path parameter leaves and reaches its endpoints like `t^4`, and integrates
  `ds/g(s)` to get elapsed time. The ceiling is piecewise constant, so fitting
  a schedule straight through its steps leaves third-derivative noise that
  acceptance charges heavily for; the ceiling is smoothed first by a running
  minimum over a window at least as wide as the whole blur support, followed by
  mean blurring, which guarantees the smoothed ceiling never exceeds the raw
  ceiling at the same parameter.
- The **acceptance** stage derives the duration from certified per-span
  convex-hull bounds on both the path and the schedule. Each schedule span is
  paired only with the path bounds over the parameter range that span can
  reach, so a straight stretch is not charged for a corner it never visits.
  Inverting each bound for the duration that makes it meet its limit turns the
  span bounds into durations, and the span needing the longest sets `T`.

The split is what makes the heuristics safe. A poor proposal yields a slow
mission, never an unsafe one, because nothing the proposal stage does enters
the duration except through bounds the acceptance stage recomputes itself.

Because the duration is the single worst span requirement, it falls only when
the binding spans are slowed and the slack spans take up the difference.
Successive passes therefore scale the ceiling down wherever acceptance charges
more than it does elsewhere, which drives the span requirements towards each
other. The planner searches a fixed, declared grid of schedule resolutions,
smoothing widths, and splits of acceleration between changing speed and
turning, scoring every candidate by the same certified requirement acceptance
uses, so widening the grid can only find a faster mission and never a less
conservative one. Planning cost is explicitly not a factor in this trade.

### 7.3 Vehicle and energy validation

The vehicle verifier should recover attitude, thrust, and body-rate demands from
the flat outputs `(x, y, z, yaw)` and derivatives. Rotor allocation then checks
whether demanded wrench remains inside a conservatively identified actuator
set. Include uncertainty margins for mass, thrust calibration, voltage sag,
drag, latency, state-estimation error, and tracking error.

Energy validation should initially use a conservative reserve policy measured
from flight data, not an unjustified high-fidelity battery model. The mission
must fit the lower of:

- duration/distance envelope demonstrated for the configuration; and
- energy prediction minus return/land and degradation reserve.

### 7.4 What the baseline executes

The executable baseline now covers the following, all still marked `SITL_ONLY`:

- **Tangent yaw.** The commanded heading is a clamped quintic of the normalized
  path parameter. It is accepted only when a Lipschitz remainder keeps it within
  two degrees of the unwrapped horizontal tangent, sampled densely enough that
  the heading cannot jump by half a turn between samples. Yaw rate, acceleration,
  and jerk are certified from that profile's own derivative bounds and the
  schedule, then scaled by the same limit reserve as translation. The chain rule
  for yaw uses derivatives with respect to the normalized parameter. Position
  derivatives of a repeated lap are scaled by the lap count; yaw is not scaled
  again, because the profile already covers every lap.
- **Repeated laps.** `loop_count` from 1 to 20 is legal only on a periodic
  curve. The schedule still outputs a normalized parameter in `[0, 1]`. The
  flown parameter is that value times the lap count, and the path derivative
  bounds used for timing are unrolled by `loops`, `loops²`, and `loops³`.
- **Interior hard points.** Each interior hard point splits the stroke into
  rest-to-rest open pieces. Endpoints are already stops. A periodic path cannot
  contain an interior hard point. An exact or maximum duration is probed per
  piece at minimum time and then shared in proportion to those minima. Each
  piece is verified on its own curve; the published report keeps the worst
  margin of each requirement.
- **Tilt, thrust, and body rate.** For an acceleration no larger than `A < g`,
  the tilt is at most `asin(A / g)`, thrust lies in `m(g - A)` to `m(g + A)`,
  and the body rate is at most `J / (g - A)` plus the yaw-rate bound. If
  translation would finish sooner than these bounds allow, duration is
  lengthened until they fit and the limiting constraint becomes tilt, thrust,
  or body rate. Rotor allocation is not checked.
- **Interview.** Waypoints, a circle through the launch point, a survey
  lawnmower, and a corridor on a bound map become an ordinary `MissionIntent`.
  Roll and pitch are not inputs. A circle may name an exit: the requested laps
  are one open polyline, each return to the launch seam is an exact
  fly-through, and the path stops at the end of the exit. Seam speed is the
  certified schedule. Without an exit the circle stays periodic and stops at
  the seam.
- **Review.** A self-contained page shows the route, speed, duration, binding
  constraint, and the limitation list. It contains the phrase "not a clearance
  to fly" and has no control that dismisses a hard failure.

The controller adds the planned yaw change to the heading measured at launch.
Tangent mode therefore matches the path only when the aircraft starts pointed
along the initial tangent. A circle exit still ends at rest, so the runner can
land from the final sample. Open-departure, which would end in motion, is
refused. Unconstrained yaw is refused.

- **Look-at yaw.** The heading is the horizontal bearing to one fixed point.
  The same two-degree certificate applies. The path is refused when it can
  reach that point, because the bearing is then undefined.
- **Exact fly-through points.** A point marked exact is interpolated by the
  curve and is not a stop. A point marked hard is still a full stop. A point
  cannot be both.
- **Occupancy map.** A hashed grid can be bound to the environment. Clearance
  is a sampled distance to occupied cells minus a Lipschitz remainder, after
  localization inflation. A corridor interview builds a deterministic
  4-connected path on that inflated grid. No map is inferred from GPS or IMU.
- **Rotor yaw torque.** The x500 SITL model uses the Gazebo base-link yaw
  inertia of 0.04 kg m^2 and the v1.17 airframe torque coefficient of 0.05 m,
  with two rotors of each sign. Duration stretches when that torque cannot
  support the certified yaw acceleration at the thrust bounds. This is not an
  identified aircraft and it does not reproduce PX4's control allocator.
- **SITL acknowledgement.** Execution of a drawn-path summary requires an
  unexpired review record bound to the trajectory and report digests. The
  record says it is not a clearance to fly. A recorded track can be compared
  with the plan offline. The repository does not start Gazebo.

## 8. Simulation, verification, and release gates

### Gate A — pure software

- schema and adversarial parser tests;
- unit/property tests for frames, spline continuity, closed-loop seams, extrema,
  retiming, and infeasibility;
- independent verifier mutation tests: every constraint must be shown to fail;
- determinism across repeated runs with pinned versions;
- fuzzing for malformed/non-finite/ill-conditioned input;
- baseline waypoint missions must byte-match or differ only by an approved
  contract migration.

Exit: all requirements through `VER-*` traced to automated evidence.

### Gate B — nominal SITL

- open paths, circles/closed loops, stops, pass-through points, vertical paths,
  yaw modes, and requested-duration cases;
- compare planned extrema with simulator truth and PX4 estimates;
- verify start, all hard points, closure, return, landing, disarm, and
  post-disarm output;
- verify that failed simulation cannot produce a launchable artifact.

Exit: predeclared tracking, margin, and touchdown thresholds pass for each
mission class.

### Gate C — robustness and faults

- winds across the declared ODD, gusts, model error, payload/mass boundaries;
- sensor noise/bias, GNSS stale/wrong/lost, estimator reset and origin change;
- CPU load, optimizer/controller process loss, control stall, packet
  delay/drop/reorder, and clock discontinuity;
- low battery, Offboard loss, data-link/RC loss, geofence behavior, rejected
  arming, and native landing;
- obstacle/map perturbations when an obstacle-aware sensor and map exist;
- repeated seeded Monte Carlo plus deterministic boundary/adversarial cases.

Exit: no unexplained failures; every expected failure reaches the specified
bounded fallback and evidence outcome. The known twelve-second gust/GNSS case
remains outside the ODD unless a separately validated design closes it.

### Gate D — hardware integration

1. lock intended airframe, autopilot, companion computer, sensors, radio, props,
   payload, mass/CG, firmware, and parameters;
2. identify thrust/rotor, inertia, drag, timing, sensor, battery, and thermal
   envelopes;
3. run production-target timing/load tests and PX4 hardware simulation;
4. run propeller-off bench tests, power transitions, watchdogs, command
   authority, Remote ID, and operator override;
5. perform tethered/contained checks where appropriate;
6. expand outdoor envelope incrementally in a sterile area, benign conditions,
   low altitude and speed, one new variable at a time;
7. review logs before every expansion.

PX4 documents that HITL and SITL bypass different parts of the system. Neither
substitutes for the other or for physical tests.

### Gate E — operational release

Release requires:

- a frozen configuration manifest and approved ODD;
- legal/airspace/site review by the responsible operator;
- requirement-to-test traceability with passing evidence;
- demonstrated failsafes and independent operator intervention;
- flight manual, maintenance/configuration control, and checklists;
- explicit residual-risk acceptance by the responsible organization and RPIC;
- a first-flight plan strictly inside the demonstrated envelope.

## 9. Operational and regulatory baseline

The legal baseline depends on jurisdiction and operation. For a civil U.S.
small-UAS operation under 14 CFR Part 107, the operator—not the planner—must
establish at least:

- registration/marking, Remote ID compliance, and remote-pilot authority;
- visual line of sight unless a valid waiver or other authorization applies;
- airspace authorization and compliance with TFRs/restricted areas;
- the applicable 87-knot, 400-foot, visibility, and cloud-clearance limits;
- preflight environment, aircraft, control-link, power, payload, crew, and
  contingency checks;
- rules for night, people, and moving vehicles;
- an RPIC who remains the final authority and can intervene.

These are not all engineering requirements and this is not legal advice. A
different country, public-aircraft operation, BVLOS operation, heavier aircraft,
multiple-aircraft operation, or operation over people changes the basis.

Remote ID and legal checks should be represented as release checklist evidence,
not hard-coded as a universal planner rule.

## 10. Delivery sequence

### Milestone 0 — freeze scope and evidence

Deliver:

- `MissionIntent`, `VehicleModel`, `EnvironmentSnapshot`, `VerificationReport`,
  and `PlanArtifact` JSON Schemas;
- requirements-to-test matrix;
- pinned baseline outputs for the current five-stop mission;
- documented ODD and release classes.

Decisions required before completion are listed in section 11.

### Milestone 1 — drawn-path baseline, no obstacles

Deliver:

- CLI/library import for a 2-D horizontal or 3-D polyline;
- deterministic cleanup, arc-length resampling, clamped spline fit, hard points,
  maximum-deviation verification, and circle/periodic semantics;
- conservative speed/acceleration/jerk/curvature retiming;
- independent continuous verifier;
- conversion to existing controller trajectory points;
- unit/property tests and nominal Gazebo scenarios.

The repository now has the pure-software part of this milestone: smoothing,
deviation bounds, certified timing, tangent yaw, stopped laps, hard-point
stops, flatness bounds, and the controller CSV. Nominal Gazebo scenarios driven
by these artifacts are not in the repository. Physical launch remains
prohibited. This milestone assumes a known-clear test volume and does not claim
collision avoidance.

### Milestone 2 — interview/API and operator review

Deliver:

- structured mission builder using the same schema;
- infeasibility and active-constraint explanations;
- route/reference overlay, altitude/yaw/speed/time plots, margins, and simulation
  result review;
- artifact approval hash and expiry;
- pre-arm artifact/live-state binding.

The interview compiler, structured infeasibility report, and static review page
are in the repository. Artifact approval hash, expiry, and a live operator UI
are not.

### Milestone 3 — mapped environment

Deliver:

- versioned map/terrain ingestion;
- inflated occupancy representation and deterministic corridor construction;
- graph-based topology search;
- continuous clearance verification;
- obstacle, stale-map, localization-error, and map-change scenario suite.

This requires an approved sensing/mapping architecture; GPS/IMU alone is
insufficient.

### Milestone 4 — higher-fidelity optimization

Deliver:

- TOPP-RA adapter for fixed-path time parameterization;
- MINCO/GCOPTER-style service for joint corridor and timing optimization;
- independently implemented flatness/rotor feasibility checks;
- fallback to the verified baseline planner;
- differential testing between methods.

### Milestone 5 — aircraft qualification

Deliver:

- identified vehicle model and conservative uncertainty bounds;
- hardware simulation/bench evidence;
- physical test cards, abort criteria, checklists, and incremental envelope
  evidence;
- configuration-controlled flight-candidate release.

Online replanning, moving obstacles, and aggressive NMPC are later, separate
assurance projects. They are not prerequisites for a robust offline planner.

## 11. Decisions that cannot be invented by software

Implementation may start on schemas and pure geometry, but physical-flight
claims are blocked until the owner supplies:

1. intended aircraft, autopilot hardware, companion computer, sensors, payload,
   all-up mass, center-of-gravity range, propellers, and battery;
2. operating jurisdiction and intended basis (for example, U.S. Part 107 VLOS);
3. indoor/outdoor use, maximum radius/altitude/speed, wind, temperature,
   precipitation, lighting, GNSS degradation, and communications ODD;
4. obstacle sensor/map source, update guarantees, and required clearance;
5. whether drawings are map-horizontal by default and which altitude datum is
   authoritative;
6. required path-fidelity tolerance and whether the planner may insert stops;
7. normal yaw policy and whether expert attitude maneuvers are in scope;
8. return/land/terminate policy for each loss mode and available manual override;
9. acceptable probability/confidence targets and test sample rationale;
10. evidence retention, operator roles, and who may approve a flight artifact.

Until those are fixed, use conservative x500 SITL defaults only and label every
artifact `SITL_ONLY`.

## 12. Sources and rationale

Primary and official sources:

1. D. Mellinger and V. Kumar, “Minimum Snap Trajectory Generation and Control
   for Quadrotors,” ICRA 2011. Establishes `(x, y, z, yaw)` flat outputs and
   smooth polynomial trajectory generation.
   <https://doi.org/10.1109/ICRA.2011.5980409>
2. H. Pham and Q.-C. Pham, “A New Approach to Time-Optimal Path
   Parameterization based on Reachability Analysis,” IEEE TRO 2018.
   <https://doi.org/10.1109/TRO.2018.2819195>
3. TOPP-RA documentation, including time-optimal and specified-duration path
   parameterization. <https://hungpham2511.github.io/toppra/>
4. Z. Wang et al., “Geometrically Constrained Trajectory Optimization for
   Multicopters,” IEEE TRO 2022, and official GCOPTER implementation.
   <https://doi.org/10.1109/TRO.2022.3160022> and
   <https://github.com/ZJU-FAST-Lab/GCOPTER>
5. S. Liu et al., “Planning Dynamically Feasible Trajectories for Quadrotors
   Using Safe Flight Corridors in 3-D Complex Environments,” IEEE RA-L 2017.
   <https://doi.org/10.1109/LRA.2017.2663526>
6. PX4 Offboard mode and Offboard-loss behavior.
   <https://docs.px4.io/main/en/flight_modes/offboard>
7. PX4 safety/failsafe configuration and preflight checks.
   <https://docs.px4.io/main/en/config/safety> and
   <https://docs.px4.io/main/en/flying/pre_flight_checks>
8. PX4 simulation and HITL documentation.
   <https://docs.px4.io/v1.17/en/simulation/> and
   <https://docs.px4.io/main/en/simulation/hitl>
9. 14 CFR Part 107, especially sections 107.31, 107.49, and 107.51.
   <https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-107>
10. 14 CFR Part 89 and FAA Remote ID guidance.
    <https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-89> and
    <https://www.faa.gov/uas/getting_started/remote_id>
11. FAA AC 107-2A, operational guidance for Part 107.
    <https://www.faa.gov/documentLibrary/media/Advisory_Circular/AC_107-2A_Editorial_Update.pdf>
12. NASA Software Safety Guidebook, NASA-GB-8719.13.
    <https://s3vi.ndc.nasa.gov/ssri-kb/static/resources/nasa-gb-8719.13.pdf>

Standards such as ASTM F3178 (operational risk assessment), F2908 (flight
manual), F3366 (maintenance manual), and F3411 (Remote ID performance) may be
useful in a formal assurance program. Their full requirements are generally
paywalled, and they are not automatically binding merely because a project cites
them. Applicability must be determined for the specific approval and operation.

## 13. Definition of success

The project is successful when it can:

1. explain exactly how a user's intent was interpreted;
2. either produce a trajectory with independently checked positive margins or
   explain why no feasible trajectory exists;
3. reproduce the same accepted artifact from the same inputs and versions;
4. demonstrate required nominal and failure behavior in simulation;
5. prevent a simulation-only, stale, changed, or failed artifact from arming a
   real aircraft;
6. retain enough evidence for an independent reviewer to reproduce the decision;
7. move to physical flight only through a configuration-controlled,
   operator-approved test program.

No finite amount of simulation proves universal safety. The defensible claim is
limited: the exact configuration passed stated requirements under a documented
ODD and test basis, with known residual risks.
