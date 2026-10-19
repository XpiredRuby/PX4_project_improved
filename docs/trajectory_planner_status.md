# Trajectory planner: plan, progress, and remaining work

Status date: 19 October 2026. This file is a progress record for the drawn-path
and interview planner in this repository. It is not a flight authorization.
Every planner artifact is `SITL_ONLY` and is not a clearance to fly.

The research plan and the requirement matrix live in
[`trajectory_planner_architecture.md`](trajectory_planner_architecture.md) and
[`trajectory_planner_traceability.md`](trajectory_planner_traceability.md).
This file says what that plan is, what the repository actually does today, and
what is still left.

Current evidence is recorded in the [research review](trajectory_planner_research_review.md)
and [connected validation record](trajectory_studio_connected_validation.md).
The [local studio](trajectory_studio_quickstart.md) supports both inputs,
independent verification, exact-artifact confirmation and optional local
PX4/Gazebo rehearsals. It has no physical-flight endpoint.

- Offline geometry/timing checks and adversarial contract tests pass.
- Chrome drawing/interview, refusal, download and mobile checks pass.
- Gazebo baseline, drawn path and fixed-heading circle completed lifecycle
  audits. A stricter truth check exposed a startup heading transient; the
  handoff was fixed and all three repeated nominal cases passed unchanged sampled targets.
  The connected browser rehearsal and bounded GPS-outage/control-stall
  responses passed. Broader fault/physical qualification remains open.
- Physical release is blocked on an identified aircraft, operating decisions,
  hardware qualification and a separate authenticated operator decision.

Completion is tracked by these evidence gates, not an uncalibrated percentage.

## 1. The whole plan

The product has two ways to describe a mission. Both become the same checked
trajectory.

1. **Drawn path.** The user draws a route. The planner stays inside a declared
   distance of that drawing, checks the physics, shows the plan, and only then
   is the route a candidate for simulation and flight.
2. **Interview.** The planner asks for the choices that matter: speed, altitude,
   yaw, time budget, and the other limits it needs. Roll and pitch are not
   questions. They are whatever attitude the acceleration requires.

Time is the last priority. The order is:

1. legal and safety limits;
2. vehicle and environment limits;
3. stay close to what the user asked for;
4. keep tracking margin and energy reserve;
5. then make the flight as short as those limits allow.

A short deadline that breaks a limit is rejected. The planner reports the
feasible duration found for its current candidate family and its binding limit;
this is not a proof of a global minimum. It does not
silently fly faster than the model allows. It also does not give the aircraft
a long idle flight when a shorter feasible trajectory exists.

A circle is not a perfect constant-speed turn. The aircraft has to start and
stop, or leave the loop and then stop. The speed on the loop is whatever the
certified schedule can sustain.

The pipeline is:

1. Turn the drawing or the interview into one versioned `MissionIntent`.
2. Fix the frame, altitude, yaw, stops, vehicle, environment, and limits before
   any timing.
3. Fit a smooth path inside the user's deviation limit.
4. Build a corridor only when a trusted obstacle map is bound to the mission.
   With no map, the plan is not an obstacle-avoidance plan.
5. Time the path with a conservative spline schedule. A heavier joint optimizer
   (TOPP-RA, then MINCO/GCOPTER) comes only after this baseline is verified on
   its own.
6. A separate checker recomputes the path, the timing, and the limits. It does
   not trust the planner's own success flag.
7. Simulate in Gazebo, including fault cases, and show the operator the path,
   the binding limit, and the evidence.
8. Require a human approval of that exact artifact before any real flight, then
   check live state again before arm.
9. Fly through the existing PX4 Offboard controller, keeping its failsafes,
   landing, and logs.

There is no neural network in the safety path. The search is deterministic.

### Delivery milestones

| Milestone | What it delivers |
|---|---|
| 0 | Freeze the contracts, the requirement matrix, the release classes, and the existing five-stop baseline. |
| 1 | Drawn-path baseline in a clear volume: smooth geometry, timing, an independent checker, controller CSV, and nominal Gazebo runs. |
| 2 | Interview, plain-language infeasibility, operator review, approval hash and expiry, and a pre-arm bind to live state. |
| 3 | A real map: versioned terrain or obstacles, an inflated corridor, continuous clearance, and map-failure tests. |
| 4 | Higher-fidelity timing (TOPP-RA, then a joint corridor optimizer), with the verified baseline kept as the fallback. |
| 5 | An identified aircraft model, bench evidence, flight-test cards, and a configuration-controlled release. |

Online replanning, moving obstacles, and aggressive onboard optimization are
later projects. They are not required for this offline planner.

### Decisions only a person can make

Physical flight stays blocked until the owner supplies the ten decisions in
section 11 of the architecture:

1. aircraft, autopilot, companion computer, sensors, payload, mass, center of
   gravity, propellers, and battery;
2. jurisdiction and operating basis;
3. indoor or outdoor operating domain, including wind, weather, light, GNSS,
   and communications;
4. obstacle sensor or map source, update rules, and clearance;
5. whether a drawing is horizontal by default, and which altitude datum is
   authoritative;
6. how close the path must stay to the drawing, and whether the planner may
   insert stops;
7. normal yaw policy, and whether expert attitude maneuvers are in scope;
8. return, land, or terminate policy for each lost link or lost position, and
   what manual override exists;
9. confidence targets and why the test sample is large enough;
10. who keeps the evidence, who operates, and who may approve a flight.

Until those exist, the model is the conservative Gazebo x500 SITL model, and
every artifact is labeled `SITL_ONLY`.

## 2. What has been done

The offline planner compiles, checks, and exports a trajectory. The last full
run of `python3 scripts/validate.py` passed **298 tests**. The work is on
pull request 3.

### Contracts and evidence

- One strict `MissionIntent` represents both a drawing and an interview.
- Vehicle, environment, verification report, and plan artifact contracts reject
  unknown fields, missing fields, and non-finite numbers.
- Canonical JSON and SHA-256 bind the pieces together. A SITL artifact cannot
  carry a flight-approval id.
- JSON Schemas for those contracts are checked into `schemas/planner/`.
- The existing five-stop waypoint mission remains the regression baseline.
- The controller CSV includes position, yaw, and the derivatives the runner
  consumes. x is north, y is east, z is down.

### Geometry

- A drawn polyline becomes a quintic B-spline.
- The fit spends a declared share of the deviation budget so corners can widen,
  and it records the spacing it actually used.
- Derivative bounds are per knot span. Arc length is measured on the smoothed
  curve.
- A point marked exact is flown through. A point marked hard is a full stop.
  A point cannot be both. Interior hard points split the path into
  rest-to-rest pieces.
- Closed curves match at the seam through jerk.

### Timing

- The path parameter is a monotone quintic of normalized time. It is stationary
  through jerk at both ends, so the aircraft starts and stops at rest.
- Duration comes from the worst certified speed, acceleration, and jerk on each
  span. A straight stretch is not charged for a corner it never visits.
- On the example drawn mission this cut the certified duration from 94.3 s to
  39.9 s, against a straight-line floor of about 25.9 s.
- An exact or maximum deadline below the feasible duration is rejected, with
  the minimum duration and the binding limit reported.
- The proposal search is a fixed grid. Acceptance recomputes the bounds from
  the schedule that was flown. A schedule that is not monotone, or not
  stationary through jerk, is rejected.

### Yaw

Roll and pitch are consequences of acceleration. The yaw choices are:

- **fixed:** one heading;
- **tangent:** follow the horizontal path direction;
- **look-at:** face one fixed point, refused if the path can reach that point;
- **profile:** an explicit list of unwrapped heading samples from the start of
  a piece to its stop.

Tangent, look-at, and profile headings are quintics. Each is accepted only
inside a certified two-degree tube, and yaw rate, acceleration, and jerk can
stretch the duration. A profile that is too sharp for that tube is refused. A
profile is also refused when a stop splits the path, because the samples belong
to one rest-to-rest piece. Unconstrained yaw is refused: it does not produce a
command the controller can fly.

The controller adds the planned yaw change to the heading measured at launch.
Tangent mode matches the path when the aircraft starts pointed along the
initial tangent.

### Circles, laps, lead-in, and exit

- A periodic lap starts and stops at its seam. `loop_count` from 1 to 20
  repeats that stopped lap.
- A circle interview can add an entry, an exit, or both. The laps become one
  open path. The lead-in leaves launch and meets the seam without stopping.
  Later passes of the seam are flown through. The path stops at the exit, or
  on the seam when there is no exit.
- The speed on a seam that is flown through is the certified schedule. It is
  not a separate cruise command.
- A path that is still moving at the end is refused. The current runner lands
  from the final sample, so the schedule stays rest-to-rest.

### Vehicle limits checked on the x500 SITL model

- Tilt, thrust, and body rate are bounded from the acceleration the timing
  already certified. If translation would finish sooner than those bounds
  allow, the duration is stretched.
- Yaw torque uses the Gazebo x500 yaw inertia (0.04 kg m²) and the v1.17
  paired-rotor coefficient (0.05 m), with per-rotor thrust from 0 N to 10 N.
  This is not an identified aircraft and it does not reproduce PX4's control
  allocator.
- The mission speed cannot exceed the vehicle model. A 0.9 scale is reserved
  by default.

### Maps and interviews

- A hashed occupancy grid can be bound to the environment. Clearance is a
  sampled distance to occupied cells, minus a Lipschitz remainder, after
  localization inflation. No map is inferred from GPS or IMU.
- A corridor interview searches that inflated grid with a deterministic
  4-connected path.
- Interviews compile waypoints, a circle through launch, a survey lawnmower,
  or a corridor through the same checker as a drawing.

### Review and SITL handoff

- A static review page shows the route, the speed, the duration, and the
  binding limit. It says the review is not a clearance to fly, and it has no
  control that dismisses a hard failure.
- A drawn-path execution requires an unexpired SITL acknowledgement bound to
  the trajectory and report digests. The acknowledgement uses the statement
  "not a clearance to fly".
- The runner checks the trajectory hash and the live cruise height before
  execution.
- An offline comparison can score a recorded track against the plan. That tool
  does not start Gazebo and does not report that a campaign ran.

## 3. What still has to be done

### Still software, and still not a flight release

Further offline research and software qualification, beyond the demonstrated baseline:

- a periodic seam whose velocity stays nonzero, and an `open_depart` ending
  that is still moving when the path finishes;
- TOPP-RA for fixed-path timing, then a MINCO/GCOPTER-style joint optimizer,
  with this verified planner kept as the fallback;
- an energy reserve taken from flight data, plus drag, battery, and model
  uncertainty on the vehicle contract;
- general soft weights on intent, beyond hard stops and the deviation tube;
- a fuller interval checker than the current hull and Lipschitz bounds;
- dedicated reviewed cancel/abort controls; drawing/interview, confirmation,
  simulation status, truth overlay and assessment download are implemented;
- weather, GNSS, and communications assumptions as a full operating-domain
  record, rather than the single wind limit stored today.

### Needs Gazebo and PX4 on a machine that has them

The connected PX4/Gazebo machine has now run nominal baseline, drawing and
circle missions, plus a browser-confirmed artifact. The repeated nominal
cases passed the sampled truth assessment after the startup heading fix.
A four-second GPS interruption passed its bounded HOLD/recovery assessment.
The 0.4 s control stall also passed its command-expiry/PX4 landing response
assessment; it is correctly recorded as an incomplete trajectory. Remaining broader qualification includes:

- additional mission patterns, tangent launch alignment, hard-point and
  altitude transitions, and continuous composition/dynamic checks;
- fault cases: wind, map failure, stale map, localization error, and a failed
  simulation that must not produce a launchable artifact;
- proof that a failed simulator run cannot be approved.

### Needs the owner, before any real aircraft

- the ten decisions in architecture section 11; the non-executable
  `physical_aircraft_intake.template.json` collects the initial information;
- an identified vehicle model and a check that matches that aircraft's
  allocator, not only the Gazebo x500 torque model;
- a sensor or a surveyed map, if obstacle clearance is going to be claimed
  outside a hand-written grid;
- bench evidence, flight-test cards, abort criteria, and a named approver;
- a flight-approval record that is allowed to exist only after that evidence.
  `SITL_ONLY` artifacts cannot carry one.

### Explicitly later

Online replanning, moving obstacles, and aggressive onboard optimization are
out of this offline planner. Unconstrained yaw stays refused.

## 4. How to read the current result

The repository can take a drawing or an interview, produce a checked
rest-to-rest trajectory, explain the binding limit, and refuse a plan that
breaks the x500 SITL model. It can rehearse an exactly confirmed fixed-heading artifact in local Gazebo
and show its sampled truth/evidence. It cannot authorize a real flight.
Read the connected validation record for the tested conditions and limits.
