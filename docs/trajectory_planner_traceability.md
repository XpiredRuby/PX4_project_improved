# Trajectory planner requirements traceability

This matrix tracks the requirements in
[`trajectory_planner_architecture.md`](trajectory_planner_architecture.md).
`Implemented` means executable repository evidence exists. It does not imply
physical-flight readiness.

| Requirement | Current evidence | Status |
|---|---|---|
| INT-001 | `mission_planner/contracts.py::MissionIntent`; both UI flows target one contract | Implemented |
| INT-002 | `CoordinateFrame`; strict frame, units, origin, and datum fields | Implemented for `LOCAL_NED` |
| INT-003 | `PathPoint.hard`; motion fields distinguish target and maximum | Partial; general soft weights planned |
| INT-004 | Strict required fields and rejected unknown fields | Implemented at contract boundary |
| INT-005 | Strict parsers and `test_unknown_missing_nonfinite_and_wrong_types_fail_closed` | Implemented |
| INT-006 | Contract requires explicit 3-D points and launch datum | Implemented; UI enforcement planned |
| INT-007 | `PathIntent.maximum_deviation_m`, symmetric fidelity bounds, and the serialized `SmoothingReport` deviation budget in `mission_planner/geometry.py` | Implemented for polyline smoothing |
| VEH-001 | `VehicleModel` contains first-release mass and motion/input envelope | Partial; inertia, rotor allocation, drag, battery, and uncertainty planned |
| VEH-002 | `validate_contract_set` rejects mission speed above model limit | Partial; all controller/PX4 limits planned |
| ENV-001 | `environment_id`, geofence envelope, optional map digest | Implemented at contract boundary |
| ENV-002 | Hashed occupancy grid, localization inflation, and a Lipschitz clearance margin; a corridor interview searches the inflated grid | Partial; no sensor builds the map |
| ENV-003 | Map identity/verification consistency check | Partial; physical-launch gate planned |
| ENV-004 | Wind limit serialized | Partial; full GNSS/weather ODD planned |
| PLN-001 | Quintic B-spline geometry is C4 internally; the quintic path-parameter schedule is monotone and stationary through jerk by construction, and acceptance rechecks both | Implemented for open paths, stopped laps, and hard-point stops |
| PLN-002 | Fixed yaw holds one heading; tangent, look-at, and an explicit heading profile are certified quintics checked against yaw rate, acceleration, and jerk | Implemented; unconstrained yaw is refused |
| PLN-003 | Periodic spline position and derivatives through jerk match at the seam; a flown lap is stationary there, so the consumed derivatives match because they are zero. A circle exit is an open path: each seam is an exact fly-through and the schedule stops only at launch and at the exit | Partial; a nonzero-velocity periodic seam is not implemented |
| PLN-004 | Per-span speed, acceleration, and jerk bounds, tilt, thrust, body rate, and paired-rotor yaw torque, with reserved limit scale | Partial; the check is the Gazebo x500 model, not an identified allocator |
| PLN-005 | Unsupported modes, invalid numerics, failed fidelity, infeasible timing, and failed verification stop output | Implemented for baseline |
| PLN-006 | Baseline is deterministic; the time law searches a fixed declared grid with no stochastic component | Implemented |
| PLN-007 | Exact and maximum duration requests reject below the reported minimum feasible duration | Implemented for baseline |
| PLN-008 | Fixed safety-first `objective_priority`; the fidelity/margin trade inside the deviation tube is an explicit serialized `deviation_utilization` | Implemented |
| VER-001 | `mission_planner/verification.py` recomputes acceptance without trusting retimer status | Implemented for baseline |
| VER-002 | B-spline hull/derivative bounds and Lipschitz-bounded fidelity checks | Partial; full dynamic interval verifier planned |
| VER-003 | Independent checks bind emitted points to geometry and check environment, dynamics, timing, and endpoints | Implemented for baseline |
| VER-004 | Canonical hashing and tamper-evident `PlanArtifact` | Implemented for `SITL_ONLY` |
| VER-005 | Cross-contract identity/frame/envelope binding plus runner checks of CSV digest and live cruise height | Partial; map/weather/battery binding planned |
| SIM-001 | Existing deterministic waypoint validation, pinned baseline digest, and deterministic drawn-path compiler tests | Implemented before live SITL |
| SIM-002..SIM-004 | General artifacts now drive the isolated Gazebo harness; baseline/drawing/circle and browser-confirmed missions passed sampled checks; GPS outage passed its response audit | Partial; wider fault and physical qualification remain open |
| EXE-001 | Artifact changes invalidate its digest; flight approval is prohibited | Implemented for `SITL_ONLY` |
| EXE-002..EXE-004 | Existing controller behavior and validation evidence | Existing controller; artifact binding planned |
| HMI-001 | Local studio shows route and dynamic limits; exact confirmation starts optional SITL and returns truth/evidence | Implemented for tested fixed-heading SITL flow; reviewed abort controls remain planned |
| HMI-002 | Review text says "not a clearance to fly" and does not say "safe to fly" | Implemented for the static page |
| HMI-003 | The review page has no control that dismisses a hard failure | Implemented for the static page |
| LOG-001 | Per-run configuration, parameter readback, CSV/ULog hashes, lifecycle/ground records and sampled assessment are collated | Implemented for isolated SITL cases; physical evidence remains open |

## Executable evidence

`tests/test_planner_contracts.py` covers:

- deterministic canonical JSON and SHA-256;
- immutable parsed contracts;
- unknown, missing, non-finite, incorrectly typed, and contradictory input;
- explicit timing, speed, and periodic-closure semantics;
- cross-contract identifiers, coordinate frame, vehicle envelope, environment
  envelope, verification acceptance, and trajectory identity;
- obstacle-map identity/verification consistency;
- acceptance derived from hard checks;
- tamper detection and prohibition of approval on `SITL_ONLY` artifacts;
- byte-for-byte synchronization of generated JSON Schemas;
- the pinned five-stop source and trajectory digests.

`tests/test_planner_geometry.py` covers:

- endpoint-exact straight paths and collinearity;
- corner smoothing accepted or rejected by the declared deviation;
- periodic circle seam continuity through jerk;
- analytic derivative agreement with finite differences;
- deterministic geometry;
- unsupported hard points, waypoint mode, degenerate paths, zero-tolerance
  requests, and invalid curve queries;
- symmetric stroke/curve deviation bounds with explicit verification resolution;
- spending the declared deviation budget to widen corners, and the validation
  of the utilization share itself;
- certified bounds on the rate of tangent-heading change, including rejection of
  paths whose horizontal tangent cannot be separated from zero;
- reuse of the curve evaluator and convex-hull bounds by a scalar schedule;
- arc length measured on the smoothed curve rather than inherited from the
  stroke it replaces.

`tests/test_planner_timing.py` covers:

- a monotone schedule that is stationary through jerk and spans the whole path;
- a duration under half of what a single global time law requires on the same
  path, and above the floor of flying the curve at the speed limit;
- certified bounds that hold against densely sampled dynamics, fit the limits,
  leave the binding constraint nearly tight, and scale correctly with duration;
- per-span requirements that differ by more than a factor of two along a path
  whose straights surround one tight corner;
- detection of tampered stationarity, monotonicity, and schedule degree;
- rejection of invalid limits, search grids, and equalization counts;
- deterministic planning, and a closed path whose schedule still starts at 0 and ends at 1.

`tests/test_planner_retiming.py`, `tests/test_planner_verification.py`, and
`tests/test_drawn_path_cli.py` cover:

- conservative derivative retiming, endpoint inclusion, stationary boundaries,
  limit margins, exact/maximum duration semantics, and tighter vehicle limits;
- one stopped lap of a periodic curve, rejection of repeating an open path, and rejection of unconstrained yaw;
- independent rejection of modified trajectory points, falsified timing/limit
  scale, and environment-bound violations;
- deterministic candidate/report/CSV output and digest sensitivity;
- fail-before-output behavior for mismatched contracts;
- compatibility with the actual controller CSV loader;
- rollback of the prior output set after an injected publication failure;
- rejection of sparse endpoint-only CSVs, altered derivatives/time laws,
  controller-origin mismatch, trajectory digest mismatch, and live cruise-height
  mismatch.

`tests/test_planner_mission.py` covers:

- a tangent-yaw certificate inside two degrees, with yaw rate inside the scaled
  vehicle limit and independent acceptance of the emitted samples;
- two laps that return to the seam after two full turns of heading;
- a one-degree tilt limit that lengthens the mission and becomes the binding
  constraint;
- an interior hard point that is a stop on the published controller CSV, with a
  review page that refuses a flight clearance;
- an interview circle that starts at launch and closes;
- the tilt and thrust identities of an acceleration ball.

The machine-readable schemas are under `schemas/planner/`. They intentionally
cover structural validation; Python contracts enforce cross-field and
cross-document invariants that JSON Schema alone does not express.
