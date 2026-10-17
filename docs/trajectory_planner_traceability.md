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
| ENV-002 | `minimum_clearance_m` | Partial; decomposed uncertainty budget planned |
| ENV-003 | Map identity/verification consistency check | Partial; physical-launch gate planned |
| ENV-004 | Wind limit serialized | Partial; full GNSS/weather ODD planned |
| PLN-001 | Quintic B-spline geometry is C4 internally; seventh-order time law is stationary through jerk | Implemented for open stop-to-stop baseline |
| PLN-002 | Geometry is independent of yaw; fixed-yaw timing is implemented | Partial; tangent/look-at/profile yaw planned |
| PLN-003 | Periodic spline position and derivatives through jerk match at the seam | Implemented for geometry |
| PLN-004 | Analytic conservative speed, acceleration, and jerk bounds with reserved limit scale | Partial; tilt/thrust/body-rate/rotor checks planned |
| PLN-005 | Unsupported modes, invalid numerics, failed fidelity, infeasible timing, and failed verification stop output | Implemented for baseline |
| PLN-006 | Baseline is deterministic and uses no stochastic search | Implemented |
| PLN-007 | Exact and maximum duration requests reject below the reported minimum feasible duration | Implemented for baseline |
| PLN-008 | Fixed safety-first `objective_priority`; the fidelity/margin trade inside the deviation tube is an explicit serialized `deviation_utilization` | Implemented |
| VER-001 | `mission_planner/verification.py` recomputes acceptance without trusting retimer status | Implemented for baseline |
| VER-002 | B-spline hull/derivative bounds and Lipschitz-bounded fidelity checks | Partial; full dynamic interval verifier planned |
| VER-003 | Independent checks bind emitted points to geometry and check environment, dynamics, timing, and endpoints | Implemented for baseline |
| VER-004 | Canonical hashing and tamper-evident `PlanArtifact` | Implemented for `SITL_ONLY` |
| VER-005 | Cross-contract identity/frame/envelope binding plus runner checks of CSV digest and live cruise height | Partial; map/weather/battery binding planned |
| SIM-001 | Existing deterministic waypoint validation, pinned baseline digest, and deterministic drawn-path compiler tests | Implemented before live SITL |
| SIM-002..SIM-004 | Existing fault harness is not yet driven by general planner artifacts | Planned |
| EXE-001 | Artifact changes invalidate its digest; flight approval is prohibited | Implemented for `SITL_ONLY` |
| EXE-002..EXE-004 | Existing controller behavior and validation evidence | Existing controller; artifact binding planned |
| HMI-001..HMI-003 | Review UI does not exist | Planned for Milestone 2 |
| LOG-001 | Artifact binds component and simulation-evidence digests | Partial; full run collation planned |

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
- reuse of the curve evaluator and convex-hull bounds by a scalar schedule.

`tests/test_planner_retiming.py`, `tests/test_planner_verification.py`, and
`tests/test_drawn_path_cli.py` cover:

- conservative derivative retiming, endpoint inclusion, stationary boundaries,
  limit margins, exact/maximum duration semantics, and tighter vehicle limits;
- explicit rejection of periodic execution and unsupported yaw policies;
- independent rejection of modified trajectory points, falsified timing/limit
  scale, and environment-bound violations;
- deterministic candidate/report/CSV output and digest sensitivity;
- fail-before-output behavior for mismatched contracts;
- compatibility with the actual controller CSV loader;
- rollback of the prior output set after an injected publication failure;
- rejection of sparse endpoint-only CSVs, altered derivatives/time laws,
  controller-origin mismatch, trajectory digest mismatch, and live cruise-height
  mismatch.

The machine-readable schemas are under `schemas/planner/`. They intentionally
cover structural validation; Python contracts enforce cross-field and
cross-document invariants that JSON Schema alone does not express.
