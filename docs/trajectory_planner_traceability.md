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
| INT-007 | `PathIntent.maximum_deviation_m` | Implemented |
| VEH-001 | `VehicleModel` contains first-release mass and motion/input envelope | Partial; inertia, rotor allocation, drag, battery, and uncertainty planned |
| VEH-002 | `validate_contract_set` rejects mission speed above model limit | Partial; all controller/PX4 limits planned |
| ENV-001 | `environment_id`, geofence envelope, optional map digest | Implemented at contract boundary |
| ENV-002 | `minimum_clearance_m` | Partial; decomposed uncertainty budget planned |
| ENV-003 | Map identity/verification consistency check | Partial; physical-launch gate planned |
| ENV-004 | Wind limit serialized | Partial; full GNSS/weather ODD planned |
| PLN-001..PLN-007 | No general trajectory optimizer exists yet | Planned for Milestone 1 |
| PLN-008 | Fixed safety-first `objective_priority` | Implemented |
| VER-001..VER-003 | Verification report contract exists; independent verifier does not | Planned for Milestone 1 |
| VER-004 | Canonical hashing and tamper-evident `PlanArtifact` | Implemented for `SITL_ONLY` |
| VER-005 | Cross-contract identity, frame, envelope, report, and trajectory binding | Partial; live pre-arm binding planned |
| SIM-001 | Existing deterministic waypoint validation plus pinned baseline digest | Implemented for legacy waypoint flow |
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

The machine-readable schemas are under `schemas/planner/`. They intentionally
cover structural validation; Python contracts enforce cross-field and
cross-document invariants that JSON Schema alone does not express.
