# Physical-release software verification — 19 October 2026

The reusable physical-release CLI, read-only live checks, one-use launch and
runtime guard are implemented. This record is SITL_ONLY; no aircraft has been
physically qualified or flown.

- Final local regression: **366 tests passed**; Ruff passed for controller,
  mission_planner, scripts, tests and analysis.
- Final-source owned Gazebo nominal run: exact retained profile/CSV, release
  hooks and battery/site/time guard, sampled assessment, native LAND/disarm
  and continuously zero commanded motor outputs passed.
- Final-source operator abort at trajectory clock >=3 s: aborted route, native
  LAND cleanup, independent truth/contact checks, final disarm and continuously
  zero outputs passed. The full-route nominal result correctly remains false.
- Physical preparation of the real passing simulation artifact with an
  incomplete qualification request was refused before any vehicle connection.
- Sixteen synthetic unit tests cover tampering (including raw ULog), identity
  and parameter mismatch, expiry/replay, insufficient abort energy, missing
  human evidence, telemetry/site/energy guards and motor mappings. They are
  software fixtures, not physical evidence.

`qualification.json` preserves exact source/evidence hashes, complete runner
configuration, results, all assessment checks and retained machine paths. Raw
CSV/ULog remain at those paths. Earlier diagnostic records are retained separately and are not substituted
for these final-source records.

The simulator does not exercise a real RC transmitter or identify a physical
aircraft. UID2/firmware/parameter mismatch branches have deterministic protocol
tests; actual board, bench, RC, power, dynamics and supervised flight evidence
are still required for each configuration. See
[the physical-release guide](../../../../docs/physical_flight_release.md).
