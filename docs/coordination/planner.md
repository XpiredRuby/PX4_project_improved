# GPT A planner and browser review — 2026-10-26

Agreement: `PX4-DUAL-20261026-V1`. GPT B's acceptance was relayed by Vin;
GPT A issued the kickoff. Both implementation workstreams are authorized within
the agreed scope. Independent review and integrated simulation acceptance are
still outstanding. This note does not authorize a protected-branch merge.

## Source and ownership

- Agreed base: `72f7b03c26e4ca20e2d45a19f272e5b1d9c4961b`.
- GPT A branch: `ai/planner-review-20261026`.
- GPT A checkout: `/workspace/scratch/fc03c897ffa6/px4-planner-review`.
- GPT B branch: `ai/native-sitl-20261026`, created/verified by GPT B only.
- GPT B remains the sole owner of laptop recovery, shared WSL, native execution
  integration and Gazebo. GPT A has made no changes to those environments.
- Preserve GPT B's earlier local planner commit and dirty files. They are not
  replaced by this branch, and the agreed starting SHA has not changed.
- Additional spending: $0. Physical operations: none.

## Confirmed defects and corrections

| Finding | Reproduced behavior at the base | Correction |
|---|---|---|
| Required point loss | Simplifying 49 collinear marked stops to a 48-vertex budget retained only the 2 endpoints. Geometric error alone did not detect lost stop semantics. | Preserve every mandatory endpoint/stop/exact point. Refuse with an explicit budget error when mandatory points exceed the limit. Preserve the input. |
| Vehicle speed bypass | An 80 m phase lasting 35 s passed at approximately 5 m/s under a 3 m/s vehicle model when phase metadata claimed a 10 m/s cap. | Check both the requested phase cap and 90% of the immutable model speed, consistent with existing phase margins. |
| Invalid numeric phase fields | Booleans could act as numbers; strings could escape as TypeError. | Require finite numeric scalars, valid four-element poses and positive limits/duration. Malformed phases raise ValueError. |
| Incorrect browser certificate scope | Missing/invalid original-request error bounds could fall back to an effective-route certificate after repair. A claimed requested bound could also bypass a missing geometry check. | Report no original-request certificate unless the effective geometry certificate and original-request bound are both valid. |

Also resolved 16 pre-existing lint findings in owned planner/test files: explicit
`zip(strict=...)` semantics and one unused test variable. No vehicle limits were
relaxed. No production schemas, CSV columns, runner compatibility flags or
simulation bindings were changed.

## Independent mathematical and browser evidence

`scripts/generate_septic_reference_fixture.py` imports no planner code. It
differentiates the rest-to-rest septic in exact rational arithmetic and exports
123 query states across translation, a stationary turn and another translation.
Noninteger clocks, coupled N/E/Down coordinates, heading above 2*pi, and shared
phase endpoints are included. Both Python generation and JavaScript CSV playback
are checked against this fixture. A separate NumPy polynomial-root calculation
locates derivative extrema and checks the continuous Bernstein bounds.

These checks establish properties of the implemented reference family. They do
not establish physical tracking accuracy, an optimal planner, or a global proof
for all possible floating-point inputs. Continuous certificate checks and
sampled playback comparisons have distinct roles.

Evidence directory: `validation/2026-10-26-planner-review/`.

| Check | Result |
|---|---|
| Untouched base: `python scripts/validate.py` | PASS, 417 Python tests |
| New Python adversarial/reference tests before correction | FAIL as retained in `new-regressions-before-fix.txt` |
| Browser certificate regressions before correction | FAIL as retained in `browser-regressions-before-fix.txt` |
| New Python tests after correction | PASS, 11 tests |
| All JavaScript test files | PASS, 44 tests |
| Ruff, including `tools/` | PASS |
| Connector subprocess tests | PASS, 5 tests; local connector code only, no laptop connection |
| Portable public build and source/manifest audit | PASS |
| Offline acceptance matrix | PASS, 16 cases; `campaign_ran=false` |
| Complete corrected Python validation | PASS, 428 tests; `VALIDATION_PASSED` |
| Actual Chromium UI interaction | NOT RUN: browser download failed; retained error log |
| PX4/Gazebo build, simulation, telemetry and physical tracking | NOT RUN by GPT A; owned by GPT B |

The reference fixture regenerates byte-for-byte. The build audit compares the
bundled source bytes and hashes with this checkout and confirms the controller
and studio executor are not bundled. It does not claim a Pyodide browser session
ran. Logs retain failures as well as successful checks.

## Execution fixture and unchanged interface

`scripts/generate_constructive_execution_fixture.py OUTPUT_DIRECTORY` produces
an accepted right-angle stop-and-turn reference. The retained copy is
`validation/2026-10-26-planner-review/constructive-execution/`.
Extract `artifacts.zip` into an unused directory to obtain the complete,
unmodified result JSON, route CSV, full mission CSV, request and SHA-256 list.
The request and hashes are also available beside the archive. Hashes inside
the manifest refer to extracted files. Archive bytes are reproducible.

- Frame: LOCAL_NED; x north, y east, z down; displayed height is -z.
- Time: seconds from the relevant artifact's start; full mission and route have
  separate clocks. Use mission phase offsets when relating their rows.
- Yaw: radians, zero north, positive toward east; preserve unwrapped turns.
- Position, velocity, acceleration and jerk share stationary phase joins.
- Route and full-mission CSVs have distinct hashes and must not be substituted.
- Planner acceptance remains separate from native executable acceptance:
  `simulation_compatible=false`, `execution_ready=false`, `flight_approval=false`
  and `campaign_ran=false` in this fixture.
- The baseline spline-only execution guard must continue refusing this family
  until GPT B implements and independently verifies its supported contract.
- Do not use the raw rational-oracle fixture as a certified flight plan.

No shared contract change is proposed by this patch. GPT B must review the
stricter speed and required-mark refusals for execution consequences. Any native
family/schema change still needs both GPTs' review and an independently usable
positive and negative fixture before integration.

## Package status and next relay

| Package | State |
|---|---|
| A1 mathematical audit/reference fixtures | Implemented; awaits GPT B review |
| A2 substantiated planner corrections | Implemented; awaits GPT B review |
| A3 browser playback/comparison correctness | Offline checks complete; real UI check blocked by missing Chromium; GPT B can run the existing no-simulation `scripts/check_studio_usability.py` against its local studio |
| A4 adversarial checks/artifact fixtures | Implemented; awaits GPT B review |
| A5 independent native and measured-results review | WAITING for GPT B's exact commit and raw evidence |

Vin should relay this note and the exact published branch/commit to GPT B.
GPT B should proceed with B1 if it has not yet received the kickoff, preserve
existing work, review these changes and return its exact integration commit,
commands/exit codes, model/source/artifact hashes and raw telemetry/truth files.
Include nominal and cancellation outcomes, actual landing/disarm/stopped-output
observations, and any unsupported cases. GPT A will independently inspect the
changed contract and recompute available metrics. Neither workstream is finally
accepted until the other GPT reviews it and the combined revision is tested.

No direct cross-account delivery channel has been established. A file or branch
does not itself notify or start the other GPT.
