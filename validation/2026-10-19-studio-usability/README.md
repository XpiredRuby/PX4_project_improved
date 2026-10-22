# Drawing and simulation usability checkpoint

Date: 2026-10-19 (America/Chicago). Scope: reference trajectory planning and
PX4/Gazebo. This increment makes the existing workflow usable without needing
to understand spline fitting, MAVLink, controller parameters or certificates.

## Delivered behavior

- Empty map with a labeled START, END, arrows, visible reference boundary,
  screen-sized start target and zoom controls. On phones the map comes first.
- Separate Draw a line and Place points tools, coordinate entry, stops and exact
  interpolation points, and Undo/Redo by complete editing action.
- Disconnected strokes require an explicit connecting-link review. Release
  endpoints and short details are retained. Invalid/oversized gestures preserve
  the earlier route instead of silently clipping or truncating it.
- Plain-language height/speed settings and Draw / Review / Simulate / Results
  stages. Detailed charts and reports are expandable.
- Clear open-route versus closed-loop wording and lifecycle explanation:
  takeoff, route, controller-managed return, landing and stopped motors.
- Structured time-limit refusals, invalidation of edited plans, serialized
  simulation starts, recovery of uncertain startup responses, and preserved
  monitoring/cancellation of the earlier confirmed plan.

The checker and flight controller retain their existing mathematical behavior.
No geometric reduction is performed. The UI preserves input order, but this
increment does not add a new ordered-correspondence proof at crossings or claim
that every possible drawing has been tested.

## Verification

| Check | Result |
|---|---|
| Full `scripts/validate.py` on the configured WSL machine | 384 discovered Python tests: 383 passed, one Node-dependent wrapper skipped; deterministic trajectory generation and Python compilation passed. |
| Actual drawing model under Node | All 11 JavaScript tests passed separately. |
| Python lint across controller, planner, analysis, scripts, tests and generator | Passed. |
| Final inline JavaScript and drawing-module syntax | Passed. |
| Real Chrome usability campaign | All 10 groups passed, no application JavaScript errors. |
| Initial nominal browser-to-Gazebo run | Passed; retained separately. |
| Repeated nominal run with final interface | Passed all 16 sampled checks, including lifecycle, actual speed/tilt, exact CSV identity, truth overlay and download. |
| Route-phase cancellation through the browser | CANCELLED with confirmed landing, disarm and stopped propulsion; interrupted route is not reported as nominal completion. |

The browser-only campaign uses real HTTP compilation, including a marked stop,
and deliberately intercepted simulation endpoints for start-race/recovery checks.
Its report explicitly says `simulation_ran: false`. The two connected nominal
runs and the separate cancellation are actual PX4/Gazebo runs, with retained
independent truth and lifecycle evidence.

See [checkpoint.json](checkpoint.json) for named checks, job identities, delivery
file hashes, raw-result hashes and per-run source fingerprints. Controller sources
match their retained run manifests. Per-run fingerprints remain attached to the
versions used for those runs; the later browser-test predicate compatibility edit
is recorded in the delivery-file hashes rather than changing historical evidence.

## Retained diagnostics

Earlier attempts are preserved. A local container-wide test attempt lacked
`pymavlink`; complete validation therefore used the configured WSL environment.
One initial full WSL run exposed a synthetic telemetry fixture expiring while
disk hashes were checked. That one-use-consumption unit test now fixes its clock;
the explicit stale-telemetry tests still exercise expiry normally. The repeated
full suite passed.

Visual review corrected mobile ordering, persistent first-step highlighting,
small-screen map redraw and endpoint-label spacing. A later browser-harness
attempt hit Chrome's CSP restriction on string-evaluated wait predicates; the
harness now passes functions, and the final campaign passed. No CSP relaxation
was added.

Raw logs, screenshots, controller CSV/ULog, simulation assessments and browser
results remain on the flash-drive-backed machine. Relevant locations:

- WSL project: `/home/xpire/PX4-studio-usability-oct19`.
- Retained plans/jobs: `/home/xpire/PX4-runs/studio-usability-oct19`.
- Windows results: `F:\PX4\usability-qa-07`,
  `F:\PX4\usability-connected-nominal-02-final`, and
  `F:\PX4\usability-connected-cancel-01`.
- WSL disk registry location was verified as `F:\WSL\Ubuntu-22.04`.

The [deployed-browser check](deployed_browser.json) also passed all 10 groups at
port 8766, including real compilation and the marked stop.

The deployed studio uses port 8766 with project outputs and temporary files on
the flash drive. [Quickstart](../../docs/trajectory_studio_quickstart.md) gives
the user steps and reproduction commands. GitHub CI results remain attached to
their respective commits; this record reports the directly observed checks above.
