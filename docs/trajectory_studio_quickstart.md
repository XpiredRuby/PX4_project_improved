# Local trajectory studio

The studio adds drawing and interview inputs to the existing verified planner.
By default it provides an offline preview. An explicit Linux/WSL backend adds
confirmed local PX4/Gazebo rehearsals. Physical flight has no browser endpoint. The separate
[supervised physical-release CLI](physical_flight_release.md) handles reviewed
aircraft qualification, preparation, local launch and abort.

From the repository root, using its Python environment:

```bash
python3 -m pip install -r requirements.txt
python3 -m mission_planner.editor
```

On Windows, use `python` in place of `python3` if that is the environment's
Python command. Open <http://127.0.0.1:8765>. A different port is available with
`--port 8766`. The server binds to loopback only. Stop it with Ctrl+C.

## Use

The studio now opens in **20 m/s research preview**, an **offline-only**
mode with an illustrative, unqualified multicopter model. Its hard speed
ceiling is 20 m/s and its initial preferred cap is 10 m/s. Enter any
preferred cap up to 20 m/s. The planner does not fall back to 1.25 m/s,
but acceleration, jerk, geometry and available path length can still make
the actual trajectory peak slower than requested. This mode has **no PX4/Gazebo
rehearsal** and is not evidence of real-drone performance.

For a vehicle-specific plan, select **Set up an aircraft**. Enter a loaded
configuration, import a saved profile, or use the registered simulator profile. **Check setup** lists missing
inputs and compatibility errors. Quad, hex, octo and custom parallel-rotor
geometry feed the same independent checker. Save/export the profile for reuse.
The explicit x500 reference demo remains available without custom setup.
See [aircraft profiles](aircraft_profiles.md) for the input sources, accuracy
and energy budgets, simulator registration and supported aircraft classes.

1. In **Draw**, click points or drag a stroke. The route starts at launch.
   Undo or clear the input, or select a straight route, L turn or circle.
   Choose final stop or closed lap. The latter visibly closes to launch.
2. In **Interview**, choose circle, lawnmower survey or waypoints. Specify the
   radius/laps, survey dimensions, or waypoint coordinates and optional stop
   flags. Both flows feed the same `MissionIntent` and independent checker.
3. Set launch-relative altitude, geometric tolerance, preferred speed cap,
   hard maximum, heading policy and optional maximum route time.
4. Select **Plan and verify**. Inspect the input/plan overlay, speed, height,
   heading, acceleration and certified limits. A refused input produces an
   error rather than an accepted output. Changing any field invalidates the
   visible result and downloads; an in-flight preview response is discarded
   if it belongs to an earlier draft.
5. Download both the full CSV and offline plan bundle for review. The bundle
   includes intent, summary, certificate and decimated display samples. The
   full CSV is the execution data; do not reconstruct it from plotted samples.

## Connected PX4/Gazebo mode

Use a Linux or WSL environment with the built PX4 executable, Gazebo and the
repository dependencies, including `pyulog`. On the configured machine:

The commands below start the reference x500 backend. For reusable profiles,
register the matching SDF/airframe and add `--sim-binding` as shown in
[the aircraft setup guide](aircraft_profiles.md#register-a-matching-simulator).
A custom profile cannot silently use the reference x500 simulator.

```bash
cd /home/xpire/PX4-trajectory-studio-oct19
python3 -m mission_planner.editor --port 8766 \
  --sitl-autopilot /home/xpire/PX4-Autopilot \
  --runs /home/xpire/PX4-runs/trajectory-studio
```

Open <http://127.0.0.1:8766>. This mode uses the existing `afvl_validation`
world and x500 model, a separate PX4 working directory per job, loopback
MAVLink endpoints and a requested simulation speed factor of 1. The GUI is
on by default; `--headless` is available for supplementary batch checks.
An existing PX4 instance is preserved and prevents a new rehearsal.

After planning, review the proposed interpretation and certificate, check the
confirmation box, then select **Confirm and rehearse in Gazebo**. Confirmation
binds the exact retained CSV digest. The server does not recompile or silently
change its altitude. The default editor height is 20 m; the live controller
still checks its uncertainty-aware minimum and can refuse a lower height or
worse navigation state. Connected rehearsals currently require fixed heading;
tangent launch alignment remains an explicit qualification gap.

The page polls the job, shows failed checks as failures, provides the assessment
for download, and overlays decimated simulator truth for the same plan. It
resumes monitoring the active job after a page refresh. Editing a draft does
not cancel an already confirmed rehearsal; results remain associated with
the earlier artifact. Use a newly verified plan for a new rehearsal.

**Safe simulator cancellation:** While a confirmed Gazebo rehearsal is running, use **Request safe simulator cancellation**. The request is tied to the exact retained plan ID and CSV SHA-256. The controller yields to native PX4 LAND/failsafe cleanup; the browser never kills an airborne simulator or force-disarms. Only fresh landing, disarm and stopped propulsion evidence permits a `CANCELLED` verdict. Missing evidence remains `FAILED`, and cancellation is not hardware authorization. Create and confirm a new plan to repeat the rehearsal.

Each retained plan contains its intent, model, environment, CSV, summary,
verification report and review record. Its `rehearsal/` directory contains
configuration and parameter readback, controller telemetry, ULog, lifecycle
audits, ground/disarm evidence, and `assessment.json`. Inspect failed targets
and the assessment's remaining qualification; a successful sampled rehearsal
is not a physical-flight clearance. The assessment explicitly distinguishes
an adaptive mission clock from actual wall-time dynamics.

There is no hardware launch button. The non-executable
[`physical_aircraft_intake.template.json`](physical_aircraft_intake.template.json)
lists the missing owner information. A real aircraft model, operating domain,
qualified failure response, hardware tests and a separate operator approval
are required before a physical execution adapter can be enabled.

## Browser regression on a configured simulation machine

The reusable browser check requires optional Playwright (`python -m pip install
playwright`) and an installed Chrome executable. It actively confirms and runs
one local SITL mission, checks artifact mismatch refusal, edit invalidation,
refresh recovery, truth overlay, assessment download and responsive layout.
Use a new output directory:

```bash
python3 scripts/check_studio_browser.py --confirm-sitl \
  --chrome /absolute/path/to/chrome \
  --output /absolute/path/to/new-browser-evidence
```

On Windows, use the connected Python environment and the installed Chrome path.
The test accepts only a loopback studio URL. Its assertions never approve a
physical flight.

## Explicit preview assumptions

- `LOCAL_NED`, metres, launch origin `launch-001`; horizontal drawing plane.
- Research default: illustrative offline-only model with a declared 20 m/s
  ceiling, 1.0 limit scale, and no validated high-speed tracking. Its output
  cannot be submitted to the built-in PX4/Gazebo campaign.
- x500 reference demo: a separate conservative SITL model with 3 m/s maximum
  and 0.9 dynamic reserve. Changing to this mode explicitly sets its preferred
  cap to 2 m/s and hard ceiling to 3 m/s. It does not inherit the 20 m/s study.
- Custom aircraft: the declared vehicle speed and 0.9 reserve still bound the
  resulting trajectory. The aircraft profile is not a speed permission.
- Environment: existing empty-world contract, 40 m radius and maximum 30 m
  launch-relative height, without an obstacle map. No obstacle-avoidance claim.
- Timing: current certified spline schedule. The preferred speed is a cap,
  not a guarantee of attained speed; the independent verifier reports the
  actual speed bound. Aircraft-specific modes retain 0.9 reserve; research
  preview uses the full declared envelope to study the requested limit.
- Geometry: requested control spacing 0.5 m, verification resolution 0.02 m,
  75% smoothing utilization of the accepted geometric deviation tube.
- CSV sampling: 0.05 s. Display samples are decimated to about 1,000 points;
  certificates apply to the original continuous representation, not the plot.
- The first route point is the selected cruise altitude above launch. This
  route certificate does not itself verify takeoff/landing transitions.
- Tangent heading requires the existing runner's launch alignment convention.
  Heading is not silently transformed into an independently verified absolute
  earth-frame yaw by this editor.
- At most 256 input points and a 120 s compilation budget. Large/complex
  requests may require the CLI. Offline preview files are temporary; use downloads
  to retain them. Connected mode also retains server-side evidence under `--runs`.

Use the CLI for map-bound corridor missions, full 3-D inputs, look-at/profile
yaw, entry/exit legs and other options already supported outside the basic
editor. The editor intentionally has no arbitrary file paths or shell-command
inputs and no hardware execution endpoints.

## Evidence changes in this increment

`load_contract` rejects duplicate JSON keys as well as non-finite constants.
Interview normalization rejects non-boolean stop flags, fractional/string/bool
lap counts, incorrect version types and unused/unknown pattern fields. It
validates the normalized contract before returning it.
Generated survey/circle expansions are bounded before constructing oversized
point lists. Plan summaries and duration-refusal messages explicitly scope the
found duration to the planner's candidate family, without claiming a proven
global minimum.

`compare_flown_track` now requires complete mission-clock coverage and enforces
`maximum_sample_gap_s` (default 0.25 s). It rejects non-finite or nonpositive
thresholds, duplicate columns and invalid time ordering, and preserves
unwrapped reference yaw during interpolation. `endpoint_tolerance_s` is
explicit, defaults to 1 microsecond and cannot reach half the mission duration.
Use a justified gap for synthetic sparse tracks; do not relax it after seeing
a physical result. Align and trim takeoff/landing separately.

The comparison still reports `campaign_ran: false`. It measures sampled
tracking only, not continuous error, lifecycle completion, simulator identity
or a complete Gazebo campaign. Those remain separate implementation gates.

## Validation

```bash
python3 scripts/validate.py
python3 -m ruff check mission_planner/editor.py mission_planner/contracts.py \
  mission_planner/retiming.py mission_planner/plan_drawn_path.py \
  mission_planner/plan_interview.py mission_planner/sitl_campaign.py \
  tests/test_planner_input_and_evidence.py tests/test_planner_editor.py
```

The focused tests include both real compiler inputs, a real HTTP-to-worker
round-trip, rejected cross-origin/host requests, missing launch endpoints,
parser adversarial cases, truncated tracks, gaps and lost yaw rotations.
JavaScript syntax was also checked with Node. The subsequent connected-machine
Chrome checks passed for drawing, interviews, refusal, invalidation, downloads
and mobile overflow; see [the connected validation checkpoint](trajectory_studio_connected_validation.md).
Live results and any remaining failed targets are recorded in that checkpoint;
unit tests alone do not establish simulator or hardware qualification.

For the researched architecture, traceable gaps, benchmark plan and human gates,
see [the research review](trajectory_planner_research_review.md).
