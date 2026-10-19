# Offline trajectory studio

The studio adds drawing and interview inputs to the existing verified planner.
It is a local, simulation-only planning preview. It does not start Gazebo,
connect to an aircraft, issue a SITL acknowledgement, or approve a flight.

From the repository root, using its Python environment:

```bash
python3 -m pip install -r requirements.txt
python3 -m mission_planner.editor
```

On Windows, use `python` in place of `python3` if that is the environment's
Python command. Open <http://127.0.0.1:8765>. A different port is available with
`--port 8766`. The server binds to loopback only. Stop it with Ctrl+C.

## Use

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

## Explicit preview assumptions

- `LOCAL_NED`, metres, launch origin `launch-001`; horizontal drawing plane.
- Model: the existing conservative x500 SITL configuration, maximum speed
  3 m/s. It is not an identified physical aircraft.
- Environment: existing empty-world contract, 40 m radius and maximum 30 m
  launch-relative height, without an obstacle map. No obstacle-avoidance claim.
- Timing: current certified spline schedule, 0.9 limit scale; the preferred
  speed currently acts as a cap, then is scaled by that reserve.
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
  requests may require the CLI. Preview files are temporary; use downloads
  for anything to retain.

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
Live nominal mission completion and fault qualification remain outstanding.

For the researched architecture, traceable gaps, benchmark plan and human gates,
see [the research review](trajectory_planner_research_review.md).
