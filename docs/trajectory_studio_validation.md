# Offline planner increment: validation record

Date: 19 October 2026. Baseline: PR #3 at
`2b58bb1166dddc7f918b6d671c1005edce9066f2`. Implementation code was published
in commit `c7db558e34f8e7c58419897d9e65d382f058da58`; the following documentation
and CI update records the results and expands lint scope. This record does not
claim a live simulator campaign or physical-aircraft qualification.

## Checks completed

| Check | Result |
|---|---|
| `python3 scripts/validate.py` | `VALIDATION_PASSED`; 318 tests passed in 154.693 s |
| Python compilation | Passed through the validation entrypoint |
| Waypoint determinism and endpoint-inclusive baseline | Passed through existing regression checks |
| Lint: planner, controller, analysis, scripts, tests and trajectory generator | Passed |
| Inline editor JavaScript, `node --check` | Passed |
| Drawing and interview through real compiler/verifier | Passed |
| Real local HTTP request -> isolated compiler worker -> certificate | Passed; invalid input returned HTTP 422 without an accepted plan |
| Cross-origin/host rejection, oversized requests, absent hardware endpoints | Passed |
| Duplicate JSON keys, coerced flags/lap counts, unused fields, oversized survey expansion | Passed |
| Track truncation, missing-middle gaps, malformed thresholds/timestamps, lost yaw rotations | Passed |

The 318 tests include the original 298 regression tests and 20 new editor,
input-boundary and evidence tests. Two existing synthetic comparison calls
declare their deliberately sparse 2 s sample gap explicitly.

## Offline demonstration

The existing interview circle example compiled through the preview worker:

| Quantity | Result |
|---|---:|
| Selected route duration | 58.716044 s |
| Exported controller samples | 1,176 |
| Binding constraint | Speed |
| Certified speed bound | 0.900000 m/s |
| Certified acceleration bound | 0.789773 m/s² |
| Certified jerk bound | 1.349323 m/s³ |
| Certified tilt bound | 4.619284 degrees |

These are planned bounds under the existing model, not measured aircraft or
Gazebo tracking results. `campaign_ran` is false and `simulation_status` is
`NOT_RUN`. The reported duration is a feasible result for the current candidate
family; global optimality is not established.

## Review environment

| Component | Version |
|---|---|
| Python | 3.12.14 |
| numpy | 2.3.5 |
| matplotlib | 3.10.8 |
| pandas | 2.2.3 |
| pymavlink | 2.4.50 |
| pyulog | 1.2.4 |
| ruff | 0.17.1 |

The repository's CI also checks its configured Python 3.10 environment. This
local record does not claim that remote checks have finished. A reproducible
physical release still needs a pinned deployment environment and configuration.

## Checks not completed

- Browser visual/interactivity QA: Chromium could not be downloaded in this
  environment. HTTP and JavaScript syntax checks do not establish layout or
  pointer behavior in the target browser.
- Live Gazebo/PX4: neither is installed here. The local Windows connector
  returned an unavailable 429 response on both access attempts.
- Nominal/fault campaign, simulator truth comparison, lifecycle evidence,
  real-time factor, hardware identification, bench tests and physical flight.

The next action is to restore access to the existing PX4/Gazebo machine, run
browser QA there, reproduce the five-stop baseline and execute the planned
general-trajectory simulation campaign. Aircraft and operating decisions are
required before the later physical-release gate.

See [the research review](trajectory_planner_research_review.md) for requirements,
trade studies, proposed thresholds, hazards and release gates, and
[the quickstart](trajectory_studio_quickstart.md) for the offline editor.
