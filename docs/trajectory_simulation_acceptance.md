# Trajectory planning and simulation acceptance

This milestone ends at an inspected PX4/Gazebo result. Use the reference x500
simulation; aircraft setup and physical deployment are outside this milestone.

## Required behavior

1. A fresh studio opens with the reference simulator selected, without an
   aircraft questionnaire. Advanced profiles remain optional.
2. Drawing and interview inputs use the same independent trajectory checker.
3. Geometry stays within its declared tolerance. Corners, starts, stops and
   speeds are smoothed, and infeasible inputs produce an explanation.
4. The execution CSV contains the full trajectory and derivatives. Display
   samples cannot replace it. Editing invalidates a previously accepted draft.
5. The user must confirm the exact retained trajectory before Gazebo starts.
6. The simulated mission completes takeoff, route, return, native landing,
   disarm and independently observed stopped propulsion.
7. Simulator truth is assessed independently and never feeds flight control.
   Position, heading, route tolerance, actual speed and actual tilt are checked.
   Acceleration is reported separately from reference acceleration; a sampled
   acceleration observation is not a measured jerk certificate.
8. The browser compares requested, planned and simulated paths, supplies the
   assessment, and preserves failures rather than treating them as passes.
9. A long compilation does not block status polling or a cancellation request.
   Concurrent compiler requests are refused, and job ownership is serialized.
10. Cancellation requests native PX4 LAND and requires fresh landed/disarmed/
    stopped-output evidence. An interrupted route is not nominal completion.

## Reproduce

Run the normal repository checks, followed by the declared offline matrix:

```bash
python3 scripts/validate.py
python3 -m ruff check controller mission_planner analysis scripts tests trajectory_generator
python3 scripts/check_trajectory_acceptance.py --output /absolute/path/to/new-offline-check
```

The offline matrix includes straight, L-turn, tight-turn, S-curve, closed-circle,
hard-stop, exact-fly-through, slower-route, interview-circle and survey inputs.
It also requires refusal of an impossible deadline, excessive requested speed,
an out-of-volume route, zero-length geometry, conflicting stop flags and NaN.
It inspects the complete exported CSV, endpoint derivatives and exact hash.
Its report explicitly says Gazebo has not run.

On the configured simulation machine, use a fresh output directory:

```bash
python3 scripts/planner_sitl_campaign.py --confirm-sitl \
  --autopilot /home/xpire/PX4-Autopilot \
  --output /absolute/path/to/new-campaign --gui \
  --cases drawn-nominal circle-nominal turn-nominal hard-stop-nominal survey-nominal
```

For the real Chrome-to-studio workflow, run the existing browser check once
normally and once with `--cancel-during-route`. The normal reference run also
draws a freehand L-shape and checks an impossible deadline before its connected
rehearsal. Use a separate output directory for each invocation:

```bash
python scripts/check_studio_browser.py --confirm-sitl \
  --chrome /absolute/path/to/chrome --output /absolute/path/to/browser-evidence
python scripts/check_studio_browser.py --confirm-sitl --cancel-during-route \
  --chrome /absolute/path/to/chrome --output /absolute/path/to/cancel-evidence
```

## Manual trial

Open <http://localhost:8766>. Keep the reference simulator and **Hold launch heading** (fixed heading);
begin at the demonstrated 20 m height, 1 m/s preferred cap, 1.25 m/s hard cap
and 0.5 m path tolerance.

1. Clear the canvas and click or drag a route from launch. Try a straight line,
   an L-turn and a circle separately.
2. Select **Plan and verify**. Inspect the original/planned overlay, duration,
   limiting constraint and speed/altitude/heading/acceleration plots.
3. Check the confirmation box, then **Confirm and rehearse in Gazebo**.
4. Inspect the simulation window and phase status. After completion inspect
   the truth overlay and download the simulation assessment.
5. On another newly confirmed plan, try **Request safe simulator cancellation**.
   Wait for the explicit landed/disarmed/stopped-output confirmation.
6. Try a one-second deadline: it should be refused. Change any accepted draft:
   confirmation and downloads must invalidate until the new draft is checked.

Planning may take time. Rehearsals also include startup and landing, so route
duration is shorter than total simulator wall time. Launch health/height checks
can refuse a preview that was mathematically feasible under its static inputs.

## Scope of the evidence

A passing matrix establishes these named cases and thresholds, not every
possible drawing or every environmental disturbance. Connected rehearsals
currently require fixed heading; tangent and other heading policies retain
their documented offline scope. Online replanning, unexpected obstacle sensing,
global time optimality and continuous actual-motion/rotor certification are
outside this baseline. Inspect the retained per-case results before reporting
the milestone as passed.
