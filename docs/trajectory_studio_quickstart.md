# Draw a route and try it in Gazebo

The studio turns your line into a smoother, checked trajectory, then lets you
watch the virtual drone follow it. The default virtual drone is ready to use;
you do not need to complete an aircraft questionnaire.

## First try

Open <http://127.0.0.1:8766> on the configured simulation computer. Keep the
standard virtual drone, 20 m height and 1 m/s cruising speed for your first run.

1. Choose **Straight**, **L turn** or **Circle** above the map. Alternatively,
   use **Draw a line**, press at the gold **START** marker, drag and release.
   Continue a line from **END**. A disconnected line shows a dashed connection
   for you to add or discard; it is not joined without your decision.
2. Select **Plan and verify**. Nothing starts flying. Review the blue smooth
   route and the route-duration summary. Sharp corners may be rounded within
   the allowed planning distance, and the drone slows for turns and stops.
3. Tick **I reviewed the blue route and want to simulate this plan**, then
   select **Start simulation**.
4. Watch takeoff, the route, return and landing. Afterwards compare the purple
   simulated flight path with the blue plan and **Download simulation results**.

The route duration excludes takeoff, the controller-generated return and landing.
Even when an open route ends at END, the current simulator workflow returns to
START before landing. A faint dotted return indication is not a certified return
trajectory; the controller generates that motion from its measured state.

## Drawing controls

- **Draw a line:** drag from START, or continue from END. The released endpoint
  is retained. There is no automatic 0.6 m point-discard rule or unreported
  simplification of your committed input.
- **Place points:** click each point in order. The next segment is previewed
  with the pointer. Each click intentionally connects that point to END.
- **Zoom in / Zoom out / Fit area:** change the map view without changing the route. Labels and the START target adapt to screen size.
- **Undo / Redo:** reverse or restore a complete drawing action.
- **New drawing:** return to the empty START marker. Undo can restore the route.
- **Enter points or add stops:** enter east/right and north/up coordinates;
  Enter adds a point. Interior points can be ordinary rounded corners, full
  stops, or exact interpolation points. An exact point does not promise a
  minimum pass-through speed; a stop does not add a timed wait.
- **Route ending:** finish the open route, or close a loop back to START.
  Loops with interior marked stops/exact points are not supported and are refused.

The map shows the configured reference boundary and a 5 m grid. Out-of-bounds
or oversized gestures are refused as a whole; your previous valid route stays
in place. A route may contain at most 256 points including any added closure.
For long routes, placing points can be easier than a dense freehand stroke.
No geometric reduction is currently used, so the full planning tolerance is
available to the existing trajectory checker.

## Settings in everyday terms

| Setting | Meaning |
|---|---|
| Flight height | Height above launch during your route. The reference volume permits up to 30 m; connected health/height checks may still refuse a lower height. |
| Cruising speed | A preferred cap, not a constant speed. 1 m/s is about walking pace. |
| Allowed rounding | Maximum modeled distance the smooth plan may depart from the entered line. This is not guaranteed actual flight accuracy. |
| Never exceed | The hard route speed cap. Cruising speed cannot exceed it. The planner keeps its existing 10% speed margin. |
| Facing direction | Keep the starting direction for connected rehearsals. Facing along the route is currently preview-only. |
| Route time limit | Optional maximum for the route only. A too-short request is refused; limits are not relaxed to meet it. |

The details panels contain motion charts, the certificate, assumptions and the
full report. Display samples are decimated; **Download trajectory CSV** provides
the complete checked execution data. The reported feasible duration applies to
this planner's candidate family; it is not a proven global minimum.

Editing a drawing or setting invalidates the plan and confirmation. Replan before
starting it. An earlier confirmed simulation retains its identity and continues
monitoring separately, even when you edit the next draft. Reloading resumes job
monitoring when that job is still retained; unsaved drawing persistence is not
promised. A missing old job does not permanently block a new plan.

## Stopping a virtual flight

Select **Stop simulation and land**. Wait for explicit landing, disarm and stopped
motor confirmation. A request to stop is not yet a completed stop. The controller
uses the existing native landing/cleanup flow; the browser does not kill an
airborne simulator or force-disarm it. An interrupted route is not nominal
mission completion. Status loss does not mean the simulator has stopped.

## Start the local application

From the repository root with its dependencies installed:

```bash
python3 -m mission_planner.editor --port 8766 \
  --sitl-autopilot /home/xpire/PX4-Autopilot \
  --runs /home/xpire/PX4-runs/trajectory-studio
```

Keep code, the WSL disk, temporary files, caches and retained run evidence on the
configured flash drive. Use the existing tunnel launcher for its environment.
The server is loopback-only. Starting it without the SITL arguments enables
planning/downloads but does not enable Gazebo execution. Custom simulator profiles
and matching model registration remain optional; see [aircraft profiles](aircraft_profiles.md).

All artifacts remain SITL_ONLY. This increment covers trajectory planning and
simulation, not a physical-flight workflow or obstacle avoidance.

## Developer verification

```bash
python3 scripts/validate.py
python3 -m ruff check controller mission_planner analysis scripts tests trajectory_generator
node --test tests/studio_drawing.test.cjs
```

Browser-only acceptance uses the real HTTP compiler and installed Chrome. Its
start-race tests intercept simulation endpoints; it never starts a simulator:

```bash
python scripts/check_studio_usability.py --url http://127.0.0.1:8767 \
  --chrome /absolute/path/to/chrome --output /absolute/path/to/new-ui-evidence
```

For an actual confirmed browser-to-Gazebo rehearsal, use
`check_studio_browser.py --confirm-sitl`; add `--cancel-during-route` in a separate
run for cancellation. Use fresh output directories and retain failed attempts.
See [acceptance scope](trajectory_simulation_acceptance.md) and
[interaction specification](trajectory_studio_interaction_plan.md).
