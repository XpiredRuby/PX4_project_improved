# Plan and inspect a trajectory

The public studio runs the Python/NumPy planner on your device. It does not run
Gazebo or connect to aircraft. Draw anywhere, choose an example, or place
points. Separate strokes connect in drawing order from launch. Original drawing
samples remain visible for comparison with the accepted reference.

1. Choose Straight, L turn, Circle or Vin, or draw a route.
2. Set height and preferred cruise speed. Never exceed is visible beside cruise;
   it remains binding. The fresh x500 demo cap is 3 m/s. A saved lower cap is
   preserved and a conflicting cruise request produces an explanation.
3. Select Plan and verify. First use downloads the engine. Stage text and elapsed
   time describe computation; Cancel check stops the browser worker and keeps
   the drawing. Retry starts a fresh worker. Editing invalidates an older result.
4. Review the original/accepted overlay, adjustments, mission phases and speed
   explanation. Closest mode may simplify, project, retime, add stops or scale.
   It discloses changes and whether the original rounding allowance was met.
5. Play the full exported reference, pause, scrub, step exact CSV samples or jump
   to events. 1x means one mission second per wall second. Compass yaw is separate
   from velocity; stationary turns rotate without translating.
6. Download the full mission CSV, route PNG or JSON report. Flight data is a
   planned reference. It is not measured flight, touchdown or shutdown evidence.

The requested cruise is an upper preference, not a promised constant speed.
The existing 10% planner margin and acceleration, jerk, turns and stops can make
motion slower. What sets the pace shows requested speed, effective planning cap,
full-route sampled peak and mean. The 3 m/s simplified x500 model ceiling has not
been raised. A 5 m/s request is adjusted and disclosed.

Freehand capture retains up to 8,192 comparison samples with bounded thinning
for very long gestures; the planner schema accepts up to 10,000 points. Dense
sketches use a bounded working set (up to 48 vertices). The original remains
separate; stop/order/fidelity and physical checks still apply. Global optimality
and obstacle avoidance are not claimed. Drafts save on this device when browser
storage is available. Restored drafts require a fresh check.

Named waypoint missions with explicit `hold_s` dwell durations use the existing
`trajectory_generator/plan_waypoint_mission.py` CLI and
`missions/five_stop_example.json`. Drawing-editor stops currently have no timed
dwell field; a stationary heading turn is not an explicit dwell request.

## Local simulation

On the configured simulator machine:

```bash
python -m mission_planner.editor --port 8766 \
  --sitl-autopilot /home/xpire/PX4-Autopilot \
  --runs /home/xpire/PX4-runs/trajectory-studio
```

Without the SITL flags this is planning-only. Keep the development server on
loopback. The connected studio requires an exact retained and confirmed artifact,
a matching model and the qualified fixed-heading spline path. Native takeoff,
return, landing and disarm are separate from the browser's full-mission reference.
The runner accepts route-only execution CSV; never substitute the browser's
full ground-to-ground CSV.

Constructive stop/turn execution remains an explicitly experimental CLI path,
with diverse artifact checks, fixed heading and constant altitude. It is not
unlocked in the ordinary studio until connected qualification succeeds. See the
[engineering audit](conference_engineering_audit.md) for the exact command.

Native execution also has a separate conservative 1.0 m/s reference-progress
policy and 1.25 m/s horizontal command cap. Navigation health and tracking can
slow the phase clock further. Faster planning does not change these policies.
Qualify higher-speed native tracking with new evidence before changing them.

Cancellation of a simulator flight requests native landing and must finish with
fresh landed/disarmed/stopped-propulsion evidence. Cancelling a browser planning
check only stops computation. Physical flight is not qualified or authorized.
