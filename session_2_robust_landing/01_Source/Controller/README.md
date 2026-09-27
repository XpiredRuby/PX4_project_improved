# PX4 randomized airborne initialization overlay

This overlay keeps the validated `fixed` controller intact and adds a bounded,
simulation-only airborne initialization experiment.

- Random X/Y: `[-2, +2] m`
- Random Z height: `[3, 10] m` by default (configurable inside the hard
  `[2, 15] m` envelope)
- Random roll/pitch: `[-2, +2] deg` by default (configurable stress-test
  limit `+/-15 deg`)
- Random yaw: `[-180, +180] deg`
- The seed and exact requested/observed pose are archived in
  `spawn_config.json`.
- Gazebo physics is paused across teleport and detachable-joint attachment, so
  the fixture captures the requested pose without setup free-fall. The fixture
  then holds that exact bounded pose while the estimator settles disarmed. PX4 then enters OFFBOARD, arms, and uses a
  bounded 0.80 m/s upward command to spool the propulsion system. Release occurs
  only after fresh telemetry holds target thrust >= 0.62 and mean four-motor
  output >= 650 for 0.15 s, without a second pose jump. Mean output measures
  total lift while allowing the expected motor split at a tilted fixed pose.
- The launcher-supplied Z height defines the backup local ground estimate; a
  stored pre-spawn ground reference is the final target when available.
- Descent slows inside the final metre, then hands actual contact detection and
  auto-disarm to PX4 LAND.
- Visible validation exports `PX4_POST_DISARM_HOLD_S=10`, keeping Gazebo open
  for 10 continuously verified seconds after auto-disarm. During that interval,
  the runner requires fresh telemetry, four zero propulsion outputs, and settled
  motion. It does not issue a forced disarm or motor-cut command; PX4 LAND
  remains responsible for touchdown and shutdown.
- Each completed run includes `randomized_metrics.json` with stabilization,
  trajectory RMSE, landing handoff, and terminal X/Y/Z errors.
- A run fails if stabilization exceeds 2 m horizontal excursion, 2 m/s
  horizontal speed, 1 m vertical excursion, or 1.5 m/s vertical speed—even
  if the eventual landing is accurate.
- The final three-trial evidence is in
  `/mnt/f/PX4/research/analysis/randomized-final-validation/VALIDATION_REPORT.md`.

Random trial:

```bash
bash /mnt/f/PX4/research/analysis/run_randomized_regression.sh random 12345
```

Manual trial:

```bash
bash /mnt/f/PX4/research/analysis/run_randomized_regression.sh \
  manual 1.0 -1.0 6.0 2.0 -2.0 90.0
```

Arguments are `x y z roll_deg pitch_deg yaw_deg`. This workflow is for PX4 SITL
and Gazebo only.
