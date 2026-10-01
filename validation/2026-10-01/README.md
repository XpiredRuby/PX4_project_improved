# Terrain-height checks

The controller receives GPS/IMU telemetry and no landing-surface height.
Use PX4 v1.17, Gazebo 8.15, the default x500 world, and the audited estimator
and failsafe settings described in the main README.

For the raised-surface case, begin on the world ground plane. After a measured
8 m climb and a fresh armed heartbeat, create the static pad in this folder:

```bash
gz service -s /world/default/create \
  --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'sdf_filename: "/absolute/path/raised_landing_surface.sdf" allow_renaming: false'
```

Require `data: true`. The pad top is at world z = +2 m. Complete the unchanged
mission and audit its log with `python3 analysis/analyze_run.py RUN_DIR`.

For the lowered-surface case, launch the next mission from the pad. After a
measured 8 m climb and a fresh armed heartbeat, remove only that pad:

```bash
gz service -s /world/default/remove \
  --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req 'name: "raised_landing_surface_v95" type: MODEL'
```

Require `data: true`; landing ground is now 2 m below the launch surface.
Do not change the mission's home Z or send a surface-height estimate.
Audit touchdown dynamics, XY error, bounce, disarm timing, and motor output.

These two level-surface cases check an unknown height change. They do not
establish performance on slopes, obstacles, or the full configured ±10 m range.

Each archived flight has an audit, a source/configuration manifest, compressed
telemetry, and an integrity record. Verify the uncompressed CSV against its
SHA-256 hash before analysis. Source hashes identify the code used for each run;
the recorded Git commit can precede uncommitted changes in that run.

`process_loss_probe.py` is an independent SITL observer. Start it before the
runner, with `--confirm-sitl`, `--runner-pid-file PATH`, and `--output PATH`.
Launch the runner as a background process and save only that process's PID to
the specified file. After a measured 8 m climb, the probe verifies the PID's
command line, stops that runner, and observes PX4 without sending a LAND command.
It requires Land as the configured Offboard-loss action, timely takeover,
bounded touchdown dynamics, no bounce, automatic disarm, and fresh zero motor
outputs for one continuous second. A normal runner completion manifest is not
expected after SIGKILL; the independent observer supplies the terminal evidence.

The probe targets the installed PX4 v1.17/x500 combination. Its actuator stream
sends `act.noutputs` in the `active` field, so this version requires an output
count of 4–32 rather than interpreting it as a bitmask. The source assignment
and an earlier rejected observer record are retained in `observer-diagnostics/`.
