# PX4 Trajectory Studio handbook

Trajectory Studio turns a drawing or ordered waypoints into a checked trajectory, lets you inspect the reference, and can run that exact route in a local PX4/Gazebo X500 simulation. The current engineering baseline is the **registered X500 in the flat AFVL validation world**. Three distinct 200 m routes passed the full simulation assessment with a 5 m/s cruise request.

| If you want to… | Start here |
| --- | --- |
| Draw, plan, compare speeds, or replay a flight | [Install and open Studio](#1-install-and-open-studio) |
| Reproduce the current X500 benchmark | [Prepare PX4 and Gazebo](#3-prepare-px4-and-gazebo) |
| Judge whether the simulator actually passed | [Read and preserve the evidence](#5-read-and-preserve-the-evidence) |
| Change the code and check a regression | [Find and test the implementation](#7-find-and-test-the-implementation) |
| Review what we built and where it stands | [Development record](#8-development-record) |

## 1. Install and open Studio

Use a Linux shell. The connected setup was exercised under Ubuntu 22.04 in Windows WSL2. Keep the source, virtual environment, and simulator build in the Linux filesystem when using WSL. Studio needs Git, Python 3.10 or newer with `venv`, and Node.js 22.13 or newer with npm. The install and frontend build require access to their dependency registries.

```bash
git clone --branch feature/offline-research-studio https://github.com/XpiredRuby/PX4_project_improved.git
cd PX4_project_improved
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
npm --prefix studio-v2 ci
npm --prefix studio-v2 run build
python -m mission_planner.research_studio
```

Open [Studio on port 8767](http://127.0.0.1:8767) on the same computer. This starts the native planner, background studies, and replay without a simulator. Keep the terminal running while using the app. The server binds to loopback. Place search and forecast retrieval require network access. Manual weather, the native planner, study execution, and local CSV replay can run offline after installation.

For a portable web build with the bundled Python runtime, follow the [static packaging instructions](../studio-v2/README.md#validation-and-static-packaging). The static build handles browser-side planning and CSV replay. ULog conversion, the local study queue, and Gazebo execution use the native server.

## 2. Plan a route and inspect the reference

Studio opens on **Start**. Choose **Try a practice flight**, **Start a blank route**, or **Open saved route**. The desktop workspace stays within the browser window. **Build route** keeps the map above the waypoint table. Use the table's arrows to reach later points. **Preview flight** keeps playback below the 3D view. **Review results** opens on **Checks** after verification. Charts and the simulator have their own tabs. Choose **More tools** for studies, recordings, and comparisons. Detailed logs and long forms scroll inside their own panes. On narrow screens, the toolbar's **Project** and **Settings** buttons open those panes.

The map uses north-up local coordinates in metres. The solid ivory line is the draft route, diamonds mark required stops, and the dashed line marks the aircraft's operating radius. The active waypoint has an amber ring. The editor shows draft length, height above launch, and waypoint count. **Fit route** brings the route back into view. In **Preview flight**, use **Top view** for a north-up plan view or **Orbit** for the default 3D angle. The **Help** button and contextual question marks explain the controls. Escape closes Help.

1. Draw freehand or add waypoints in order. You can use **Open project** for a saved `.px4-project.json` or import waypoint CSV. Use east and north in metres, and enter positive altitude above launch. Internal position and yaw use local NED. Positive down is below launch. A fixed yaw of 0° points north and 90° points east.
2. Mark required stops and set **Maximum geometric deviation**. Set the aircraft model, **Preferred cruise**, and **Never exceed**. Preferred cruise guides retiming. The aircraft limit and the user ceiling still bind. Acceleration, jerk, stops, and planning margin affect the speed you see.
3. Select **Check route**. Review any adjustments and choose **Watch flight preview** to start playback. The planner can smooth, simplify, project, retime, or shorten a difficult request. Confirm the route shown against your requirement before using it.
4. In playback, switch between the reference route and the full planned mission. The route CSV is the controller input. The full-mission CSV is a reference visualization of takeoff, route, return, and landing. Native PX4 execution supplies its own takeoff, return, and landing behavior.
5. Save the editable project, **Route CSV**, and **Report JSON** together. For a speed comparison, run a sweep or use **Queue study** to retain an immutable snapshot. **Download case** exports its exact trajectory and solver inputs. **Export study** exports the cases, including failures and timing. Return to the same native study directory to recover a queue after restarting Studio.

Weather plots use steady wind and an optional point-mass response with editable drag and response-time assumptions. A weather sensitivity result describes that model. Use the Gazebo check and ground-truth logs to evaluate a simulator run. See [Studio's controls and recording formats](../studio-v2/README.md).

## 3. Prepare PX4 and Gazebo

For the reference setup use PX4 v1.17.0 at `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, Gazebo Sim 8.15.0, and the X500 airframe ID `4001`. Install the PX4 build and simulator prerequisites and run the upstream `make px4_sitl gz_x500` smoke test. Stop that smoke test before starting Studio. A usable existing build contains `build/px4_sitl_default/bin/px4` and `build/px4_sitl_default/rootfs/gz_env.sh`. [PX4 Ubuntu setup](https://docs.px4.io/v1.17/en/dev_setup/dev_env_linux_ubuntu) · [PX4 Gazebo X500 target](https://docs.px4.io/v1.17/en/sim_gazebo_gz/vehicles)

From the Studio repository root, set the firmware path and copy the archived validation world into the PX4 simulation tree. The campaign compares this world's bytes with the checked-in world before enabling the flat-world approach settings. If there is an existing `afvl_validation.sdf`, compare it and preserve your version before replacing it.

```bash
PX4_TREE="$HOME/PX4-Autopilot"
WORLD=validation/2026-10-03/worlds/afvl_validation.sdf
cp -n "$WORLD" "$PX4_TREE/Tools/simulation/gz/worlds/"
cmp "$WORLD" "$PX4_TREE/Tools/simulation/gz/worlds/afvl_validation.sdf"

python -m mission_planner.simulation_binding \
  --autopilot "$PX4_TREE" --model x500 --airframe-id 4001 \
  --profile validation/2026-11-22-full-200m/x500-cruise-profile.json \
  --output "$HOME/px4-x500-cruise-binding.json"
```

The binding records the profile, resolved model and airframe, and their content hashes. Generate a fresh binding for each PX4 checkout. A modified simulator model may fail profile verification. This profile declares a 6.5 m/s horizontal limit and the registered test's power, reserve, and flight-time allowances. The runner applies its operating volume of at most 80 m horizontal radius and 30 m above launch.

## 4. Run the current benchmark

In the Studio repository with the virtual environment active, launch the connected workbench. Use a different study directory if another Studio instance is open.

```bash
python -m mission_planner.research_studio \
  --port 18768 \
  --studies-dir "$HOME/PX4-workbench-connected-studies" \
  --runs "$HOME/PX4-workbench-connected-runs" \
  --sitl-autopilot "$PX4_TREE" \
  --sim-binding "$HOME/px4-x500-cruise-binding.json"
```

1. Open [the connected workbench](http://127.0.0.1:18768). Use **Open project** to load the [out-and-back](../validation/2026-11-22-full-200m/out-and-back.px4-project.json), [square](../validation/2026-11-22-full-200m/square.px4-project.json), or [triangle](../validation/2026-11-22-full-200m/triangle.px4-project.json). These projects carry the matching registered profile. For your own draft, select **Registered Gazebo aircraft**.
2. Select **Check route**. Check for a 200 m accepted route, 12 m altitude, a 5 m/s preferred speed, and the displayed fixed heading. The out-and-back, square, and triangle use 45°, 0°, and 90° respectively. Each stops at its interior vertices.
3. Open **Simulator**, then select **Run verified route in Gazebo**. The app sends the retained plan identifier and exact CSV digest. The runner owns one headless local SITL instance at a time. Keep the Studio server open. Refreshing the same tab resumes monitoring. Select **Simulator run** in the command bar to reopen its status. **Request landing** asks the active rehearsal to land. Wait for its reported ground and disarm state.
4. Open **View simulator checks** after completion. Look for **Verified rehearsal passed** and inspect the measurements in the assessment. The benchmark sequence is takeoff, route, return, settle, a nominal 0.1 m/s slow approach, and PX4 AUTO LAND. PX4's logged downward command was limited to 0.6 m/s during the audited native descent. Measured vertical velocity differs slightly from the setpoint.

The connected constructive route uses fixed heading and constant altitude and begins over launch. Importing an arbitrary drawing can yield a valid preview without a supported simulator execution plan. The legacy editor and low-level research CLI have separate launch rules. Keep the reproduction above as the first smoke test before changing route type, altitude, airframe, or world.

## 5. Read and preserve the evidence

| Artifact | What it tells you |
| --- | --- |
| Editable project and accepted `intent.json` | What was requested and what the planner accepted. Read the adjustment report before comparing distances. |
| `trajectory.csv`, `summary.json`, `verification.json` | Exact setpoints and clocks, model/intent identifiers, continuous reference checks, and file digests. |
| `worker.log`, `rehearsal/status.json` | Launch progress and the reason if the campaign refused a plan or failed. |
| `rehearsal/reviewed-plan/simulation_configuration.json` and `effective_parameters.json` | World and aircraft configuration, plus the PX4 parameters read back before flight. |
| `rehearsal/reviewed-plan/assessment.json` and `final_ground.json` | Sampled independent truth, tracking, heading, approach rate, first on-ground position, and two seconds of disarmed zero actuator outputs. |
| `batch/reviewed-plan/research_log_*.csv`, runner manifest, and `runtime/**/*.ulg` within the reviewed run | Original controller and simulator sources for diagnosing or reproducing the assessment. |
| `studio-recording-*.zip` within a completed reviewed run | Aligned telemetry and receipt for **Mission comparison**. Unzip, then import its CSV and manifest with the corresponding route selected. |

These paths start inside `<runs>/<plan_id>/`. Retain the full run directory, the matching source revision, the editable project, and the PX4/model/world versions when sharing a result. GitHub has the benchmark configurations, machine-readable summaries, and hashes. The full raw simulator logs remain in the run directories on the simulator machine. The public repository can reproduce the experiments with a configured environment.

To reassess a complete retained run locally, run:

```bash
python analysis/planner_rehearsal.py "$HOME/PX4-workbench-connected-runs/<plan_id>/rehearsal/reviewed-plan"
```

Use a copy when preserving an original assessment. This command writes a new `assessment.json` in the case folder. Keep failed attempts and record software changes alongside each repeat.

### Current evidence

| Campaign | Result |
| --- | --- |
| [Full 200 m X500 tests](../validation/2026-11-22-full-200m/README.md) | Out-and-back, square, and triangle each passed 22 checks. Measured horizontal travel was 199.86–200.29 m, peak route speed 4.88–4.95 m/s, tracking RMS 0.103–0.155 m, and touchdown error 0.011–0.066 m. All seven observed interior stops passed. The triangle also passed browser import, launch, reload, and review. |
| [Slow approach and AUTO LAND](../validation/2026-11-22-native-land/README.md) | The earlier shortened route verified the approach before the full 200 m runs. It exposed PX4's higher descent demand above its landing threshold. The repeat bounded both native descent commands and retained the measured speed separately. |
| [Five-stop tracking campaign](../validation/2026-11-21-five-stops/README.md) | One of the initial trials narrowly exceeded the declared tracking threshold. Both trials with velocity lead passed using the same reference and thresholds. |
| [Earlier connected and fault tests](trajectory_studio_connected_validation.md) | Historical evidence for launch/heading recovery, exact artifact handoff, and bounded simulator fault responses. Evidence applies to its recorded builds and scenario conditions. |
| [Clean install and unit tests](../validation/2026-11-22-full-200m/README.md#verification) | A fresh Linux install built the frontend, served the app, verified the API's 200 m reference, and passed 454 Python tests and 32 Studio frontend tests. Live Gazebo was verified on the separate reference laptop. |

The published simulator results cover the documented X500 configuration, calm flat world, and sampled telemetry. Independent contact time at the rigid-body level, operation in wind and unknown terrain, other configured aircraft, and physical qualification remain separate experiments. The 5 m/s value is a preference. The tables give actual peak and sustained-speed measurements.

## 6. Troubleshoot

| Symptom | What to inspect |
| --- | --- |
| Studio says the frontend is missing | Run the npm install and build from step 1. Start the server with the same checkout. |
| The port or study directory is busy | Stop the previous idle Studio instance or specify a free port and a separate study directory. SQLite allows one owner for each study directory. |
| Registered aircraft option is absent | Confirm all connected server flags. Open `/api/capabilities` on the local server and check that `sitl_enabled` and `registered_aircraft` are true. |
| Binding changed or model does not match | Compare the configured PX4 tree, its model and the intended profile. Generate a new binding if the inspected files are the intended versions. |
| The planner accepts less than 200 m | Check the displayed aircraft, requested route, stops, and budget. Load a versioned benchmark project to reproduce the supported case. |
| Simulator button is disabled | Plan again after edits. Confirm the profile, fixed heading, altitude, launch origin, and the compatibility message. A completed plan needs a new verification before another rehearsal. |
| Launch fails or a simulator is already owned | Read `worker.log` and the run status. Stop the separate PX4 test you started or wait for the current campaign to finish. |
| Landing or a check fails | Read the named failed check, effective parameters, and final ground evidence. Preserve the run. Use the recorded data to diagnose before retrying. |
| Windows browser cannot reach WSL localhost | First verify that `curl http://127.0.0.1:18768/api/capabilities` works in the same WSL distribution. Then check Windows' localhost forwarding. Keep the application on loopback. |

## 7. Find and test the implementation

| Area | Entry points |
| --- | --- |
| React interface, project encoding, playback, browser handoff | [studio-v2/src](../studio-v2/src) |
| Native HTTP server, retained jobs, study queue | [research_studio.py](../mission_planner/research_studio.py), [studio_jobs.py](../mission_planner/studio_jobs.py), [study_queue.py](../mission_planner/study_queue.py) |
| Intent, constraints, closest-feasible geometry and timing | [contracts.py](../mission_planner/contracts.py), [preview.py](../mission_planner/preview.py), [closest.py](../mission_planner/closest.py), [lifecycle.py](../mission_planner/lifecycle.py) |
| Independent certificate and exact controller CSV | [lifecycle_verification.py](../mission_planner/lifecycle_verification.py), [stop_turn_execution.py](../mission_planner/stop_turn_execution.py) |
| Model binding, simulator campaign, native control | [simulation_binding.py](../mission_planner/simulation_binding.py), [planner_sitl_campaign.py](../scripts/planner_sitl_campaign.py), [offboard_runner.py](../controller/offboard_runner.py) |
| Measurement and truth cross-checks | [planner_rehearsal.py](../analysis/planner_rehearsal.py), [sitl_ground_truth.py](../analysis/sitl_ground_truth.py), [simulation_physics.py](../analysis/simulation_physics.py) |

Run the automated checks from the repository root:

```bash
python -m unittest discover -s tests
npm --prefix studio-v2 test
npm --prefix studio-v2 run build
```

The optional [Chromium smoke suite](../studio-v2/README.md#validation-and-static-packaging) checks the native planner UI, downloads, and playback. Repeat one of the [full-distance projects](../validation/2026-11-22-full-200m/README.md) for a controlled simulator regression. Compare the exact CSV SHA-256, waypoint order, requested versus accepted distance, tracking, and landing checks. Expect new plan identifiers and telemetry hashes on new runs.

For the older named-waypoint CLI with timed holds, see [Waypoint missions](../README.md#waypoint-missions). For detailed models and history, see [aircraft profiles](aircraft_profiles.md), [requirements traceability](trajectory_planner_traceability.md), [planner architecture](trajectory_planner_architecture.md), and [validation summary](validation_summary.md).

## 8. Development record

- **Controller and ground-truth audits.** Added smooth GPS/IMU takeoff, trajectory tracking, return and landing handoff, timestamp and navigation checks, guarded failsafe response, and separate simulator-truth assessment. [Controller logic](code-logic.md)
- **Verified trajectory planner.** Added versioned intent, path geometry and ordered fidelity checks, physical and actuator limits, heading schedules, stop boundaries, and independently checked CSV output. [Planner architecture](trajectory_planner_architecture.md) · [October engineering audit](conference_engineering_audit.md)
- **Engineering workbench.** Added freehand and waypoint editing, three-dimensional playback, adjustable aircraft and requested speeds, weather analysis, saved studies, export/import, and measured flight replay. [Studio features and formats](../studio-v2/README.md)
- **Connected execution.** Bound the exact retained CSV to the simulation request. Enabled fixed-heading level stop/turn routes in the connected React workbench. Added an isolated PX4/Gazebo worker, phase status, landing requests, reload recovery, and assessment exports. [Five-stop benchmark](../validation/2026-11-21-five-stops/README.md)
- **Registered X500 tuning and landing.** Verified simulator model identity, retained the stock allocator, calibrated GNSS latency for the simulator, aligned the planned NED heading, and added the measured slow approach followed by native AUTO LAND. [Landing evidence](../validation/2026-11-22-native-land/README.md)
- **Full-distance retiming and repeatability.** A 200 m request previously scaled to 189.1 m to fit the declared budget. The planner now uses faster, continuous ramps for eligible level legs. The separately implemented adapter checks the exact midpoint and CSV. The three full-distance projects fit the same budget with all requested stops retained. [Final benchmark and editable projects](../validation/2026-11-22-full-200m/README.md)
