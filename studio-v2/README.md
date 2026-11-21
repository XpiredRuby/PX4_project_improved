# PX4 Trajectory Studio

Use the localhost workbench to draw a route, calculate a time-parameterized reference, compare requested speeds, and replay recorded flights. The default vehicle is the X500 reference model. The aircraft selector also includes the repository's Gazebo X500 SDF profile and three illustrative multicopter profiles. Selecting a different model sends its assessed limits to the planner and invalidates the old result.

## Run locally

From the repository root, with Python 3.10+ and Node 22.13+ installed, run:

```bash
python -m pip install -r requirements.txt
npm --prefix studio-v2 ci
npm --prefix studio-v2 run build
python -m mission_planner.research_studio
```

Open **http://127.0.0.1:8767**. Once dependencies are installed, local planning, scenario comparison and flight replay work offline. Place search and hourly forecasts use the internet. The local server binds to loopback and starts with the simulator disconnected.

## First experiment

1. Draw a freehand path or edit the waypoint table. Mark an interior point as a stop or an exact fly-through point as appropriate.
2. Enter a preferred cruise speed and a separate never-exceed speed. Any finite positive speed request is accepted as input. **Plan & verify** reports the requested speed, model and user limits, verified route peak, route duration and adjustments. The reference planner applies tracking margin and time scaling. The result is a feasible profile under its model assumptions. Global minimum time and physical flight accuracy require separate evidence.
3. In **Results & Analysis**, run the same route at several requested speeds. Export the sweep to retain the original inputs, result digest, speed, duration and limiting reasons for each case. Use **Compare steady winds** to compare airspeed at the same route velocities under calm and four horizontal wind directions.
4. Save the editable project for repeat runs. **Report JSON** exports inputs, profile, solver-source digest, model and verification results, route identity, and the weather snapshot. **Route CSV** exports the exact verified route reference.
5. To inspect flight evidence, select **Load recorded x500 flight** or import a recording. Replay follows the recording's own timeline and setpoints. Use **Mission comparison** for a recording with a common origin, clock and a matching route.

### Unattended studies

**200 m · five stops** loads a closed 200 m X500 study with five interior full stops and a preferred cruise of 50 m/s. **Queue study** freezes the current project, requested speeds, aircraft and weather. You can edit the drawing, queue another weather scenario, close the tab and return to the batch. Keep the native server running for work to continue. Restarting it recovers unfinished work from SQLite. Use `--studies-dir PATH` to select a saved collection. One studio owns each collection at a time.

**Download case** contains the exact route CSV, compiler request and result, case metrics and Python, NumPy and compiler versions. **Export study** contains the original project and a row for each case, including failures and attempt counts. Retry acts on failed, cancelled or timed-out cases. If the compiler changes, restore the scenario and start a new study to retain separate provenance.

### Mission comparison

The report measures 3D tracking error against the selected route, elapsed time on that phase, ordered stops and horizontal touchdown error. The route clock aligns positions to path progress. The elapsed clock measures how long the maneuver took. A stop requires position and speed to satisfy their respective limits at the same observation. The default thresholds are 0.5 m tracking RMS, 10% elapsed-time error, and 0.5 m at 0.25 m/s within one mission-clock second of each planned stop. The landing band defaults to a 5 m radius in the local horizontal plane. The report records the chosen criteria.

The retained X500 example uses its original 90.06 s route and 2,539 route-phase truth samples. It yields 0.1601 m tracking RMS, 126.90 s elapsed on the route and 0.0783 m horizontal error at the first on-ground indication during native landing. The longer elapsed time exceeds the 10% study threshold. The target is the last recorded horizontal landing setpoint. This historical manifest identifies the archived route input and lacks a controller runtime receipt. Interior stop declarations are unavailable for this run.

To reopen a case on another computer, choose **Use a saved study case** under Mission comparison. This retains the exact native CSV even when the browser compiler produces small floating-point differences. For an imported run, select the aligned telemetry CSV (or CSV.gz) and its JSON receipt. Use NED metres in `truth_x/y/z` and `desired_x/y/z`, elapsed seconds in `elapsed_s`, a `phase` column and a route clock in `mission_time_s` or `phase_clock_s`. A `segment` change marks a coordinate reset. Landing uses the transition to `landed_state=1` inside `PX4_LAND` or `PX4_FAILSAFE`, with a recorded `native_landing_reference_x/y` or a declared `landing_target_local_ne_m` pair. The latter uses absolute local N/E metres.

The receipt identifies the source (`retained_sitl`, `hardware` or `synthetic`), `run_id`, `controller_runtime.trajectory_sha256` and `telemetry_sha256` of the uncompressed CSV bytes. Digests preserve line endings. When the existing rehearsal assessment has matching plan, selected file and controller digests, it writes a `studio-recording-*.zip` containing both import files. Recording origin and independent truth remain responsibilities of the data producer. Missing evidence remains visible. **Test report with synthetic run** injects 0.100 m of position error to exercise the active plan. **Test simulator handoff offline** exercises the request and receipt protocol with a synthetic response.

### Weather

Manual scenarios accept wind speed, meteorological direction *from*, gust, temperature, pressure and optional relative humidity. Forecast mode uses a selected coordinate and UTC time, retrieves hourly Open-Meteo data and retains the forecast-valid hour separately from retrieval time. Changing site or time clears the active forecast. Air-relative speed uses ground velocity minus the assumed wind vector. **Predict tracking** uses the selected model's mass, thrust and tilt limits, a critically damped position response, and an assumed isotropic drag area. It advances a 3D point mass from the first route state, uses uniform horizontal wind, and exports the predicted positions and deviation. The initial 0.08 m² drag area and 0.8 s response time are editable assumptions. This model does not reproduce the PX4 controller, estimator or landing contacts. Dynamic pressure and aerodynamic drag use a dry-air density estimate. Humidity and gust are recorded descriptors. No aerodynamic tracking or touchdown response to weather has been validated in this milestone.

### Recorded data and public benchmark

| Input | Interpretation |
| --- | --- |
| Bundled X500 SITL recording | Independent simulator truth and its own recorded reference. The bundled raised-landing-surface run provides 3,508 paired positions and a 3D tracking RMS of 0.143 m. |
| Aligned CSV or CSV.gz | Seconds in `elapsed_s`, NED metres in `truth_x/y/z` or `estimate_x/y/z`, optional `desired_x/y/z`. Missing or nonfinite position channels are omitted and reference gaps are reported. |
| NanoBench CSV | Seconds in `t`, Vicon XYZ metres in `px,py,pz`, and command position in `sp_ctrltarget_x/y/z`. Original world axes are preserved. |
| PX4 ULog on localhost | Uses valid local estimator positions and the most recently published setpoint at or before sample time, up to 250 ms old. Coordinate resets split the recording. Estimates without an eligible reference remain visible with unscored tracking error. |

The benchmark tab contains a separate closed-loop prediction experiment on **Crazyflie 2.1**. Two medium circle flights fit the single time constant of a first-order position response. The predictor starts once from measured initial position and advances using recorded position setpoints. On three held-out flights, the model's 3D RMS position errors were 0.259 m (medium figure eight), 0.201 m (slow Lissajous), and 0.200 m (fast Lissajous). The corresponding setpoint-versus-Vicon errors were 0.413, 0.323 and 0.506 m. This is evidence for those flights and the low-order predictor. X500 physical prediction accuracy is still unmeasured.

To reproduce the benchmark, check out [NanoBench revision 934a1ab](https://github.com/syediu/nanobench/tree/934a1ab92c458cad99c9278a5cb63bf68af6e56c), then run:

```bash
python scripts/benchmark_recorded_tracking.py /path/to/nanobench/datasets/dataset
```

The [benchmark evidence](benchmarks/nanobench.json) includes input SHA-256 digests, whole-flight splits, coverage, per-axis bias, RMS, p95 and peak errors. Dataset files stay with the original publisher.

## Validation and static packaging

```bash
npm --prefix studio-v2 test
python -m unittest discover -s tests
python -m pip install playwright
python -m playwright install chromium
python scripts/check_research_studio_browser.py
```

The browser check runs the actual native planner, uses a deterministic forecast response, tests exports and imports, replays the retained flight, changes aircraft and verifies a freehand drawing. Supply `--csv PATH` and `--ulog PATH` to exercise external NanoBench and ULog inputs. To produce the separate portable static build, run `python web/build_studio_v2.py` after building React, then serve `studio-v2/preview/` at the root of a static origin. By default, static mode retrieves the Pyodide runtime on first use. For disconnected static planning, package with `python web/build_studio_v2.py --bundle-runtime` while online to include Python and NumPy in the preview directory. Both static modes accept CSV replay. Binary ULog conversion and unattended studies use the native localhost server. To create the downloadable package with both launch modes and the worked example, run:

```bash
python web/build_studio_v2.py --bundle-runtime --archive PX4_Trajectory_Studio_Offline.zip
```

Extract the ZIP and run `python start_studio.py` in its root. This needs Python 3.10+ and NumPy. Install pyulog for binary ULog import. Browser-only planning also works with `python -m http.server 8767 --bind 127.0.0.1`.

[Open-Meteo forecast API](https://open-meteo.com/en/docs) · [PX4 ULog parser](https://github.com/PX4/pyulog)
