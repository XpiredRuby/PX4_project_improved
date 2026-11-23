# Full 200 m X500 Gazebo benchmark

The cases use a 5 m/s preferred cruise, a 6.5 m/s horizontal cap, a level 12 m route, and fixed NED heading. The requested path includes a full stop at each vertex. The declared flight-time allowance is 337.5 s after setting aside the battery reserve. The planner keeps all requested vertices and certifies speed, acceleration, jerk, tilt, and thrust using continuous bounds.

On level fixed-heading legs, the planner can use two mirrored polynomial ramps. Their shared midpoint has continuous position, velocity, acceleration, and jerk. The controller CSV includes the exact midpoint so interpolation stays within each polynomial. The adapter independently checks the polynomial and midpoint sample before execution. The adapter also verifies archived plans with the original septic polynomial.

## Verification

Each run is judged against its own unchanged 200 m reference and the same registered aircraft limits. Measured travel integrates Gazebo's horizontal speed on the route phase. Tracking RMS uses the paired 3D positions. An observed interior stop requires a ground-truth sample within 0.5 m of the requested point at no more than 0.25 m/s, within one mission-clock second of the stop. Touchdown uses the first PX4 on-ground sample during AUTO LAND.

| Route | Measured travel | Peak speed | Continuous time near 5 m/s | 3D tracking RMS | Touchdown error |
| --- | ---: | ---: | ---: | ---: | ---: |
| [Out-and-back](out-and-back-result.json) | 199.87 m | 4.90 m/s | 6.63 s | 12.5 cm | 4.5 cm |
| [Square](square-result.json) | 200.29 m | 4.88 m/s | 2.83 s | 15.5 cm | 1.1 cm |
| [Triangle](triangle-result.json) | 199.86 m | 4.95 m/s | 4.63 s | 10.3 cm | 6.6 cm |

“Near 5 m/s” uses a ±0.25 m/s interval. All three runs passed all 22 checks, and all seven interior stops met the stated position and speed criteria. The slow approaches averaged 0.096–0.102 m/s. PX4 AUTO LAND commanded at most 0.600 m/s during the audited descent, followed by automatic disarm and two seconds of zero outputs. The triangle was imported, planned, launched, and reviewed in Chromium. The status and evidence survived reloading the page during the run.

The documented install was repeated in a clean Linux environment with a fresh Python virtual environment and the npm lockfile. The production build served the Studio, and the native preview API reproduced the full 200 m route with matching CSV and report hashes. All 454 tests in the clean checkout and all 32 Studio frontend tests passed. The live flights used the laptop's PX4/Gazebo installation.

## Reproduce in Trajectory Studio

Build the app following the [local installation instructions](../../studio-v2/README.md). From the repository root, register your compiled local PX4 Gazebo X500. The binding records the model and airframe hashes.

```bash
python -m mission_planner.simulation_binding \
  --autopilot ~/PX4-Autopilot --model x500 --airframe-id 4001 \
  --profile validation/2026-11-22-full-200m/x500-cruise-profile.json \
  --output ~/px4-x500-cruise-binding.json

python -m mission_planner.research_studio \
  --port 18768 --sitl-autopilot ~/PX4-Autopilot \
  --runs ~/PX4-workbench-bound-runs \
  --sim-binding ~/px4-x500-cruise-binding.json
```

Use **Open project** to load the [out-and-back](out-and-back.px4-project.json), [square](square.px4-project.json), or [triangle](triangle.px4-project.json). Select **Check route** and check that the route distance is 200 m. Select **Run verified route in Gazebo**. Expand **View simulator checks** when the run finishes.
