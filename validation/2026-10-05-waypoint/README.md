# October 5 five-stop waypoint evidence

This directory freezes the compact result of the first live five-stop mission
for the strict `LOCAL_NED` planner. The run used PX4 v1.17.0 at
`d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, Gazebo 8.15.0, and the x500 model
in the `afvl_validation` world. Simulator truth was used only after flight.
No physical aircraft was flown.

## Result

- The strict JSON plan compiled deterministically into 2,520 points spanning
  125.899 s. Its source and generated CSV hashes matched the published candidate.
- The mission completed `TAKEOFF -> TRAJECTORY -> RETURN_HOME -> ALIGN ->
  HANDOFF -> PX4_LAND` with outcome `SUCCESS` and no scenario errors.
- All five named stops had stationary-reference evidence and passed unchanged
  0.75 m position-p95 and 0.4 m/s speed-p95 limits. The worst observed values
  were 0.044 m and 0.019 m/s.
- Estimated trajectory tracking was 0.109 m XY p95 and 0.016 m Z p95.
  Independent simulator truth measured 0.194 m XY p95, 0.267 m XY maximum,
  0.044 m Z p95, and 0.051 m Z maximum.
- The onboard touchdown audit measured 0.049 m XY error. Independent truth
  measured 0.062 m. PX4 confirmed contact, automatic disarm, and fresh zero
  propulsion; the wrapper then stopped only its owned simulator process group.
- Ruff, 223 regression tests, five connector subprocess tests, deterministic
  generation, and both GitHub validation workflows passed. Planner/auditor
  branch-aware coverage is 91% combined.

`results.json` contains the thresholds, exact metrics, source identifiers, and
raw-artifact hashes. The selected trajectory is bound to both the run manifest
and the evidence by SHA-256. The full controller CSV, aligned truth trace, and
ULog remain on the validation computer because of their size.

## Score scope

The evidence supports a **95/100 SITL research-software assessment**: strict
contracts, analytic C3 planning, independent pre-arm validation, deterministic
tests, live route execution, independent truth, and safe shutdown all have
direct evidence. The remaining five points represent missing physical-airframe
validation, obstacle sensing/avoidance, and a waypoint-specific fault matrix.

This score is engineering judgment for the stated software/SITL scope. It is
not a reliability probability or a physical-flight-readiness claim. The
recorded gust plus twelve-second GNSS-outage failure remains unresolved and is
not hidden by this nominal waypoint pass.
