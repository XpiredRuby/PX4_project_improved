# Engineering audit — 25 October 2026

This increment starts from PR #6, `72f7b03c26e4ca20e2d45a19f272e5b1d9c4961b`.
It provides offline verification and an experimental native adapter. It does not
qualify the new family for reliable connected SITL or physical flight.

## As-is audit

| Component | Finding at takeover | Action / retained boundary |
|---|---|---|
| Arbitrary finite sketches | Already implemented: ordered simplification, envelope repair, stop/turn, bounded scaling | Preserved; repeated native corpus includes 10,000 points, crossings, reversals, Vin and hover |
| Preferred speed | Hidden 1.25 m/s user cap overrode higher requests | Fresh demo cap is 3 m/s, visible beside cruise; explicit lower caps and restored drafts remain binding, with a warning |
| Aircraft model | Simplified x500 ceiling 3 m/s, acceleration 1 m/s², jerk 1.5 m/s³ | Unchanged; 5 m/s request remains clipped and disclosed |
| Native speed policy | `cruise_reference_speed_m_s=1.0`, `cruise_command_speed_limit_m_s=1.25` | Intentionally preserved; `_motion_speed_scale` slows the phase clock and velocity feedforward. A faster browser plan is not evidence of faster native tracking |
| Native constructive execution | Studio refuses stop/turn certificates; no native polynomial adapter | Added explicit experimental CLI adapter with full artifact audits; ordinary studio guard stays closed pending empirical qualification |
| Native interpolation | Separate linear interpolation of position and derivatives | Added opt-in septic Hermite reconstruction for audited constructive artifacts only; legacy families retain their behavior |
| Batch artifact staging | Runner stage copied CSV and summary, without constructive certificate sidecars | Experimental staging copies and re-audits all bound sidecars; regression exercises the real batch path |
| Truth/reference comparison | Expected linear interpolation, inconsistent with the new native C3 reconstruction | Constructive assessment uses independent endpoint-equation phase evaluation; legacy comparison stays unchanged |
| Named stops / dwell | Existing `trajectory_generator/plan_waypoint_mission.py` supports names and `hold_s` | Preserved and covered by existing five-stop tests; drawing-editor stops still have no explicit dwell UI |
| Visualization | Already full-resolution CSV playback, 1x time, compass, stationary yaw, 3D/height | Preserved; added explicit ready/playing/paused/complete labels |
| Browser cancellation | Worker timeout existed, user cancellation absent; stale worker errors could reset a later worker | AbortSignal terminates active computation; old worker callbacks cannot affect a retry |
| Planner states | Busy/error/ready inferred from DOM | Explicit start, engine preparation, checking, stale edit, adjusted, verified, cancelled, error, draft states; elapsed time without invented percentage |
| PR #5 truth audit | Already incorporated into #6 | No blind merge. #6 additionally hemisphere-aligns quaternions and refuses missing interpolation brackets/gaps |
| Genuine simulator evidence | Historical compressed telemetry retained; latest family never flown in this checkout | Re-audit historical recording separately; new connected launch is retained as FAILED before simulator start |
| Public hosting | Existing URL supplied; no Site identity in Git checkout and no owned/shared editable Site returned by connected Sites account | Local build/browser tested; existing production site cannot be updated from this account |
| Hardware | Not qualified | No hardware transport or physical-flight changes |

## Mathematical and engineering changes

The planner's geometry and dynamic limits are unchanged. New speed diagnostics
use every exported route sample to distinguish requested cruise/cap, effective
intent, conservative planning cap, sampled peak and trapezoidal sampled mean.
A peak from sampled data is not a continuous upper bound and is labeled accordingly.

The constructive adapter independently solves the eight endpoint equations for
a seventh-order scalar rest-to-rest polynomial: position 0/1 and zero first,
second and third derivatives at both ends. Numerical critical-root evaluation
checks speed, acceleration, jerk and yaw derivative extrema, diversifying the
existing continuous Bernstein checker. The existing physical checker is also
rerun for tilt, thrust, body rate, rotor authority and operating volume. Shared
physical primitives remain shared; this is not formal independent organizational V&V.

Every CSV state is reconstructed from phase poses and duration. Every phase
boundary must be an exact CSV sample, clocks must increase with at most 0.05 s
spacing, all fields must be finite, and effective vertices/laps must be visited
in order. Canonical hashes bind intent, models, candidate phases and certificate;
CSV SHA-256 binds exact serialized motion. Rehashing a corrupt derivative does
not evade reconstruction. Full ground-to-ground CSVs are refused by the native
adapter. Native takeoff, return, PX4 landing and shutdown keep their ownership.

The experimental execution path currently requires fixed heading and constant
verified cruise altitude. Offline diverse audits also cover changing-yaw
constructive references. A flag does not declare qualification: acknowledgement,
model/profile binding, launch-height checks, navigation health, cancellation,
landing, disarm and propulsion evidence remain required.

## Reproduce

```bash
python scripts/validate.py
python -m ruff check controller mission_planner analysis scripts tests trajectory_generator
node --test tests/*.test.cjs
python scripts/check_conference_planner.py --output /absolute/new/repeated-plans --repeats 2
python scripts/check_trajectory_acceptance.py --output /absolute/new/strict-acceptance
python web/build_public_studio.py dist
```

On a machine with the compiled x500 PX4/Gazebo environment, the new experimental
case constructs five named stops, audits the exact route, issues the existing
bounded harness acknowledgement, and uses native lifecycle ownership:

```bash
python scripts/planner_sitl_campaign.py --confirm-sitl --experimental-stop-turn \
  --autopilot /home/xpire/PX4-Autopilot --output /absolute/new/constructive-sitl \
  --cases stop-turn-nominal
```

For an already retained, fixed-heading constructive plan, use the same flag
with `--reviewed-plan /absolute/retained/plan --cases reviewed-plan`.
An unexpired acknowledgement must already match that artifact. This never
recompiles the confirmed plan. The ordinary browser Start path still refuses
this experimental family.

## Evidence and remaining qualification

See `validation/2026-10-25-conference/README.md` for executed results and failures.
No new Gazebo trajectory or high-speed tracking claim is made. Before promoting
the adapter, run the new five-stop family and fixed-heading handwriting, native
cancellation and failure recovery campaigns on an identified simulator; retain
truth velocity/acceleration/quaternion and actuator evidence; compare the exact
reference and actual run with clock and telemetry-coverage gates. Then qualify
changing-yaw launch alignment and decide whether to revise native speed policy.
Legacy native CSV families still use linear interpolation between exported rows;
this increment makes no new general between-row C3 execution claim for them.
Hosted Gazebo, arbitrary obstacles, global closest-path/time optimality and
physical flight remain outside this validated increment.
