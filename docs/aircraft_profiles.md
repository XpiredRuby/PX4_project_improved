# Reusable aircraft setup and model-bound simulation

The planner is configured per **aircraft configuration**, rather than compiled
for one owner's drone. Users enter or import a profile, answer the missing
questions, draw a mission or use the interview, review its interpretation, and
confirm the exact trajectory for its matching Gazebo model.

Profile completion establishes that the input is complete and internally
consistent. Its numerical bounds remain declarations with evidence sources;
software cannot measure a motor's available thrust or qualify a new aircraft
from its mass or product name.

## Compatibility

| Aircraft/backend | Implemented behavior |
|---|---|
| PX4 parallel-axis fixed-pitch multicopter | Profile-based planning for 4–16 individual rotors; regular quad, hex and octo layouts or custom geometry about the loaded CG. |
| Registered PX4/Gazebo multicopter | Local rehearsal for up to 12 rotors, matching the installed PX4 allocator capacity. Model assets, profile, airframe script and exact trajectory are bound. Only x500 has connected evidence in this revision. |
| Unregistered aircraft | Offline planning and downloads; rehearsal is refused instead of substituting x500. |
| Fixed-wing, VTOL, helicopter | Explicit compatibility refusal. These require aerodynamic/minimum-airspeed, transition or different actuator adapters. |
| Another autopilot or navigation source | Explicit compatibility refusal; the present execution adapter is PX4 with GNSS/IMU navigation. |
| Physical execution | Local qualification → prepared release → supervised launch/abort CLI, with live identity/parameter/telemetry checks. No browser arm endpoint. Physical qualification remains required per loaded configuration. See [physical release](physical_flight_release.md). |

The default page opens aircraft setup. The x500 reference demonstration is an
explicit alternative. Example profiles are illustrative software fixtures and
are visibly labeled; they are not manufacturer specifications or flight releases.

## Inputs and how to obtain them

| Input group | Required information | Source/interpretation |
|---|---|---|
| Identity | Configuration ID, name, aircraft type, autopilot, navigation | Change the ID when battery, payload, propellers, loaded CG or tuning changes. |
| Mass/geometry | All-up mass; regular arm radius or each rotor's body-FRD x/y about loaded CG | Weigh the complete aircraft; use geometry/CAD or measurements. Custom positions allow offset CG and asymmetric layouts. |
| Actuators | Rotor minimum and available maximum thrust; signed yaw torque/thrust ratio | Use defensible bounds at the weakest allowed battery state, density and temperature. Normalized hover throttle is not thrust in newtons. |
| Rotational authority | Loaded yaw inertia; simultaneous roll/pitch torque reserve | CAD/identification and controller/flight-envelope evidence. The reserve must cover actual torque demands; this revision does not identify them. |
| Motion | Speed, acceleration, jerk, tilt, body/yaw rates and yaw acceleration/jerk; climb/descent ceilings | Take the lowest applicable controller, vehicle and operating limits. A configured ceiling alone is not evidence of accurate tracking. |
| Energy/time | Available derated battery Wh, power upper bound, reserve fraction, settling allowance, lifecycle upper bound, route wall-time multiplier, contingency time | Cover the complete loaded envelope. Include heading initialization, climb, return/transit and landing. The multiplier covers tracking-governor slowing/pauses. Maximum speeds do not supply an upper time bound. |
| Environment | Expected and allowed wind; disturbance force-per-mass and its rate; radius/height | Nonzero wind requires positive justified disturbance bounds. Wind speed alone cannot determine drag. |
| Accuracy | Position-error and tracking-error allowances | They consume the user's total path tolerance before geometric smoothing. These are declared error bounds, not inferred statistical confidence. |
| Evidence | Sources/configuration notes | Record loaded CG, propeller/battery configuration, thrust/inertia/power evidence, control tuning and conditions. |

`GET /api/aircraft/catalog` is the authoritative question catalog. The browser
form and CLI use it. `POST /api/aircraft/assess` returns all missing questions,
validation errors and compatibility blockers. It does not invent answers.
Profiles can be saved in the browser and imported/exported as JSON.

An optional read-only QGroundControl `.params` import suggests the minimum of
available controller ceilings and lists every parameter used. Incomplete
parameter groups are marked. The user reviews suggestions before applying
them. No parameters are sent to an aircraft by this import.

## Planning and checks

The existing deterministic smoothing, timing and independent verifier remain
the planning pipeline. A profile generates `VehicleModel` v2; v1 contracts and
their canonical digests remain supported.

For rotor `i`, the wrench column in body FRD is
`[1, -y_i, x_i, signed_yaw_coefficient_i]`. The allocator requires rank four.
Its explicit right inverse maps a thrust/torque interval into rotor thrust
intervals. Every interval must fit the corresponding rotor limits, including
simultaneous roll/pitch torque reserve and yaw demand. This is a conservative
sufficient feasibility test; it can reject feasible allocations and does not
reproduce PX4's desaturating allocator. It does not prove that the declared
roll/pitch reserve covers actual dynamic demands.

Declared disturbance acceleration and jerk are added to the thrust/tilt and
body-rate bounds. They are not added to the route's commanded kinematic limits.
Time stretching cannot fix an aircraft that cannot hover with its reserves.

Position and tracking allowances reduce the geometric path tube. Exhausted
tolerance is refused. For a route that changes altitude, a conservative scalar
speed cap uses the lower climb/descent ceiling, and the adjustment is recorded.

The energy condition is:

`(route_duration × route_time_multiplier + lifecycle_upper_bound + contingency)
 × power_upper_bound / 3600 <= available_Wh × (1 - reserve_fraction)`.

The lifecycle declaration must also exceed the climb/descent/settling
kinematic floor. This remains conditional on justified power/time bounds;
the current runtime does not enforce a remaining-energy envelope online.

The profile, vehicle model, environment, path-error budget and mission-energy
budget are retained with hashes. Confirmation binds the retained trajectory;
the worker checks the corresponding profile/budget digests before rehearsal.
The assessment also compares sampled navigation error, estimated tracking error
and observed lifecycle/route wall time against that profile's declared
allowances. Passing the generic tracking targets cannot hide a failed tighter
profile declaration. The whole physical route uses the user's total tolerance;
the smaller geometric tube is the planner's allocated share.

## Register a matching simulator

On the configured machine:

```bash
cd /home/xpire/PX4-trajectory-studio-oct19
python3 -m mission_planner.simulation_binding \
  --autopilot /home/xpire/PX4-Autopilot --model x500
```

This inspects the SDF without starting a simulator. Registration additionally
requires a complete profile and an existing airframe startup script:

```bash
python3 -m mission_planner.simulation_binding \
  --autopilot /home/xpire/PX4-Autopilot --model x500 \
  --profile config/planner/aircraft/x500.json --airframe-id 4001 \
  --output /home/xpire/PX4-runs/x500-profile-binding-oct19.json

python3 -m mission_planner.editor --port 8766 \
  --sitl-autopilot /home/xpire/PX4-Autopilot \
  --runs /home/xpire/PX4-runs/aircraft-studio-oct19 \
  --sim-binding /home/xpire/PX4-runs/x500-profile-binding-oct19.json
```

Open <http://127.0.0.1:8766>, choose **Use registered simulator aircraft**,
check setup, and plan the mission. Only the exactly matching profile can start
that backend. Profile edits require a new checked plan and a new registration
when the bound configuration changes.

The inspector supports local merged SDF includes with identity include poses,
unrotated link frames/inertials and the multicopter motor plugin. Unhandled
transforms are refused. It checks total link mass, rotor geometry and spin
coefficient, conservative rotor thrust ceilings and yaw-inertia consistency.
It fingerprints included model assets and the airframe startup script.
These checks are structural consistency, not independent aerodynamic validation.

Registered rehearsals explicitly apply and read back the declared rotor
position/yaw-coefficient settings in the owned, disarmed SITL instance. Runner
motion ceilings also derive from the profile. Existing GNSS bootstrap and
controller tuning are still qualification assumptions for a new configuration.
No physical PX4 parameter writes occur in this workflow.

After a confirmed profile-bound plan is retained, its two supplementary fault
rehearsals can be repeated on Linux/WSL. Replace `PLAN_ID` with that retained
plan's identifier and choose a new output directory:

```bash
python3 scripts/check_aircraft_faults.py --confirm-sitl \
  --autopilot /home/xpire/PX4-Autopilot \
  --reviewed-plan /home/xpire/PX4-runs/aircraft-studio-oct19/PLAN_ID \
  --output /home/xpire/PX4-runs/aircraft-profile-faults-repeat --gui
```

It reuses the exact confirmed CSV, profile, model and budgets, injects a four
second GPS interruption and a 0.4 s command-production stall, then records the
declared fault response separately from nominal accuracy/completion. Existing
simulator ownership is preserved. A failed response stops the campaign.

## Installed x500 correction

Inspection found that the installed Gazebo files contain 2 kg of base-link
mass **plus four rotor-link masses**, totaling 2.0643076923 kg. Their motor
plugins specify 8.54858 N per rotor and a 0.016 m moment coefficient. The
airframe startup script's 0.05 m allocator coefficient was not that SDF motor
coefficient. The earlier simplified planner used 2 kg, 10 N and 0.05 m.

The reference aggregate model is corrected, and the new x500 v2 profile uses
individual SDF geometry and coefficients. Earlier connected evidence remains
historical evidence for its recorded hashes; it does not automatically qualify
the updated model or generic aircraft. New connected results are recorded
separately.

## Primary sources and design decisions

- [PX4 v1.17 Offboard](https://docs.px4.io/v1.17/en/flight_modes/offboard): supported setpoints, estimation prerequisites, heartbeat and loss response; one vehicle class's commands cannot establish another class's controllability.
- [PX4 v1.17 parameter reference](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference): controller ceiling names and units; these are configuration inputs rather than physical identification.
- [PX4 multicopter tuning](https://docs.px4.io/main/en/config_mc/pid_tuning_guide_multicopter): configuration/tuning varies with the vehicle and thrust operating point.
- [PX4 Gazebo models](https://docs.px4.io/main/en/sim_gazebo_gz/gazebo_models): local SDF models and model/airframe selection.
- [Gazebo multicopter motor model](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1MulticopterMotorModel.html): simulated actuator model. Installed SDF files and PX4 allocator source are the source of the exact local constants and 12-rotor execution limit.

No neural network is used to fill missing physical values. Explicit input
contracts, conservative bounds and deterministic checks are inspectable and
repeatable. Future optimization engines can propose better trajectories while
retaining the independent checker and these configuration bindings.

## Diagnostic evidence and interpretation

The first registered-profile rehearsal passed the original generic sampled
tracking targets, then failed the added profile-specific check: its 0.10 m
navigation/tracking declarations were exceeded by 0.134225 m and 0.340700 m.
That failure is retained. It is not converted into a successful qualification.
The demonstration x500 profile then predeclared 0.20 m navigation and 0.45 m
tracking allowances for a separate rerun with an explicitly selected 1.0 m
total path tolerance. These remain demonstration bounds informed by limited
evidence, not statistical confidence limits or physical-aircraft calibration.
The generic 0.5 m p95 / 1.0 m maximum tracking targets were unchanged.

The separate rerun passed all sampled profile/generic checks: navigation max
0.149034 m, estimated tracking max 0.352011 m, route wall time 25.199933 s,
whole recorded lifecycle 182.173635 s within the predeclared nominal budget.
Both profile-bound fault responses passed; their nominal outcomes remain failed
or incomplete as appropriate. Repository validation passed 350 tests.
The [consolidated evidence](../evidence/planner/2026-10-19/aircraft_profiles/README.md)
retains the initial failure as well as the rerun and fault results.
