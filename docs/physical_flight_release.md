# Preparing and running a supervised physical flight

The reusable workflow is: complete the aircraft profile, plan, inspect the
interpretation, confirm the exact path, pass its registered Gazebo rehearsal,
qualify the loaded aircraft and operating site, prepare a separate release,
and explicitly launch through the local CLI with the operator and mentor.
Aircraft-specific measurements are data inputs; they do not require rewriting
the planner. Mass alone cannot supply thrust, inertia, battery, tuning or
tracking evidence.

Supported execution is PX4 GNSS/IMU parallel-axis fixed-pitch multicopters
with 4–12 rotors, fixed mission heading, one monitored battery on
`BATTERY_STATUS` ID 0, and every propulsion output observable in the same
`ACTUATOR_OUTPUT_STATUS` bank. PX4 v1.17 is the tested simulator version.
Other aircraft classes and autopilots need new dynamics and execution adapters.
Only x500 currently has connected simulation evidence. Physical qualification
has not been performed by the project.

## Qualification request

Run from the repository root after a passing nominal, profile-bound rehearsal
with the current source. `PLAN` is the retained studio directory containing
`summary.json`, `aircraft_profile.json` and `rehearsal/reviewed-plan`.

```bash
python3 -m mission_planner.flight_release init \
  --plan "$PLAN" --output qualification/request.json
```

This reads local evidence and creates a **blocked template**. It does not
connect to a vehicle. Complete its null fields with the qualified people who
have access to the aircraft. Keep evidence files relative to the request's
directory and include their SHA-256 digests. Do not use the illustrative x500,
quad, hex or octo fixtures as physical identification evidence.

| Evidence category | Required review |
|---|---|
| Loaded aircraft identification | Loaded mass/CG, rotor locations/spin, inertia and individual allocation assumptions, actual payload/propeller/battery configuration. |
| Thrust, power and battery | Available thrust across the operating envelope, power upper bound, usable Wh at the declared minimum launch charge, battery estimator/voltage accuracy, reserve and abort/landing allowance. |
| Controller and bootstrap | Exact complete `MissionConfig`, PX4 tuning, thrust calibration, GPS/IMU-only heading bootstrap, tilt/height limits and non-route phases. |
| Tracking and navigation | Conservative position and tracking errors, speed/acceleration/jerk/body-rate limits and time multiplier across declared conditions. A single sampled pass is not a universal guarantee. |
| Manual override and failsafes | Bench/supervised evidence of mode takeover, RC/lost-link behavior, offboard loss, low battery and native landing. Identify every motor's output index, PX4 function parameter, disarmed output and stop tolerance. PWM disarmed values need not be zero. |
| Site, terrain and obstacles | Lawful operating basis, cleared volume, surveyed launch coordinates, terrain/altitude datum, route **and** bootstrap/takeoff/return/landing clearance, GNSS conditions and measured wind bound. |
| PX4 parameters | JSON object of approved numeric parameter values. Include the complete reviewed parameter snapshot where available, all audited estimator/safety/bootstrap/allocation values, every motor output-function assignment, control tuning and failsafe/override/geofence settings. Every supplied parameter is checked live. |

The physical profile’s contingency must cover `land_timeout_s + 10` seconds
for guard-triggered landing, takeover and shutdown confirmation. This allowance
is additional to the nominal lifecycle and the retained reserve; the energy
budget must still pass. The simulator’s 30-second demonstration contingency
is insufficient for physical release and remains a demonstration assumption.

The record pins the nonzero MAVLink 2 UID2, exact firmware/custom version,
board version, system/component, source, profile and rehearsed configuration.
It also requires the named operator, reviewer and PhD mentor. Qualification
expires within 30 days; launch conditions must be reviewed within 15 minutes.
The wind/site declarations must contain the planned operating envelope. A
changed profile, tuning, source or trajectory requires another rehearsal and
review. The physical parameter values audited by the runner and the allocator
must match those used in rehearsal; simulator tuning must be adjusted and
rehearsed before accepting different physical values.

`motor_output_indices` refers to the observed telemetry bank, not an assumed
connector order. `motor_function_parameters` lists the approved PX4 output
function parameter for each motor; values must include each Motor 1..N once
(functions 101..100+N). All mappings and disarmed values need bench evidence.
Multiple unobservable output banks are unsupported. The telemetry confirms
commanded disarmed values; it cannot independently measure rotor rotation.

## Prepare

```bash
python3 -m mission_planner.flight_release prepare \
  --request qualification/request.json --output releases/flight-001
```

Preparation validates all evidence, the complete passing nominal assessment,
independent contact audit, raw CSV/ULog/evidence digests,
ground/disarm/stopped-output evidence, exact runner
configuration and current source fingerprint. It freezes a self-contained
bundle and prints its `release_sha256`. Original planner artifacts remain
`SITL_ONLY`; the separate `PHYSICAL_FLIGHT_PREPARED` record is an operator
attestation with integrity checks. Hashes and UID2 are not cryptographic
reviewer identity or transport authentication. Use a trusted local host and
qualified direct/secured MAVLink link, and do not edit parameters while a
flight is being prepared or run.

## Launch and abort

**The launch command below can arm and operate the physical aircraft.** Use it
only in the qualified cleared site with the named operator and PhD mentor
present. `LINK` is the explicitly selected serial or network MAVLink endpoint;
never infer it from the connected x500 demonstration.

```bash
MAVLINK20=1 python3 -m mission_planner.flight_release launch \
  --release releases/flight-001/release.json \
  --confirm-sha256 "$RELEASE_SHA256" --connection "$LINK" \
  --operator "$OPERATOR" --mentor "$MENTOR" --mentor-present
```

The runner checks exact bytes/configuration before connection, then requests
identity and reads parameters before its receiver starts. It never writes
physical PX4 parameters. Fresh disarm, on-ground and all mapped disarmed
outputs must remain confirmed for two seconds before prestream. Immediately
before arming it rechecks the release, telemetry and five-minute local launch
confirmation, then atomically consumes the release. A failed arming attempt
also consumes it; prepare a newly reviewed launch rather than replay it.

During bootstrap and trajectory execution the guard checks battery, RC link,
GPS/site boundary, energy-derived flight deadline and abort request at up to
100 ms intervals. Missing/unknown telemetry blocks or aborts; RC RSSI 255 is
unknown. Battery reserve includes the declared contingency. Expiry applies
before arming; an aircraft is not abruptly stopped because a record expires
in flight. Guard failure enters the runner's existing native LAND/PX4-failsafe
cleanup, respecting pilot mode takeover and never force-disarming airborne.
The independent native PX4 battery, geofence and link failsafes remain essential.

From another terminal:

```bash
python3 -m mission_planner.flight_release abort \
  --release releases/flight-001/release.json
```

Keep the controller running for landing and automatic disarm confirmation.
The marker is an additional local abort path, not a substitute for the pilot's
independent override. An abort lands under the configured native policy; it is
not a general obstacle-aware emergency diversion planner. This entire policy
must be reviewed for the cleared operating volume.

`live_preflight.json`, `CONSUMED.json` and the runner manifest preserve the live
identity/parameters, release digest, operator/mentor and output-stop criteria.
No browser endpoint accepts a physical arm/launch command.

## Software verification

`tests/test_flight_release.py` uses explicitly synthetic fixtures to test
missing evidence, integrity, identity/parameter mismatch, expiry/replay,
telemetry/energy/site refusals and motor mappings. These fixtures qualify no
aircraft. `scripts/check_release_workflow.py --confirm-sitl ...` exercises
release hooks, the battery/site/time guard and operator abort in an owned
Gazebo instance. Its reports remain `SITL_ONLY`; the simulator does not supply
human RC or physical identification evidence.

Protocol references: [PX4 v1.17 Offboard](https://docs.px4.io/v1.17/en/flight_modes/offboard),
[PX4 parameters](https://docs.px4.io/v1.17/en/advanced_config/parameter_reference),
[MAVLink common messages](https://mavlink.io/en/messages/common.html).
