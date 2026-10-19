# Connected-machine validation checkpoint

Date: 19 October 2026. This follows the offline validation record. It records
actual browser checks, simulator refusals and executed simulation missions.
All flights described here are Gazebo SITL. Physical aircraft have not flown.

## Browser result: passed

The studio ran in an isolated WSL checkout of commit
`b67d71a0893a51913853f072458dbd17ee30c346`, served on local port 8766 to the
installed Windows Chrome. Playwright exercised the real page and compiler:

| Check | Observed result |
|---|---|
| Drawn straight path | Accepted by the independent verifier; route duration 25.167786 s |
| Edit after acceptance | Prior result hidden and invalidated |
| One-second deadline | HTTP 422; no accepted plan displayed |
| Circle interview with tangent yaw | Accepted offline; route duration 58.716044 s |
| Downloaded CSV and bundle | CSV SHA-256 matched the bundle's bound trajectory digest |
| Desktop viewport, 1440 x 1100 | No horizontal overflow; screenshot inspected |
| Mobile viewport, 390 x 844 | No horizontal overflow |
| JavaScript errors | None recorded |

The automated test's final state was `PASSED`. Browser screenshots and the raw
downloaded artifacts remain in the machine's dedicated QA folder. A browser
preview remains offline evidence; the tangent-yaw case did not execute in PX4.

## Simulator environment identified

- Gazebo Sim 8.15.0.
- PX4 source commit `d6f12ad1c4f70ad3230afd7d86e971421e02fef4`, with an existing
  compiled `px4_sitl_default` executable.
- Existing `afvl_validation` world; its actual content must be hashed per run.
- Python 3.10.12, numpy 2.2.6, matplotlib 3.10.9, pandas 2.3.3 and
  pymavlink 2.4.49 observed in the simulator-side runner manifest.
- No running PX4/Gazebo instance was present at initial inspection.

The autopilot checkout has local additions. A commit identifier alone therefore
does not describe the complete simulator configuration. The new harness records
the world and executable hashes and must be extended with model/plugin hashes
before making stronger reproducibility claims.

## Startup and pre-arm findings

| Attempt | Observation | Disposition |
|---|---|---|
| Existing five-stop driver against new branch | Explicit CSV supplied without its required planner summary | Runner rejected before transport/arming. Batch handoff fixed. |
| Retry with summary forwarding | Active PX4 estimator still enabled barometer/magnetometer/optical-flow/range fusion | GPS/IMU-only contract rejected before arming. |
| Isolated PX4 working directory and startup overrides | Airframe startup changed `SYS_FAILURE_EN` back to 1 | Safety contract rejected before arming. Added post-start ground-state readback for safety parameters. |
| Retry with parameter readback | PX4 rejected ARM command 400, result 1; log reported no ground-station connection | No arming. A simulated GCS heartbeat is prepared; health checks remain enabled. |

The wrapper confirmed ground/disarm/zero actuator outputs before shutting down
completed earlier attempts. Failed results are retained and are not upgraded
to nominal passes. After the connection returned, the heartbeat change was deployed and the
baseline completed, including native landing and automatic disarm.

## Software changes from these findings

`sitl_scenario_batch.py` now forwards a frozen planner/waypoint summary to the
runner, supports a bound SITL acknowledgement for drawn artifacts, and rejects
missing or conflicting artifacts before execution. A missing final-state field
reports the runner's actual refusal instead of hiding it behind a `KeyError`.

The new `planner_sitl_campaign.py` has completed live lifecycle runs and is
undergoing the stricter sampled qualification described below. It creates a separate PX4 working directory for each case,
declares parameter overrides, verifies the estimator was configured at startup,
checks ground state, reads back post-start safety settings, archives hashes and
runs the existing scenario assessor. Its prepared GCS heartbeat represents
simulation transport liveness, not operator approval. It selects only local
SITL executables and loopback endpoints; it has no serial-device selection.

The live baseline reported a required uncertainty-aware cruise height of
19.56 m. The draft general-path cases therefore explicitly compile at 20 m and
set the controller's configured floor to the same value. The existing live
height equality check remains active and will reject greater required heights.
Fixed heading is used for initial circle tracking so launch/tangent alignment
can be qualified separately. These choices are serialized test conditions.

## Executed flight findings

Commit `d69df17172c584425b2a34aff40f6ce1436c7b35` completed the five-stop
baseline, drawn path and fixed-heading circle with successful original
lifecycle audits, native landing, automatic disarm and a separate continuous
two-second zero-output confirmation. Requested speed factor was 1; the two
general-path runs enabled the Gazebo GUI.

The additional independent assessment evaluates every flight sample through
the first native-landing `ON_GROUND` report, exact runtime-reference binding,
CSV/runner digests, physical position and heading, touchdown truth, shutdown
and measured real-time factor. Its position/heading statistics combine
TAKEOFF, TRAJECTORY, RETURN_HOME and ALIGN; per-phase distributions are also
reported. HANDOFF is excluded from reference tracking because PX4 can already
be descending while its LAND acknowledgement arrives. Native descent is
assessed by its lifecycle/contact evidence rather than an inactive offboard
reference.

This stricter assessment **failed** the initial baseline and drawn run on
startup heading: maximum physical errors were 11.362° and 13.772° respectively,
above the predeclared 10° target. Their original lifecycle passes are retained;
they are not called complete qualification passes.

Investigation found that the averaged heading recorded during GPS yaw
acquisition lagged the current heading at the transition to position control.
Commit `832ab7224daeceb115f07ea0fff5cd514f53dabf` preserves the fresh finite
measured heading at this handoff, while leaving the averaged launch XYZ
unchanged. Stale/non-finite attitude refuses the transition. No acceptance
threshold was changed.

The repeated five-stop baseline passed the sampled assessment: aggregate
physical 3-D position p95 0.193 m / max 0.575 m; heading p95 3.521° / max
4.831°; measured mean real-time factor 1.000034; complete truth coverage
through contact; confirmed two-second ground/disarm/zero outputs. Return-home
position p95 was 0.558 m, so this is an aggregate p95 pass, not a claim that
every individual phase had p95 below 0.5 m. Full per-phase data are retained.
The repeated drawn and circle cases were still running at this checkpoint.

PX4 can log an undefined local origin before estimator initialization. The
truth alignment selects the flight interval plus the allowed 0.25 s bracket;
invalid values inside that interval still refuse assessment. It never fills
missing flight truth or extrapolates after ULog recording stops. Post-contact
rows are excluded from the flight-truth coverage count; fresh post-disarm
actuator evidence is assessed separately.

Contact speeds currently refer to the first land-detector `ON_GROUND` report.
They do not identify the instant of rigid-body impact independently. The
sampled pass also does not establish actual wall-time dynamic/rotor margins,
obstacle-map clearance, an identified physical airframe, or hardware release.

## Campaign procedure

1. Restore the existing PowerShell tunnel and inspect simulator/process state.
2. Deploy the current batch adapter and draft harness together; verify hashes.
3. Complete a five-stop baseline with the simulated GCS heartbeat and unchanged
   health checks.
4. Run the drawn and fixed-heading circle cases, including takeoff, return,
   native landing, disarm and post-disarm checks.
5. Evaluate ULog ground truth and estimates separately, derive tracking and
   timing metrics, and apply the predeclared thresholds from the research plan.
6. Only after nominal evidence passes, exercise the bounded GNSS and control
   stall scenarios. A failed scenario stops progression.

Example command on a configured Linux simulation machine:

```bash
python3 scripts/planner_sitl_campaign.py --confirm-sitl \
  --autopilot /absolute/path/to/PX4-Autopilot \
  --output /absolute/path/to/new-campaign \
  --cases five-stop-baseline
```

Do not interpret a draft harness, successful startup or pre-arm rejection as a
successful simulation of the planned route. Whole-mission truth comparisons,
real-time-factor evidence and the wider fault campaign remain outstanding.
