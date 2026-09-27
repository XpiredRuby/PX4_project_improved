# PX4 Session 2 — Robust Randomized-Start Landing

Prepared for: Vishnu Saj
Prepared by: Vin Manoj Nair
Package date: 2026-09-27

## Scope

This submission contains the active Session 2 implementation and validation evidence for simulation-only randomized airborne initialization, stabilization, trajectory tracking, controlled descent, PX4 native LAND, and automatic disarm.

Environment:
- PX4 v1.17.0 SITL
- Gazebo Harmonic x500
- MAVLink offboard control
- Python outer-loop controller
- Simulation only; this package is not physical-flight certification

## Demonstrated result

- Five randomized/boundary trials completed: 5/5
- PX4 native LAND and automatic disarm: 5/5
- Maximum terminal XY error: 0.065 m
- Maximum absolute terminal Z error: 0.065 m
- Maximum handoff vertical speed: 0.105 m/s
- Maximum trajectory Z RMSE: 0.027 m
- Separate visible proof held Gazebo open for 10 seconds after auto-disarm
- Final proof state: armed false, all four motor outputs zero, settled motion

See:
- 03_Validation/Visible_Five/VALIDATION_REPORT.md
- 03_Validation/Visible_Five/trials.csv
- 05_Evidence/Post_Disarm_Proof/randomized_metrics.json
- 05_Evidence/Post_Disarm_Proof/runtime-status.txt

## Package structure

- 01_Source/Controller — active randomized-start controller, PX4 runner, configuration, tests, and supporting controller files
- 01_Source/Trajectory_jerk — jerk-limited trajectory generator used for the validated mission
- 02_Reproduction — analysis, regression, and trial-summary scripts
- 03_Validation/Visible_Five — strongest five-trial validation report and machine-readable results
- 03_Validation/Final_Three — earlier three-trial final validation retained as supporting evidence
- 04_Launchers — visible five-trial and post-disarm proof launchers/status files
- 05_Evidence/Five_Trial_Summaries — compact logs, spawn configurations, metrics, and runtime status for each accepted visible trial
- 05_Evidence/Exact_Source_Used_For_Five_Trials — exact archived source used for the 5/5 validation set
- 05_Evidence/Post_Disarm_Proof — complete representative proof-run archive, including the later post-disarm observation source and full telemetry/log data
- 06_Test_Results — current verification results
- CHECKSUMS_SHA256.txt — integrity hashes for every packaged file

## Reproduction

Random trial:

    bash /mnt/f/PX4/research/analysis/run_randomized_regression.sh random 42

Manual bounded trial:

    bash /mnt/f/PX4/research/analysis/run_randomized_regression.sh manual 1 -1 5 2 -2 90

Five visible trials:

    bash /mnt/f/PX4/internal/run-visible-five.sh

Visible post-disarm proof:

    bash /mnt/f/PX4/internal/run-visible-proof.sh

## Current verification

Run on 2026-09-27 before packaging:
- Randomized overlay tests: 19/19 passed
- Protected fixed-baseline tests: 11/11 passed

## Source provenance

The 5/5 validation runs used the archived runner with SHA-256:

    b12bbedbebc053efa43ce571b76777ed5874fa7e3d8e191681a8fd214abf8398

The active runner and post-disarm proof archive use the later runner with SHA-256:

    884500775e9c2caf025e3c8e2c3d835313ed724ad9060761913bff55fe318c0f

The later version adds the 10-second post-disarm telemetry/zero-output observation gate. Both exact source states are preserved in this package.

## GitHub status

The public repository https://github.com/XpiredRuby/PX4_project_improved contains the earlier controller/trajectory research package from 2026-09-04. The Session 2 randomized-start landing files in this ZIP were not present in that repository during the 2026-09-27 audit.

## Deliberate exclusions

- Failed development snapshots under randomized/history
- Python __pycache__ and compiled .pyc files
- Redundant full raw archives for all five accepted trials; compact evidence for all five is included, plus one complete post-disarm proof archive
- Unrelated Session 1/baseline deliverables

## Safety

Do not use this package to arm or fly the physical research vehicle without Vishnu Saj or Dr. Moble Benedict physically present and explicitly authorizing the test.
