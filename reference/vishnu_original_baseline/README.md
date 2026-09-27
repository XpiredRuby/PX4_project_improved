# Vishnu Saj Original PX4 Baseline

This directory preserves the original PX4 research code supplied by Vishnu Saj
and used as the starting baseline for Vin Manoj Nair's controller work.

Permission to include this baseline in the repository was confirmed on
2026-09-27.

## Contents

- `source/Position_Controller/` — original PID position controllers, vehicle
  state handler, plot utility, and trajectory reader
- `source/Trajectory/` — original non-jerk trajectory generator
- `source/Trajectory_jerk/` — original jerk-limited trajectory generator
- `PX4_project-main.zip` — untouched original archive containing all 49 files,
  including six CSV datasets and the 19 compiled cache files present in the
  supplied ZIP

The browsable `source/` tree intentionally contains only the 24 original
Python source files. Historical CSV data and cache files remain preserved
byte-for-byte in the original ZIP.

Original archive SHA-256:

```text
e4c831e2e705863994cb127beb12a85d68b48527e5093084b3c1c2c8e80e90f2
```

## Baseline behavior

The original position controller runs an external 20 Hz velocity-command loop
through the following phases:

```text
TAKEOFF -> TRAJECTORY -> LAND -> DONE
```

It provides the initial PID and trajectory-tracking foundation. Its landing
phase commands zero horizontal velocity and stops descent near the ground, but
does not request PX4 native LAND, verify touchdown, or confirm automatic
disarming.

## Updated implementation

The active Session 2 implementation is maintained separately at:

- [Session 2 robust landing package](../../session_2_robust_landing/README_SUBMISSION.md)
- [Active randomized-start controller](../../session_2_robust_landing/01_Source/Controller/RandomizedPositionController.py)
- [Active PX4 mission runner](../../session_2_robust_landing/01_Source/Controller/vishnu_offboard_runner.py)
- [Five-trial validation](../../session_2_robust_landing/03_Validation/Visible_Five/VALIDATION_REPORT.md)

The baseline is retained for attribution, reproducibility, and comparison. It
must not replace the validated Session 2 controller.

## Safety

This material is simulation research code. Do not use it to arm or fly a
physical vehicle without explicit authorization and in-person supervision from
Vishnu Saj or Dr. Moble Benedict.
