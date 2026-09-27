# Verification Results

Verification date: 2026-09-27

## Randomized landing overlay

Command:

    python3 /mnt/f/PX4/research/randomized/test_randomized_overlay.py

Result: 19 tests passed, 0 failed.

Coverage includes seeded bounded poses, quaternion normalization, propulsion-release gating, stabilization safety gates, translated trajectory origin, controlled final-metre descent, LAND handoff gating, and the post-disarm zero-output/settled-motion gate.

## Protected fixed baseline

Command:

    python3 /mnt/f/PX4/research/analysis/test_fixed_overlay.py

Result: 11 tests passed, 0 failed.

Coverage includes the controller result contract, foreign-heartbeat filtering, vector velocity limiting, landing endpoint hold, anti-windup, monotonic timing, runtime health guards, origin initialization, MAVLink mask correctness, and setpoint watchdog behavior.
