# Native integration and measured-results review

Agreement: `PX4-DUAL-20261026-V1`.

GPT A's independent planner/browser implementation and offline checks are
complete for the scope recorded in `planner.md`. This document reserves the
inputs and results for the native review. Empty cells mean **not supplied or not
reviewed**, never a passing result. No Gazebo, physical tracking, touchdown or
shutdown result is inferred from planner verification.

## Source and artifact identity — GPT B input

| Required input | Value |
|---|---|
| GPT B branch and exact commit SHA | |
| Exact combined revision tested, including GPT A changes | |
| Preserved earlier local planner commit / dirty-work archive | |
| Native adapter/certificate family and version | |
| Simulator/PX4 repository URLs and exact commits | |
| Gazebo version, world, vehicle and build command | |
| Build exit code and retained log path | |
| Active aircraft model/configuration path and SHA-256 | |
| Environment and controller configuration SHA-256 | |
| Controller operating rate and native speed/clock policies | |
| Original request and effective intent paths/hashes | |
| Route CSV path and SHA-256 | |
| Full mission CSV path and SHA-256, if used | |
| Verification report, acknowledgement and certificate hashes | |
| Frame origin, NED axis convention and clock origins/offsets | |
| Run identifiers, seeds and scenario/configuration hashes | |
| Run timeout / iteration budget and actual exit codes | |
| Raw controller telemetry, PX4 ULog and Gazebo truth paths | |
| Raw contact, armed state and motor-output evidence paths | |
| GPT B review of PR #8, including accepted/rejected findings | |

## Run record — GPT B input

Provide one copy of this row per attempted run. Retain failures and interrupted
attempts; do not select only successful runs. Keep archived historical runs
separate from runs of the current combined revision.

| Run ID | Scenario | Exact revision | Command/log | Exit code | Nominal result | Cancel/failsafe result | Raw evidence |
|---|---|---|---|---|---|---|---|
| | | | | | | | |
| | | | | | | | |
| | | | | | | | |
| | | | | | | | |
| | | | | | | | |

## Independent assessment — GPT A results

Thresholds must come from the reviewed active configuration and declared
requirements. Record the actual clock and reference reconstruction used for
each metric. Do not treat reference-versus-estimator agreement as truth tracking.

| Review item | Evidence / recomputation | Threshold source | Result |
|---|---|---|---|
| Source, model, report and CSV identities match executed artifacts | | | |
| Supported family; refusal of malformed/mismatched certificates | | | |
| Native between-sample reconstruction agrees with declared reference | | | |
| Route/full-mission time offsets and phase ownership | | | |
| NED/down signs and yaw unwrap agree across channels | | | |
| Finite, monotone telemetry and declared gap/coverage policy | | | |
| Actual loop periods, deadline misses and telemetry freshness | | | |
| Truth position tracking: RMS / p95 / maximum, by phase | | | |
| Ordered route fidelity, required visits, stops and laps | | | |
| Truth speed and attitude limits, by phase | | | |
| Acceleration/jerk method, filtering and limitations | | | |
| Heading error and stationary-turn behavior | | | |
| Nominal lifecycle and actual final ground/contact observation | | | |
| Actual disarm and configured stopped-output observations | | | |
| Cancellation/failsafe ownership and observed end state | | | |
| Failed runs / missing channels / excluded cases explained | | | |
| Actual browser usability test on the integrated local studio | | | |

## Review procedure ready for the supplied artifacts

1. Freeze the supplied run directories and record hashes before analysis. Read
   GPT B's exact diff against the agreed base and inspect the actual combined
   revision; do not silently substitute a similarly named branch.
2. Recompute artifact hashes and confirm the adapter's family, unit, frame and
   clock contracts. Verify that positive fixtures execute only through supported
   guards and negative fixtures fail before a simulator/controller starts.
3. Check raw clock monotonicity, gaps, finite values and phase coverage. Align
   clocks only from documented origins/offsets; never optimize an offset just to
   reduce reported tracking error. Expose excluded intervals and their reasons.
4. Reconstruct the executed reference with the supported interpolation. Compare
   it separately with estimator and truth channels. Recompute metrics per phase
   and across the full declared run, preserving the native speed/time policy.
5. Inspect actual contact, landing, disarm and stopped outputs. A command,
   process exit, missing telemetry or zero reference height is insufficient.
6. Record disagreements and their evidence. Accept a correction only after its
   relevant checks run on the named combined revision. Keep nominal completion
   separate from successful cancellation and from physical-flight qualification.

## Decision record

Current disposition: **NOT ASSESSED — GPT B artifacts absent**.

| Field | Value |
|---|---|
| Reviewed combined commit | |
| Reviewed evidence manifest SHA-256 | |
| Findings / required corrections | |
| Rerun identifiers after corrections | |
| Independent reviewer and review date | |
| Final native integration disposition | |

The blank result fields do not block completion of GPT A's independent
implementation package. They preserve the remaining review gate without
inventing measurements or changing execution authority. There is no additional
spending, protected-branch merge or physical-operation authorization here.
