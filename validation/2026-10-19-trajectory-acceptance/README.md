# Trajectory and Gazebo acceptance — 19 October 2026

The reference simulation workflow passed the declared automated acceptance checks. Manual user testing is the next gate.

Implementation commit: 34cb0dc76c58217baaea16f6732f4458a7b692ff. All five cases bind the same Python source fingerprint. The browser exercised the retained implementation with the exact committed HTML and test script.

Validation: 380 tests in the full local suite, then the final nine-test job check including one additional concurrency regression (381 unique tests); Ruff and JavaScript syntax passed. All 16 offline acceptance cases passed. Chrome passed freehand drawing, impossible-deadline refusal, exact confirmation, nominal simulation, refresh recovery, truth overlay/download and separate route-phase cancellation.

| Case | Checks | 3-D tracking p95 / max (m) | Max speed (m/s) | Max tilt (deg) |
|---|---:|---:|---:|---:|
| turn-nominal | 16 passed | 0.286 / 0.446 | 0.910 | 4.18 |
| hard-stop-nominal | 16 passed | 0.358 / 0.456 | 1.119 | 5.23 |
| circle-nominal | 16 passed | 0.147 / 0.356 | 1.015 | 4.19 |
| drawn-nominal | 16 passed | 0.287 / 0.380 | 0.976 | 4.16 |
| survey-nominal | 16 passed | 0.347 / 0.527 | 0.932 | 4.07 |

The separate hard-stop audit checks actual simulator position and speed at the required interior stop, not only a stationary reference. See hard-stop-quality.json for its predeclared 0.5 m / 0.2 m/s / 0.1 s thresholds and raw evidence hashes.

Raw CSV, ULog, simulator logs, Chrome profiles and screenshots remain on the flash-drive-backed machine at the paths recorded in summary.json. Compact hashes and results are retained here; large raw artifacts are intentionally not copied into Git.

Reproduction and manual trial: [trajectory simulation acceptance](../../docs/trajectory_simulation_acceptance.md). These results apply to the named fixed-heading x500 scenarios and do not establish arbitrary-path, all-weather or continuous actual-dynamics guarantees.

## Final interface recovery

A dedicated Chrome check confirms that an unavailable saved rehearsal is cleared, the reference simulator is visible without a questionnaire, and a new plan can reach its explicit confirmation gate. See final-ui-recovery.json for the final HTML and executable check hashes. Reproduce on Windows with the Playwright environment:

```powershell
python validation/2026-10-19-trajectory-acceptance/check_session_recovery_windows.py --output F:\PX4\new-recovery-check
```

The full nominal/cancellation browser runs and targeted final recovery check retain separate HTML hashes in summary.json. An earlier hidden-selector browser failure is retained under F:\PX4\readiness-browser-oct19\nominal-hidden-selector-failed; the selector was made directly visible before the successful full browser runs.
