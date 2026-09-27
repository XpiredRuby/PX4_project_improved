# PX4 Research Controller

Research-grade PX4 multicopter position-control overlay, jerk-limited trajectory
generator, offline analysis tools, and regression tests developed for Texas A&M
research
## Current evidence

- Final exact-source SITL regression completed with PX4 native LAND and disarm.
- 25/25 controller, trajectory, analysis, and presentation tests passed.
- Five paired baseline/fixed SITL trials completed.
- Mean XY position RMSE: 0.2061 m baseline → 0.2033 m fixed.
- Landing drift: 50.1% lower by group means.
- Takeoff and landing velocity-command transition steps reduced by 95.0% and
  99.2%, respectively.
- Nine >75 ms loop intervals in 26,763 iterations were traced to host/WSL
  scheduling stalls, not controller computation.

These results are software-in-the-loop evidence, not flight certification.

## Session 2 — robust randomized-start landing

The complete mentor-facing Session 2 submission is in
[`session_2_robust_landing/`](session_2_robust_landing/README_SUBMISSION.md).

- Five randomized/boundary SITL trials completed: 5/5
- PX4 native LAND and automatic disarm: 5/5
- Maximum terminal XY error: 0.065 m
- Maximum absolute terminal Z error: 0.065 m
- Maximum handoff vertical speed: 0.105 m/s
- Full exact-source snapshots, launchers, trial summaries, validation reports,
  raw proof telemetry, post-disarm logs, tests, and SHA-256 checksums included
- Current randomized overlay tests: 19/19 passed
- Protected fixed-baseline tests: 11/11 passed

## Repository layout

| Path | Purpose |
|---|---|
| `controller/` | Validated MAVLink outer-loop controller and safe runner |
| `session_2_robust_landing/` | Complete randomized-start landing implementation and validation package |
| `reference/vishnu_original_baseline/` | Vishnu Saj's original baseline source and untouched archive |
| `trajectory_generator/` | Deterministic jerk-limited mission generator |
| `analysis/` | Run analysis and presentation-ready plot generation |
| `tests/` | Portable controller, trajectory, analysis, and plot tests |
| `scripts/validate.py` | Cross-platform generation and regression entrypoint |
| `docs/` | Audit, reproducibility, safety, and publishing guidance |
| `.github/workflows/ci.yml` | Automated generation, compilation, and tests |

## Local validation

Use Python 3.10 or newer:

```bash
python -m pip install -r requirements.txt
python scripts/validate.py
```

The validation runner uses separate Python processes for the controller and
trajectory-generator suites because both validated codebases intentionally have
a module named `trajectory.py`.

Generate mentor-facing plots from the exported CSV package:

```bash
python analysis/presentation_plots.py /path/to/plot-ready-csvs presentation_plots
```

## Code lineage

1. [Vishnu Saj original baseline](reference/vishnu_original_baseline/README.md)
   preserves the supplied starting controller, trajectory generators, datasets,
   and untouched original ZIP.
2. The root `controller/`, `trajectory_generator/`, `analysis/`, and
   `tests/` directories contain the first validated improvement phase.
3. [`session_2_robust_landing/`](session_2_robust_landing/README_SUBMISSION.md)
   contains the active randomized-start stabilization, trajectory return,
   controlled PX4 LAND handoff, automatic-disarm proof, and complete validation
   evidence.

## Safety boundary

Do not use this repository for a physical flight without explicit authorization
and in-person supervision from Dr. Benedict or Vishnu Saj. Review
[`docs/SAFETY.md`](docs/SAFETY.md) before any hardware work.

## Publishing status

This repository is currently public. Ownership, attribution, and license terms
must still be confirmed by the research team before broader redistribution.
See [`docs/PUBLISHING_CHECKLIST.md`](docs/PUBLISHING_CHECKLIST.md).
