# PX4 Vishnu Research Controller

Research-grade PX4 multicopter position-control overlay, jerk-limited trajectory
generator, offline analysis tools, and regression tests developed for Texas A&M
research with Vishnu Saj and Dr. Moble Benedict.

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

## Repository layout

| Path | Purpose |
|---|---|
| `controller/` | Validated MAVLink outer-loop controller and safe runner |
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

## Safety boundary

Do not use this repository for a physical flight without explicit authorization
and in-person supervision from Dr. Benedict or Vishnu Saj. Review
[`docs/SAFETY.md`](docs/SAFETY.md) before any hardware work.

## Publishing status

This repository is private and has not been made public. Ownership, attribution,
and license must be confirmed by the research team before publication.
See [`docs/PUBLISHING_CHECKLIST.md`](docs/PUBLISHING_CHECKLIST.md).
