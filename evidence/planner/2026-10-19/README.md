# Connected PX4/Gazebo evidence

All records are `SITL_ONLY`. None authorizes physical flight.

`qualification.json` retains configuration, sampled assessment, implementation
hashes, raw CSV/ULog digests and browser-check outcomes. Initial failed runs
remain alongside later passes. The full raw files are on the simulation
machine under `/home/xpire/PX4-runs`; the relative `case_path` locates each run.

`nominal_tracks.json` contains display-only decimated input, plan and truth
tracks for the repeated drawing and circle cases. `nominal_tracks.svg` renders
them. They are not the acceptance oracle. Regenerate the figure with:

```bash
python3 scripts/plot_planner_evidence.py \
  evidence/planner/2026-10-19/nominal_tracks.json \
  evidence/planner/2026-10-19/nominal_tracks.svg
```

To reproduce an assessment with the full raw evidence and dependencies:

```bash
python3 analysis/planner_rehearsal.py /absolute/path/to/case-folder
```

Interpret the scopes separately:

- A nominal sampled pass requires reference/digest binding, mission-clock
  coverage, recorded mission truth, declared tracking/fidelity targets,
  native-landing contact evidence, shutdown confirmation and measured 1×
  timing. Per-phase errors are retained alongside aggregate tracking results.
- A fault-response pass permits the declared HOLD or aborted trajectory.
  Nominal failures remain visible. The control-stall runner's nonzero exit and
  `PX4_FAILSAFE` outcome are expected evidence of that response.
- Truth coverage starts with the post-bootstrap mission CSV. Earlier guarded
  yaw-bootstrap motion is retained in ULog but is not independently scored by
  this assessment. Contact speeds use the first land-detector ON_GROUND report,
  not independently identified rigid-body impact time.
- Neither pass establishes continuous actual dynamics, identified rotor/power
  reserves, mapped obstacle clearance, a complete operating domain, or an
  authenticated physical-flight release. The existing controller uses an
  adaptive mission clock; a planned derivative certificate does not certify
  all actual wall-time dynamics.

The connected validation document explains conditions, fixes, failed targets
and remaining qualification. The research review contains the broader test
matrix and owner-dependent hardware gates.
