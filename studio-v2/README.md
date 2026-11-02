# PX4 Trajectory Studio 2.0 — Milestones 1 & 2 preview

This is a separate React + TypeScript engineering-workspace frontend, **not**
a replacement for the legacy application. It reuses the existing
`mission_planner.preview` contract and the established Pyodide
`web/planner-worker.js` compiler for browser-side trajectory planning.

## Build and run offline-source preview

From the repository root:

```bash
python -m pip install -r requirements.txt
cd studio-v2
npm install
npm run build
cd ..
python web/build_studio_v2.py
python -m http.server 8767 --bind 127.0.0.1 --directory studio-v2/preview
```

Open http://127.0.0.1:8767 . Internet access is needed for the initial Pyodide
download and live weather. Planner code is served on the same origin as this
site; no native PX4 process or airframe control is exposed.

For frontend-only work: `cd studio-v2 && npm run dev`. This development mode
does not bundle the Pyodide worker; run the full build above for integrated
planning. Never use a development server as a safety-qualified flight interface.

## Scope and evidence

- Interactive local-NED waypoint editor, precise coordinate table, hard/exact
  points, drawing, route presets, import/export of editable JSON and waypoint CSV.
  CSV headers: point_id,north_m,east_m,altitude_m,hard,exact (last three optional
  except north/east/altitude). Place search uses Open-Meteo Geocoding/GeoNames.
- Equal-axis 2D input view and orbitable 3D engineering view; synchronized
  reference timeline and extrema-preserving plot display. Route vs full
  ground-to-ground mission reference can be toggled; this is NOT Gazebo truth.
  The playback interpolator reconstructs a C3 seventh-order reference from
  serialized position, velocity, acceleration and jerk at adjacent timestamps.
- Exactly the existing planner backend and verifier; review accepted result
  before downloading the reference CSV and numerical evidence.
- Manual wind inputs and on-demand Open-Meteo hourly modeled forecasts.
  Wind is meteorological **FROM**; converted to N/E vector, subtracted from
  reference ground velocity to calculate air-relative speed.
- **Weather is not passed to the existing planner, and it is not simulated
  in Gazebo.** Gust and vertical wind modeling are not claimed. Air-relative
  speed is analytical, based on the reference velocity and assumed wind.
- 20 m/s remains a research-request ceiling; x500 has an immutable 3 m/s
  vehicle-model cap. Reference verification is not PX4 actual tracking.

No generated reference is a flight approval. Simulation execution, truth logs,
PX4 actuator data and planned-vs-measured errors are Milestone 3 and later.

## Sources and integration

- `web/build_public_studio.py`: stable browser planner asset bundling.
- `mission_planner/preview.py`: accepted reference and speed explanations.
- `web/browser-api.js`: isolated browser planner adapter, no native control.
- Open-Meteo forecast docs: https://open-meteo.com/en/docs

### Validation commands

```bash
cd studio-v2
npm run typecheck
npm test
npm run build
cd ..
python web/build_studio_v2.py
python scripts/validate.py
```

The branch does not automatically replace the existing hosted website. The
new `preview` directory is ignored by Git and is a separate deployable artifact.
