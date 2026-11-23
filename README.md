# PX4 Trajectory Studio

**Draw a drone route, check it, preview the flight, and see how accurately a real X500 would fly it, before anyone goes outside.**

Built for the Holybro X500 V2 on PX4 v1.17 with Gazebo 8.15. The planner and safety checks are independent of the simulator; Gazebo is used to fly the result.

![Route with accuracy corridor, animated wind and live conditions](docs/img/route-weather.png)

| | |
|---|---|
| **Plan** | Draw or type waypoints; the planner builds a smooth, speed/acceleration/jerk-limited trajectory and an independent verifier checks it. |
| **Predict** | Every route gets a 95% accuracy corridor scored on **real X500 V2 flights** it never saw (public PX4 logs). |
| **Weather** | Live conditions (Open-Meteo + nearest NOAA station), wind at flight altitude, turbulence, forecast uncertainty, animated wind. |
| **Simulate** | One click flies the verified plan in PX4 SITL + Gazebo and compares plan vs simulated flight. |
| **Be honest** | A badge shows *Calibrated / Extrapolating / Uncalibrated*; the Accuracy evidence tab shows the data behind every number. |

---

## Run it in your browser (about 5 minutes)

Needs **Python 3.10+** and **Node.js 22+**. No PX4 or Gazebo needed for planning.

```bash
git clone https://github.com/XpiredRuby/PX4_project_improved.git
cd PX4_project_improved
git checkout docs/clean-readme            # this branch

python -m pip install -r requirements.txt
cd studio-v2 && npm ci && npm run build && cd ..

python -m mission_planner.research_studio --port 8767 --studies-dir ~/px4-studio-studies
```

Open **http://127.0.0.1:8767**, choose **Build route → 200 m · five stops**, then **Check route**.
Live weather needs an internet connection; everything else runs locally.

<details><summary>Optional: connect PX4 SITL + Gazebo (Linux / WSL2)</summary>

Follow [`docs/trajectory_studio_handbook.md`](docs/trajectory_studio_handbook.md): build PX4 v1.17 SITL, register the
aircraft with `mission_planner.simulation_binding`, then start the studio with `--sitl-autopilot` and `--sim-binding`.
Optional extras: `--sitl-tuning-params your.params` (use your aircraft's own PX4 tuning) and
`scripts/install_x500_cal.py` (calibrated Gazebo x500).
</details>

---

## How it is built

```mermaid
flowchart LR
    subgraph Browser["Browser (React + TypeScript)"]
        UI["Studio UI<br/>map · 3D preview · results"]
        ACC["Accuracy model<br/>corridor · badge · GNSS what-if"]
        WX["Live conditions<br/>wind · turbulence · density alt."]
    end
    subgraph Server["Local Python server (research_studio)"]
        API["HTTP API"]
        PLAN["Planner<br/>geometry + timing"]
        VER["Independent verifier"]
        Q["Study queue"]
    end
    subgraph Sim["Simulation (Linux / WSL2)"]
        RUN["Offboard runner<br/>pymavlink · 20 Hz"]
        PX4["PX4 v1.17 SITL"]
        GZ["Gazebo 8.15<br/>x500 / x500_cal"]
    end
    subgraph Data["Public data (offline scoring)"]
        FR["PX4 Flight Review<br/>real X500 V2 logs"]
        NB["NanoBench<br/>motion capture"]
    end
    OM["Open-Meteo · NOAA/NWS"]

    UI <--> API
    API --> PLAN --> VER --> Q --> RUN
    RUN <-->|MAVLink| PX4 <--> GZ
    WX <-->|HTTPS| OM
    FR & NB --> FID["fidelity/ + scripts/<br/>fit · gate · score"]
    FID -->|"config/fidelity/*.json"| ACC
```

### From a drawing to a flight

```mermaid
flowchart TD
    A["Waypoints / drawn path<br/>(MissionIntent)"] --> B["Smooth path<br/>quintic B-spline in a deviation tube"]
    B --> C["Time law<br/>speed · acceleration · jerk limits"]
    C --> D{"Independent verifier<br/>limits · tube · stops"}
    D -- fails --> E["Show why + feasible speed"]
    D -- passes --> F["Signed plan artifact<br/>SHA-256 bound, SITL_ONLY"]
    F --> G["Offboard runner"]
    G --> H["Gazebo flight + ground truth"]
    H --> I["Plan vs flight report"]
```

### What the aircraft does in a mission

```mermaid
stateDiagram-v2
    [*] --> TAKEOFF
    TAKEOFF --> TRAJECTORY
    TRAJECTORY --> RETURN_HOME
    RETURN_HOME --> ALIGN
    ALIGN --> HANDOFF
    HANDOFF --> PX4_LAND
    PX4_LAND --> [*]
    TRAJECTORY --> HOLD: navigation degraded
    HOLD --> TRAJECTORY: recovered
    HOLD --> PX4_FAILSAFE: position lost
```

### How the accuracy numbers are earned

```mermaid
flowchart LR
    L["Public X500 logs<br/>Offboard flights"] --> Q1["Quality gates<br/>duplicates · corrupt setpoints"]
    Q1 --> S["Split by aircraft<br/>(never train and test on the same drone)"]
    S --> FIT["Fit closed-loop model"]
    FIT --> ENV["95% error envelope<br/>per speed band"]
    ENV --> CHK["Check coverage on<br/>held-out aircraft"]
    CHK --> UI2["Corridor + badge in Studio"]
    W["Live weather<br/>Dryden turbulence"] --> WT["Gust term (modelled)"] --> UI2
    R["Real setpoints replayed<br/>in Gazebo"] --> GAP["Sim-to-real gap"]
```

---

## Results so far

| Question | Answer | Evidence |
|---|---|---|
| How well does Studio predict a real X500 from its setpoints? | Median error **0.13 m** vs 0.22 m if you assume the plan is flown exactly | [x500 public logs](validation/2026-11-23-x500-public-logs/README.md) |
| Does the 95% bound hold on drones it never saw? | Yes below 2 m/s (held-out coverage ~99%); faster flight is flagged | same |
| What is the biggest simulator-vs-reality gap? | **Controller tuning.** Using the real aircraft's PX4 tuning cuts Gazebo's p95 error from 3.1 m to 1.2 m | same, section 2 |
| Is the Gazebo x500 physically right? | Close but optimistic: motors respond 4-15x faster than real ones; calibrated `x500_cal` provided | same, sections 3 and 5 |
| What if GPS drops for 12 s in 6.7 m/s wind? | PX4 failsafe descent drifts **32-55 m**; reproduced on three SITL cases | Accuracy evidence tab |

| ![Accuracy evidence panel](docs/img/evidence.png) | ![GNSS outage what-if](docs/img/gnss.png) |
|---|---|
| Accuracy evidence tab | GNSS-outage what-if |

**Honest limits.** Public X500 logs carry the drone's own GPS-based estimate, not independent truth, and no measured wind.
The wind term in the corridor is modelled, not yet checked against measured wind. Nothing here has flown a physical aircraft; all output is `SITL_ONLY`.
The next step is a few supervised X500 flights with RTK GPS and a ground anemometer.

---

## Repository map

| Folder | What is inside |
|---|---|
| `studio-v2/` | The Studio web app (React + TypeScript) |
| `mission_planner/` | Planner, verifier, HTTP server, aircraft profiles, SITL binding |
| `controller/` | PX4 Offboard runner (pymavlink), mission phases, safety checks |
| `fidelity/` | Real-flight loaders, identification and scoring (public datasets) |
| `scripts/` | Command-line tools: SITL campaigns, log mining, replay, scoring |
| `analysis/` | Post-flight analysis and plotting |
| `config/`, `schemas/`, `missions/` | Planner settings, JSON schemas, example missions; `config/fidelity` drives the accuracy UI |
| `validation/` | Dated evidence folders, each with a README |
| `evidence/` | Retained plan artifacts and the archived X500 flight |
| `docs/` | Handbook, architecture, safety, reproducibility |
| `tests/` | 469 Python tests (plus 40 frontend tests in `studio-v2/tests`) |
| `web/`, `trajectory_generator/` | Portable browser planner build; original trajectory generator |

## Check everything

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q                 # 469 tests
python -m ruff check mission_planner controller analysis scripts tests fidelity
cd studio-v2 && npm test            # 40 tests
```

Safety: see [`docs/SAFETY.md`](docs/SAFETY.md). Physical flights require the supervised release process in
[`docs/physical_flight_release.md`](docs/physical_flight_release.md).
