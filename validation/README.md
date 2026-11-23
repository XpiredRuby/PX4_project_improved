# Validation evidence

Each dated folder is one experiment, frozen with its inputs, results and (where present) a README. Newest first.
Nothing here is a physical flight: real-flight folders use **public** logs; the rest are PX4 SITL + Gazebo runs.

## Start here: accuracy against real flights

| Folder | Question it answers | Data |
|---|---|---|
| [`2026-11-23-x500-public-logs`](2026-11-23-x500-public-logs/README.md) | How well does the planner predict a real Holybro X500 V2? How far is Gazebo from reality? | PX4 Flight Review, real X500 V2 |
| [`2026-11-23-measured-wind`](2026-11-23-measured-wind/README.md) | How much does measured wind push a quadcopter off its route? | CMU DJI M100, onboard anemometer |
| [`2026-11-23-nanobench-fidelity`](2026-11-23-nanobench-fidelity/README.md) | Does the prediction method work against independent motion-capture truth? | NanoBench |

## Simulator rehearsals (PX4 v1.17 SITL + Gazebo 8.15)

| Folder | What ran |
|---|---|
| [`2026-11-22-native-land`](2026-11-22-native-land/README.md) | Slow approach and PX4 AUTO LAND, 22 checks |
| [`2026-11-22-full-200m`](2026-11-22-full-200m/README.md) | Full 200 m route at 5 m/s cruise |
| [`2026-11-21-five-stops`](2026-11-21-five-stops/README.md) | 200 m route with five stops, four trials |
| [`2026-10-25-conference`](2026-10-25-conference/README.md) | Planner engineering validation and browser checks |
| [`2026-10-19-trajectory-acceptance`](2026-10-19-trajectory-acceptance/README.md) | 16 offline acceptance cases |
| [`2026-10-05-waypoint`](2026-10-05-waypoint/README.md) | First live five-stop mission |
| `2026-10-03` | Raw logs: GPS-loss, gust and terrain SITL cases (used by tests and the GNSS what-if) |
| `handwriting`, `motion` | Small browser QA records used by tests |
