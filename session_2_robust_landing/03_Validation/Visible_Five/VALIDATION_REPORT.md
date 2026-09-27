# PX4 randomized initialization validation

Validated trials: **5/5**

| Mode / seed | Z requested | Roll / pitch | Final XY | Final Z | Handoff | Z RMSE |
|---|---:|---:|---:|---:|---:|---:|
| random / 7 | 7.557 m | -1.71° / 0.14° | 0.044 m | -0.019 m | 0.232 m @ 0.101 m/s | 0.015 m |
| random / 42 | 4.925 m | -1.11° / 0.95° | 0.033 m | 0.004 m | 0.229 m @ 0.101 m/s | 0.017 m |
| random / 2026 | 6.583 m | 1.44° / -1.59° | 0.003 m | -0.014 m | 0.242 m @ 0.104 m/s | 0.011 m |
| random / 31415 | 9.188 m | -1.83° / 0.10° | 0.011 m | -0.006 m | 0.222 m @ 0.099 m/s | 0.027 m |
| manual / 3637565058 | 5.000 m | 2.00° / -2.00° | 0.065 m | -0.065 m | 0.239 m @ 0.105 m/s | 0.017 m |

## Aggregate acceptance results

- Native PX4 landing/auto-disarm: 5/5
- Maximum absolute requested-vs-realized spawn-height error: 0.000005 m
- Maximum terminal XY error: 0.065 m
- Maximum absolute terminal Z error: 0.065 m
- Maximum handoff vertical speed: 0.105 m/s
- Handoff height range: 0.222–0.242 m
- Maximum trajectory Z RMSE: 0.027 m
- Median stabilization time: 8.450 s
- Maximum stabilization horizontal excursion: 1.587 m
- Maximum stabilization horizontal speed: 1.305 m/s
- Maximum stabilization vertical excursion: 0.449 m
- Maximum stabilization vertical speed: 0.629 m/s

Every trial used the same circle trajectory, returned to the captured airborne X0/Y0,
slowed through the final metre, and required PX4 native landing detection and auto-disarm.

## Visible touchdown and post-disarm proof

A follow-up GUI trial used random seed 42 and archive
`/mnt/f/PX4/logs/20260920-220044-research-randomized-jerk`.

- PX4 native LAND and auto-disarm: confirmed
- Continuous post-disarm observation: 10.0 s
- Terminal armed state: false
- Terminal motor outputs: 0.0, 0.0, 0.0, 0.0
- Terminal velocity: vx 0.0010 m/s, vy 0.0067 m/s, vz 0.0014 m/s
- Terminal XY error: 0.054 m
- Terminal Z error: 0.011 m
- Randomized-overlay unit tests: 19/19 passed
- Protected fixed-baseline tests: 11/11 passed

The observation gate requires fresh heartbeat, position, and actuator telemetry,
four zero propulsion outputs, and continuously settled motion. It does not send
a forced-disarm or motor-cut command; touchdown detection and propulsion
shutdown remain PX4 LAND behavior.

