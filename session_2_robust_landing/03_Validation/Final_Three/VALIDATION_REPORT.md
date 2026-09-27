# PX4 randomized initialization validation

Validated trials: **3/3**

| Mode / seed | Z requested | Roll / pitch | Final XY | Final Z | Handoff | Z RMSE |
|---|---:|---:|---:|---:|---:|---:|
| random / 42 | 4.925 m | -1.11° / 0.95° | 0.071 m | 0.003 m | 0.234 m @ 0.103 m/s | 0.007 m |
| random / 2026 | 6.583 m | 1.44° / -1.59° | 0.023 m | -0.012 m | 0.231 m @ 0.102 m/s | 0.010 m |
| manual / 2196374805 | 5.000 m | 2.00° / -2.00° | 0.039 m | 0.023 m | 0.234 m @ 0.102 m/s | 0.009 m |

## Aggregate acceptance results

- Native PX4 landing/auto-disarm: 3/3
- Maximum absolute requested-vs-realized spawn-height error: 0.000005 m
- Maximum terminal XY error: 0.071 m
- Maximum absolute terminal Z error: 0.023 m
- Maximum handoff vertical speed: 0.103 m/s
- Handoff height range: 0.231–0.234 m
- Maximum trajectory Z RMSE: 0.010 m
- Median stabilization time: 9.350 s
- Maximum stabilization horizontal excursion: 1.604 m
- Maximum stabilization horizontal speed: 1.342 m/s
- Maximum stabilization vertical excursion: 0.459 m
- Maximum stabilization vertical speed: 0.610 m/s

Every trial used the same circle trajectory, returned to the captured airborne X0/Y0,
slowed through the final metre, and required PX4 native landing detection and auto-disarm.
