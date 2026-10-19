# Connected PX4/Gazebo trajectory studio validation — 2026-10-19

**SITL ONLY. No physical aircraft or hardware authority was exercised.**

Reproduction used a fresh local loopback editor backend on port 8767, PX4
v1.17's compiled SITL x500 and Gazebo, and exact retained plans. The same
basic 20 m fixed-heading two-point drawing was confirmed twice.

1. A nominal mission passed all 13 recorded checks, including complete
   mission-clock tracking, native landing and fresh motor-stop evidence.
2. A second mission was interrupted only after the trajectory phase had
   been observed in live controller logs. The cancellation API accepted
   the exact plan ID and trajectory digest. The controller used PX4's
   native LAND/failsafe path and the job was classified CANCELLED only
   after fresh landing, disarm and all four zero actuator outputs.

See **summary.json** for recorded checks, exact SHA-256 digests and
ground/disarm observations. The full controller telemetry, PX4 ULogs,
simulator logs and campaign assessments remain in the flash-drive-backed
WSL environment. This compact record is not a replacement for those raw
artifacts and does not verify other vehicle models, weather, maps or hardware.
