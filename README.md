# Simple PX4 Flight Controller

This compact controller replaces the previous 2,735-line main controller. It keeps
the basic structure of Vishnu's code while making the mission values explicit.

## Mission

1. Connect to PX4 SITL.
2. Confirm that the vehicle is disarmed and on the ground.
3. Take off to 8 m, the altitude used in Vishnu's baseline.
4. Follow a straight 200 m path at no more than 5 m/s.
5. Hold the endpoint while landing.
6. Descend at 0.5 m/s above 5 m AGL.
7. Descend at 0.1 m/s at or below 5 m AGL.
8. At 0.15 m AGL, hand touchdown control to PX4 LAND.
9. Disarm only after PX4 reports `ON_GROUND`.

The trajectory advances by exactly 0.1 seconds per command. The real control loop is
scheduled at 10 Hz and independently checked against a 0.09 to 0.11 second tolerance
band. Three consecutive timing violations request PX4 `AUTO.LAND`.

## Run in SITL

```bash
python -m pip install -r requirements.txt
python simple_flight.py --heading-deg 0
```

- `0` degrees flies north.
- `90` degrees flies east.
- The default MAVLink connection is `udp:127.0.0.1:14540`.
- Flight data is saved to `flight_log.csv`.

Run the small logic test suite with:

```bash
python -m unittest tests/test_simple_flight.py
```

## Files

| File | Purpose |
|---|---|
| `simple_flight.py` | The complete controller and PX4 connection |
| `tests/test_simple_flight.py` | Checks the agreed values and mission logic |
| `requirements.txt` | Python packages |

## Important limit

The landing altitude is "known": the initial local-NED height is treated as the
landing surface height. This is appropriate for a flat SITL test that lands at the
same surface elevation. It is not terrain detection or obstacle avoidance.

Test this in PX4 SITL before any hardware discussion. Python is not a real-time
operating system, so the log must confirm the requested 10 Hz timing under load.
