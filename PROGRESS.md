# PROGRESS: public-data fidelity run (2026-11-23)

Branch `claude/validation-harness` (local; not pushed). Goal: make Trajectory Studio's predictions as trustworthy as public data allows, with a clear UI.

## Done
1. **Real-flight harness** (`fidelity/`, `scripts/score_nanobench.py`): NanoBench Crazyflie + Vicon. Data-quality gates removed 25 of 107 files (20 byte-identical duplicates).
2. **Public X500 V2 log miner** (`scripts/mine_x500_logs.py`, `fidelity/px4_logs.py`): 120 Offboard logs from PX4 Flight Review; 27 duplicates removed by SHA-256.
3. **ESC-RPM hunt** (`scripts/hunt_esc_logs.py`): 400 logs screened, 8 with RPM (4 unique flights).
4. **Identification + scoring** (`fidelity/x500_ident.py`, `scripts/score_x500_logs.py`): held-out-aircraft tracking model, speed-band 95% envelopes with held-out coverage, hover thrust, drag, motor lag.
5. **Gazebo replay of real setpoints** (`scripts/sitl_replay.py`): 40 real windows flown in PX4 v1.17 SITL, default tuning vs each aircraft's own tuning.
6. **Studio UI**: live conditions (Open-Meteo current + NWS station cross-check), animated wind compass and map wind field, log-law wind at flight altitude, Dryden turbulence, forecast uncertainty, density altitude, accuracy corridor around the route, confidence badge, Accuracy evidence panel with credibility card.
7. **Real-aircraft tuning in SITL** (`simulation_binding.py --sitl-tuning-params`): allow-listed, hash-bound, SITL-only.
Evidence: `validation/2026-11-23-x500-public-logs/README.md`.

## Key findings
- Studio model vs real X500s (held-out aircraft): median error 0.13 m vs 0.22 m for plan-equals-flight; 95% bound 1.26 m at 20 s below 2 m/s (99% held-out coverage).
- Biggest sim-to-real gap is configuration: Gazebo with the real aircraft's PX4 tuning cuts p95 error from 3.1 m to 1.2 m.
- Default Gazebo x500: hovers at 73% vs fleet median 62%; motors respond in 12.5 ms vs 55-190 ms real; 3-4x less drag than the real fleet.

## Next (highest value first)
1. Calibrated Gazebo model variant: motorConstant x1.38, motor time constant ~0.1 s, higher rotor drag; re-run the replay to measure the improvement.
2. Predict PX4 failsafe behaviour for GNSS outages longer than EKF2_NOAID_TOUT instead of a touchdown point.
3. More fast-flight (> 3 m/s) public windows; then own RTK + anemometer flights once the software is complete.
