import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { corridor, densityAltitude, dryden, speedBand, trackingBound, windAtAltitude } from '../src/fidelity.ts';

const env = JSON.parse(readFileSync(new URL('../../config/fidelity/x500_envelope.json', import.meta.url)));

test('envelope file carries a calibrated bound for slow flight that grows with horizon', () => {
  assert.equal(speedBand(env, 1), 'slow');
  const b1 = trackingBound(env, 1, 1), b20 = trackingBound(env, 1, 20);
  assert.ok(b1 > 0 && b20 >= b1);
  assert.ok(trackingBound(env, 1, 7) >= trackingBound(env, 1, 5));
});

test('log-law wind increases with height and equals input at 10 m', () => {
  assert.ok(Math.abs(windAtAltitude(5, 10) - 5) < 1e-9);
  assert.ok(windAtAltitude(5, 40) > 5 && windAtAltitude(5, 3) < 5);
});

test('Dryden low-altitude intensities follow MIL-F-8785C ratios', () => {
  const d = dryden(5, 100 * 0.3048);
  assert.ok(d.su > d.sw && d.sw > 0);
  assert.ok(Math.abs(d.su / d.sw - 1 / Math.pow(0.177 + 0.0823, 0.4)) < 1e-9);
});

test('density altitude is near zero at ISA sea level', () => {
  assert.ok(Math.abs(densityAltitude(15, 1013.25)) < 5);
  assert.ok(densityAltitude(35, 1000) > 700);
});

test('corridor flags speeds and winds outside the calibrated envelope', () => {
  const calm = corridor(env, 1.5, { speed: 1, gust: 1.5, altitude: 12 });
  assert.equal(calm.level, 'calibrated');
  assert.ok(calm.radius >= calm.tracking);
  const fast = corridor(env, env.calibrated_speed_max_m_s * 2, { speed: 1, gust: 1, altitude: 12 });
  assert.equal(fast.level, 'uncalibrated');
  const windy = corridor(env, 1.5, { speed: 9, gust: 14, altitude: 12 });
  assert.notEqual(windy.level, 'calibrated');
  assert.ok(windy.radius > calm.radius);
});

import { gnssOutage } from '../src/fidelity.ts';

test('GNSS outage shorter than EKF2_NOAID_TOUT does not trigger failsafe (SITL gust-gps-4s: 0.63 m)', () => {
  const r = gnssOutage(4, 6.7, 19.6);
  assert.equal(r.failsafe, false);
  assert.ok(Math.abs(r.driftSimM - 0.63) < 0.3);
});

test('calm 12 s outage triggers Descend with sub-metre drift (SITL calm-gps-12s: 0.69 m)', () => {
  const r = gnssOutage(12, 0, 19.6);
  assert.equal(r.failsafe, true);
  assert.ok(Math.abs(r.driftSimM - 0.69) < 0.5);
});

test('gust 12 s outage reproduces the calibration case (SITL gust-gps-12s: 29.2 m) and predicts more on a real X500', () => {
  const r = gnssOutage(12, 6.7, 19.6);
  assert.ok(Math.abs(r.driftSimM - 29.2) < 4, String(r.driftSimM));
  assert.ok(r.driftRealM > r.driftSimM);
});
