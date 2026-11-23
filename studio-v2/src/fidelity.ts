// Prediction-accuracy and weather-uncertainty model used by the Studio UI.
// Envelope numbers come from scripts/score_x500_logs.py (real X500 V2 flights, referent = onboard EKF2).

export type Level = 'calibrated' | 'extrapolating' | 'uncalibrated';
export type Terrain = 'open' | 'mixed' | 'complex';

export type Envelope = {
  schema: string; aircraft: string; referent: string; flights: number; vehicles: number;
  horizons_s: number[]; speed_bands_m_s: [number, number, string][];
  envelope_95_m: Record<string, Record<string, number> | null>;
  heldout_coverage: Record<string, Record<string, number>>;
  calibrated_speed_max_m_s: number;
  loop: { horizontal: { wn_rad_s: number; zeta: number; delay_s: number }; vertical: { wn_rad_s: number; zeta: number; delay_s: number } };
  drag_per_mass_linear_s?: number;
};

export const ROUGHNESS_M: Record<Terrain, number> = { open: 0.03, mixed: 0.1, complex: 0.8 };
// Forecast 10 m wind-speed error (1 sigma) for 0-6 h lead, from WIND-Bench HRRR-vs-MADIS RMSE (1.5-3.1 m/s).
const FORECAST_SIGMA: Record<Terrain, number> = { open: 1.5, mixed: 2.0, complex: 3.0 };

export function speedBand(env: Envelope, speed: number): string {
  const hit = env.speed_bands_m_s.find(([lo, hi]) => speed >= lo && speed < hi);
  return hit ? hit[2] : env.speed_bands_m_s[env.speed_bands_m_s.length - 1][2];
}

/** 95% tracking-error bound (m) after `horizon` seconds; null when that band has no calibration data. */
export function trackingBound(env: Envelope, speed: number, horizon = 20): number | null {
  const band = env.envelope_95_m[speedBand(env, speed)];
  if (!band) return null;
  const hs = env.horizons_s;
  const h = Math.min(Math.max(horizon, hs[0]), hs[hs.length - 1]);
  const i = hs.findIndex(x => x >= h);
  if (i <= 0) return band[String(hs[0])];
  const [h0, h1] = [hs[i - 1], hs[i]];
  const [b0, b1] = [band[String(h0)], band[String(h1)]];
  return b0 + (b1 - b0) * (h - h0) / (h1 - h0);
}

/** Log-law wind at height z (m) from the 10 m value. */
export function windAtAltitude(u10: number, z: number, terrain: Terrain = 'open'): number {
  const z0 = ROUGHNESS_M[terrain];
  const zz = Math.max(z, 2 * z0 + 0.5);
  return u10 * Math.log(zz / z0) / Math.log(10 / z0);
}

/** MIL-F-8785C / MIL-HDBK-1797 low-altitude Dryden intensities (m/s) and length scale (m). */
export function dryden(u10: number, altitude: number, terrain: Terrain = 'open') {
  const w20 = windAtAltitude(u10, 6.096, terrain); // 20 ft reference wind
  const hft = Math.max(altitude, 1) / 0.3048;
  const sw = 0.1 * w20;
  const su = sw / Math.pow(0.177 + 0.000823 * hft, 0.4);
  const lu = hft / Math.pow(0.177 + 0.000823 * hft, 1.2) * 0.3048;
  return { su, sv: su, sw, lu };
}

export function forecastSigma(terrain: Terrain, leadHours = 0): number {
  return FORECAST_SIGMA[terrain] + 0.3 * Math.max(0, leadHours) / 24;
}

/** Density altitude (m) from temperature (degC) and station pressure (hPa), dry air. */
export function densityAltitude(tempC: number, pressureHpa: number): number {
  const rho = pressureHpa * 100 / (287.05 * (tempC + 273.15));
  return 44330.8 * (1 - Math.pow(rho / 1.225, 0.234969));
}

export type Corridor = { radius: number; tracking: number; wind: number; level: Level; reasons: string[] };

/**
 * Predicted-path corridor (95%) around the planned route.
 * tracking: calibrated on held-out real flights; wind: model-based (gust excess plus 2-sigma Dryden
 * turbulence through the identified closed loop), not yet validated against measured wind.
 */
export function corridor(env: Envelope, maxSpeed: number, wind: { speed: number; gust: number; altitude: number; terrain?: Terrain }): Corridor {
  const terrain = wind.terrain ?? 'open';
  const reasons: string[] = [];
  let level: Level = 'calibrated';
  const calMax = env.calibrated_speed_max_m_s;
  let tracking = trackingBound(env, maxSpeed);
  if (tracking === null || maxSpeed > calMax) {
    const fast = env.envelope_95_m.fast ?? env.envelope_95_m.slow;
    const base = fast ? fast[String(env.horizons_s[env.horizons_s.length - 1])] : 5;
    tracking = base * Math.min(3, Math.max(1, maxSpeed / Math.max(calMax, 0.1)));
    level = maxSpeed > 1.5 * calMax ? 'uncalibrated' : 'extrapolating';
    reasons.push(`Planned speed ${maxSpeed.toFixed(1)} m/s exceeds the ${calMax.toFixed(1)} m/s covered by public X500 flights.`);
  }
  const { su } = dryden(wind.speed, wind.altitude, terrain);
  const uAlt = windAtAltitude(wind.speed, wind.altitude, terrain);
  const gustAlt = windAtAltitude(Math.max(wind.gust, wind.speed), wind.altitude, terrain);
  const disturbance = Math.max(0, gustAlt - uAlt) + 2 * su;
  const k = env.drag_per_mass_linear_s ?? 0.45;
  const wn = env.loop.horizontal.wn_rad_s;
  const windPart = k * disturbance / (wn * wn);
  if (uAlt > 5) {
    if (level === 'calibrated') level = 'extrapolating';
    reasons.push(`Wind at flight altitude ${uAlt.toFixed(1)} m/s: wind response is modelled, not validated against measured wind.`);
  }
  if (uAlt > 10) {
    level = 'uncalibrated';
    reasons.push('Wind above 10 m/s is outside every dataset used for calibration.');
  }
  if (reasons.length === 0) reasons.push(`Inside the calibrated envelope: ${env.vehicles} real X500s, held-out coverage reported.`);
  return { radius: Math.hypot(tracking, windPart), tracking, wind: windPart, level, reasons };
}

export const LEVEL_TEXT: Record<Level, string> = {
  calibrated: 'Calibrated', extrapolating: 'Extrapolating', uncalibrated: 'Uncalibrated',
};
