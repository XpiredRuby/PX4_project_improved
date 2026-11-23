import { useCallback, useEffect, useState } from 'react';
import { CloudSun, RefreshCw, Radio } from 'lucide-react';
import { densityAltitude, dryden, forecastSigma, windAtAltitude, type Terrain } from './fidelity';

type Now = { speed: number; dir: number; gust: number; temp: number; pressure: number; time: string };
type Station = { id: string; name: string; speed: number | null; dir: number | null; gust: number | null; time: string };
export type LiveApply = { windSpeed: number; windDirection: number; gust: number; temperature: number; pressure: number; source: string; fetchedAt: string; forecastValidAt?: string };

const REFRESH_MS = 10 * 60 * 1000;
const kmh = (v: unknown) => (typeof v === 'number' && Number.isFinite(v) ? v / 3.6 : null);

async function fetchNow(lat: number, lon: number): Promise<Now> {
  const q = new URLSearchParams({ latitude: String(lat), longitude: String(lon), wind_speed_unit: 'ms',
    current: 'wind_speed_10m,wind_direction_10m,wind_gusts_10m,temperature_2m,surface_pressure' });
  const r = await fetch('https://api.open-meteo.com/v1/forecast?' + q);
  if (!r.ok) throw Error('Open-Meteo returned ' + r.status);
  const c = (await r.json()).current;
  return { speed: c.wind_speed_10m, dir: c.wind_direction_10m, gust: c.wind_gusts_10m, temp: c.temperature_2m, pressure: c.surface_pressure, time: c.time + 'Z' };
}

async function fetchStation(lat: number, lon: number): Promise<Station> {
  const pts = await fetch(`https://api.weather.gov/points/${lat.toFixed(4)},${lon.toFixed(4)}`);
  if (!pts.ok) throw Error('Measured station data is available for US locations only');
  const stations = await (await fetch((await pts.json()).properties.observationStations)).json();
  const s = stations.features?.[0]?.properties;
  if (!s) throw Error('No nearby station');
  const obs = (await (await fetch(`https://api.weather.gov/stations/${s.stationIdentifier}/observations/latest`)).json()).properties;
  return { id: s.stationIdentifier, name: s.name, speed: kmh(obs.windSpeed?.value), dir: obs.windDirection?.value ?? null, gust: kmh(obs.windGust?.value), time: obs.timestamp };
}

function Compass({ dir, speed }: { dir: number; speed: number }) {
  // Arrow points where the wind blows TO; streak speed follows wind speed.
  const dur = Math.max(0.6, 3.2 - speed * 0.25);
  return <svg className="wind-compass" viewBox="-50 -50 100 100" role="img" aria-label={`Wind from ${Math.round(dir)} degrees at ${speed.toFixed(1)} metres per second`}>
    <circle r="44" className="compass-ring"/>
    {['N', 'E', 'S', 'W'].map((t, i) => <text key={t} x={[0, 36, 0, -36][i]} y={[-32, 4, 40, 4][i]} textAnchor="middle">{t}</text>)}
    <g transform={`rotate(${dir + 180})`}>
      {[-12, 0, 12].map(x => <line key={x} className="compass-streak" x1={x} x2={x} y1="22" y2="6" style={{ animationDuration: dur + 's' }}/>)}
      <path className="compass-arrow" d="M0 -30 L8 -14 L2 -16 L2 24 L-2 24 L-2 -16 L-8 -14 Z"/>
    </g>
  </svg>;
}

export default function LiveConditions({ latitude, longitude, altitude, onApply }:
  { latitude: number; longitude: number; altitude: number; onApply: (v: LiveApply) => void }) {
  const [now, setNow] = useState<Now | null>(null);
  const [station, setStation] = useState<Station | null>(null);
  const [stationNote, setStationNote] = useState('');
  const [error, setError] = useState('');
  const [busy, setBusy] = useState(false);
  const [terrain, setTerrain] = useState<Terrain>('open');
  const load = useCallback(async () => {
    if (!Number.isFinite(latitude) || !Number.isFinite(longitude)) return;
    setBusy(true); setError('');
    try { setNow(await fetchNow(latitude, longitude)); } catch (e) { setError(e instanceof TypeError ? 'Live weather unavailable: check the internet connection.' : e instanceof Error ? e.message : String(e)); }
    try { setStation(await fetchStation(latitude, longitude)); setStationNote(''); } catch (e) { setStation(null); setStationNote(e instanceof TypeError ? 'Nearby station unavailable right now' : e instanceof Error ? e.message : String(e)); }
    setBusy(false);
  }, [latitude, longitude]);
  useEffect(() => { load(); const id = setInterval(load, REFRESH_MS); return () => clearInterval(id); }, [load]);

  const uAlt = now ? windAtAltitude(now.speed, altitude, terrain) : 0;
  const turb = now ? dryden(now.speed, altitude, terrain) : null;
  const sigma = forecastSigma(terrain);
  const disagreement = now && station?.speed != null ? Math.abs(station.speed - now.speed) : null;
  return <section className="live-conditions" aria-label="Live conditions">
    <div className="live-head"><span className="live-dot" aria-hidden="true"/><strong>Live conditions</strong>
      <button className="icon-button" aria-label="Refresh live conditions" onClick={load} disabled={busy}><RefreshCw size={14} className={busy ? 'spin' : undefined}/></button></div>
    {error && <div className="hint warn" role="alert">{error}</div>}
    {now ? <>
      <div className="live-grid">
        <Compass dir={now.dir} speed={now.speed}/>
        <dl className="live-stats">
          <div><dt>Wind 10 m</dt><dd>{now.speed.toFixed(1)} m/s</dd></div>
          <div><dt>From</dt><dd>{Math.round(now.dir)}° {['N','NE','E','SE','S','SW','W','NW'][Math.round(now.dir/45)%8]}</dd></div>
          <div><dt>At {Math.round(altitude)} m</dt><dd>{uAlt.toFixed(1)} m/s</dd></div>
          <div><dt>Gusts</dt><dd>{now.gust.toFixed(1)} m/s</dd></div>
          <div><dt>Turbulence σ</dt><dd>{turb!.su.toFixed(2)} m/s</dd></div>
          <div><dt>Forecast error</dt><dd>±{sigma.toFixed(1)} m/s (1σ)</dd></div>
          <div><dt>Density alt.</dt><dd>{Math.round(densityAltitude(now.temp, now.pressure))} m</dd></div>
        </dl>
      </div>
      <label className="terrain-select">Surroundings
        <select value={terrain} onChange={e => setTerrain(e.target.value as Terrain)}>
          <option value="open">Open field</option><option value="mixed">Trees / crops</option><option value="complex">Buildings / forest</option>
        </select></label>
      <div className="station-row"><Radio size={13} aria-hidden="true"/>{station
        ? <span>Measured at {station.id}: {station.speed?.toFixed(1) ?? '—'} m/s{station.dir != null ? ` from ${Math.round(station.dir)}°` : ''}
            {disagreement != null && disagreement > sigma ? <b className="warn-text"> · disagrees with forecast by {disagreement.toFixed(1)} m/s</b> : ''}</span>
        : <span className="muted">{stationNote || 'Checking nearby station…'}</span>}</div>
      <button className="primary apply-live" onClick={() => onApply({ windSpeed: now.speed, windDirection: now.dir, gust: now.gust,
        temperature: now.temp, pressure: now.pressure, fetchedAt: new Date().toISOString(), forecastValidAt: now.time,
        source: 'Open-Meteo current conditions · 10 m modelled wind' })}><CloudSun size={15}/> Use these conditions</button>
      <div className="attribution">Weather: <a href="https://open-meteo.com/" target="_blank" rel="noreferrer">Open-Meteo</a> (CC BY 4.0) · Observations: NOAA/NWS · updated {new Date(now.time).toLocaleTimeString()}</div>
    </> : !error && <div className="hint">Loading current conditions…</div>}
  </section>;
}
