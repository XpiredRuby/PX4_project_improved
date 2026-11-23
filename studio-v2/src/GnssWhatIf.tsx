import { useState } from 'react';
import { SatelliteDish } from 'lucide-react';
import { gnssOutage } from './fidelity';

export default function GnssWhatIf({ windAtAlt, altitude }: { windAtAlt: number; altitude: number }) {
  const [outage, setOutage] = useState(12);
  const r = gnssOutage(outage, windAtAlt, altitude);
  const level = !r.failsafe ? 'calibrated' : r.driftRealM > 10 ? 'uncalibrated' : 'extrapolating';
  return <section className="gnss-whatif" aria-label="GNSS outage what-if">
    <h4><SatelliteDish size={15} aria-hidden="true"/> What if GNSS drops?</h4>
    <label className="gnss-slider">Outage length <b>{outage} s</b>
      <input type="range" min={1} max={30} value={outage} onChange={e => setOutage(Number(e.target.value))}/></label>
    <div className={`gnss-result ${level}`}>
      {r.failsafe
        ? <>PX4 failsafe at {r.failsafeAtS} s · expected drift <b>{r.driftSimM.toFixed(0)} m</b> in Gazebo, <b>~{r.driftRealM.toFixed(0)} m</b> on a real X500</>
        : <>No failsafe · drift about <b>{r.driftRealM.toFixed(1)} m</b></>}
      <span className="drift-bar" style={{ ['--drift' as string]: Math.min(1, r.driftRealM / 60) }} aria-hidden="true"/>
    </div>
    <ol className="gnss-steps">{r.steps.map(s => <li key={s}>{s}</li>)}</ol>
    <small className="muted">Wind {windAtAlt.toFixed(1)} m/s at {Math.round(altitude)} m. Checked against three Studio SITL outage cases;
      the real-aircraft figure scales the wind response by measured real-fleet drag and has not been flight-tested.</small>
  </section>;
}
