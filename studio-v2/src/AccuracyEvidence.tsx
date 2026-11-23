import evidence from '../../config/fidelity/x500_evidence.json';
import errorPlot from '../../validation/2026-11-23-x500-public-logs/x500_prediction_error.png';
import dragPlot from '../../validation/2026-11-23-x500-public-logs/x500_drag_real_vs_sim.png';
import windPlot from '../../validation/2026-11-23-measured-wind/measured_wind.png';

import GnssWhatIf from './GnssWhatIf';

type Row = { label: string; value: string; note?: string };
const ev = evidence as unknown as { headline: Row[]; credibility: { factor: string; score: number; why: string }[]; limits: string[]; sources: { name: string; url: string }[] };

export default function AccuracyEvidence({ windAtAlt = 0, altitude = 12 }: { windAtAlt?: number; altitude?: number }) {
  return <div className="evidence-panel">
    <h3>How accurate is this prediction?</h3>
    <p className="muted">Scored on real Holybro X500 V2 flights that the model never saw. The reference is each aircraft's own GPS-aided estimate, not independent ground truth.</p>
    <div className="evidence-cards">{ev.headline.map(r => <div key={r.label} className="evidence-card"><span>{r.label}</span><b>{r.value}</b>{r.note && <small>{r.note}</small>}</div>)}</div>
    <div className="evidence-figures">
      <figure><img src={errorPlot} alt="Prediction error growth on held-out real flights"/><figcaption>Prediction error vs time, held-out real flights</figcaption></figure>
      <figure><img src={windPlot} alt="Drag against measured airspeed and route deviation against measured turbulence"/><figcaption>Measured wind (onboard anemometer, 196 flights)</figcaption></figure>
      <figure><img src={dragPlot} alt="Drag per mass of the real fleet compared with the simulator"/><figcaption>Real fleet vs simulator drag</figcaption></figure>
    </div>
    <GnssWhatIf windAtAlt={windAtAlt} altitude={altitude}/>
    <h4>Credibility (NASA-STD-7009A style, 0–4)</h4>
    <table className="credibility"><tbody>{ev.credibility.map(c => <tr key={c.factor}><th>{c.factor}</th>
      <td><span className="meter" aria-label={`${c.score} of 4`}>{[0, 1, 2, 3].map(i => <i key={i} className={i < c.score ? 'on' : ''}/>)}</span></td><td>{c.why}</td></tr>)}</tbody></table>
    <h4>Not claimed</h4>
    <ul>{ev.limits.map(l => <li key={l}>{l}</li>)}</ul>
    <div className="attribution">Data: {ev.sources.map((s, i) => <span key={s.url}>{i ? ' · ' : ''}<a href={s.url} target="_blank" rel="noreferrer">{s.name}</a></span>)}</div>
  </div>;
}
