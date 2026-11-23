import { useState } from 'react';
import { ShieldCheck, ShieldAlert, ShieldX } from 'lucide-react';
import { LEVEL_TEXT, type Corridor } from './fidelity';

export default function ConfidenceBadge({ corridor, onEvidence }: { corridor: Corridor; onEvidence: () => void }) {
  const [open, setOpen] = useState(false);
  const Icon = corridor.level === 'calibrated' ? ShieldCheck : corridor.level === 'extrapolating' ? ShieldAlert : ShieldX;
  return <div className="confidence-wrap">
    <button className={`confidence-badge ${corridor.level}`} aria-expanded={open} aria-haspopup="dialog"
      onClick={() => setOpen(o => !o)} title="How far the real aircraft is expected to stray from this plan">
      <Icon size={15} aria-hidden="true"/><span>{LEVEL_TEXT[corridor.level]}</span><b>±{corridor.radius.toFixed(1)} m</b>
    </button>
    {open && <div className="confidence-pop" role="dialog" aria-label="Prediction confidence">
      <strong>Expected path accuracy: ±{corridor.radius.toFixed(1)} m (95%)</strong>
      <ul>{corridor.reasons.map(r => <li key={r}>{r}</li>)}</ul>
      <div className="split-row"><span>Tracking (real flights)</span><b>±{corridor.tracking.toFixed(2)} m</b></div>
      <div className="split-row"><span>Wind gusts (modelled)</span><b>±{corridor.wind.toFixed(2)} m</b></div>
      <button onClick={() => { setOpen(false); onEvidence(); }}>See the evidence</button>
    </div>}
  </div>;
}
