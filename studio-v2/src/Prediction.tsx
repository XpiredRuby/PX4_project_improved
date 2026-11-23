import {useState} from 'react';
import {exportBlob} from './engineering';
import {predictTrack} from './prediction';
import type {AircraftLimits} from './prediction';
import type {Environment,Sample} from './types';

type Props={samples:Sample[];environment:Environment;vehicle:AircraftLimits;digest:unknown};
export default function Prediction({samples,environment,vehicle,digest}:Props){
  const [area,setArea]=useState('0.08'),[response,setResponse]=useState('0.8');
  const [result,setResult]=useState<ReturnType<typeof predictTrack>|null>(null);
  const [error,setError]=useState('');
  function run(){
    try{
      if(!area.trim()||!response.trim())throw Error('Enter both model assumptions.');
      setResult(predictTrack(samples,environment,vehicle,{dragArea:Number(area),responseTime:Number(response)}));
      setError('');
    }catch(e){setResult(null);setError((e as Error).message);}
  }
  function save(){
    if(!result)return;
    exportBlob('predicted-track.json',JSON.stringify({model:'point-mass-tracking-v1',scope:'uncalibrated offline prediction',
      routeSha256:digest,environment,vehicle,assumptions:{dragAreaM2:Number(area),responseTimeS:Number(response)},...result},null,2),'application/json');
  }
  const stride=result?Math.max(1,Math.ceil(result.points.length/1000)):1;
  const path=result?.points.flatMap((p,i)=>{
    if(i%stride!==0&&i!==result.points.length-1)return [];
    const fraction=(p.time-samples[0].time)/(samples[samples.length-1].time-samples[0].time);
    return [(i?'L':'M')+(5+fraction*620).toFixed(2)+' '+(85-80*p.error/Math.max(.01,result.maxError)).toFixed(2)];
  }).join(' ');
  return <section className="data-section"><h3>Predicted tracking under wind</h3>
    <p>A point-mass model follows the verified route with limited thrust and tilt. Wind creates quadratic drag. Drag area and controller response are research assumptions for this run.</p>
    <div className="form-row"><label>Drag area CdA (m²)<input type="number" min="0" step="any" value={area} onChange={e=>{setArea(e.target.value);setResult(null);}}/></label>
      <label>Response time (s)<input type="number" min="0" step="any" value={response} onChange={e=>{setResponse(e.target.value);setResult(null);}}/></label></div>
    <button onClick={run}>Predict tracking</button>
    {error&&<p role="alert">{error}</p>}
    {result&&<><p>Predicted 3D tracking RMS {result.rms.toFixed(3)} m · peak {result.maxError.toFixed(3)} m · time at thrust or tilt limit {result.limitedTime.toFixed(2)} s</p>
      <svg viewBox="0 0 630 90" aria-label="Predicted tracking error over route time" style={{width:'100%'}}><path d={path} stroke="var(--trace)" fill="none" strokeWidth="2"/></svg>
      <small>Route time {samples[0].time.toFixed(1)}–{samples[samples.length-1].time.toFixed(1)} s · error scale 0–{Math.max(.01,result.maxError).toFixed(2)} m</small>
      <p><button onClick={save}>Export predicted positions</button></p></>}
  </section>;
}
