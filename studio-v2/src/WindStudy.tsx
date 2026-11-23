import {useMemo, useState} from 'react';
import {exportBlob, metrics} from './engineering';
import type {Environment, Sample} from './types';

export default function WindStudy({samples,environment,digest}:{samples:Sample[];environment:Environment;digest:unknown}){
  const [speed,setSpeed]=useState(5);
  const rows=useMemo(()=>{
    if(!Number.isFinite(speed)||speed<0||!samples.length)return [];
    return ['Calm','North','East','South','West'].map((label,index)=>{
      const windSpeed=index?speed:0, fromDegrees=index?90*(index-1):0;
      const values=metrics(samples,{...environment,mode:'manual',windSpeed,windDirection:fromDegrees});
      return {from:label,fromDegrees,windSpeed,peakAirspeed:values!.air};
    });
  },[samples,environment,speed]);
  if(!samples.length)return null;
  return <details className="data-section"><summary>Compare steady winds on this route</summary>
    <label>Study wind speed (m/s)<input type="number" min="0" step="any" value={Number.isFinite(speed)?speed:''} onChange={e=>setSpeed(e.target.valueAsNumber)}/></label>
    <p>Each case uses the same verified route and a uniform horizontal wind from the indicated direction.</p>
    {rows.length>0&&<><table className="point-table"><thead><tr><th>Wind from</th><th>Wind (m/s)</th><th>Peak airspeed (m/s)</th></tr></thead><tbody>
      {rows.map(r=><tr key={r.from}><td>{r.from}</td><td>{r.windSpeed}</td><td>{r.peakAirspeed.toFixed(2)}</td></tr>)}</tbody></table>
      <button onClick={()=>exportBlob('steady-wind-study.json',JSON.stringify({schema:'trajectory-studio.wind-study.v1',referenceSha256:digest,scope:'steady horizontal wind applied to route reference velocities',rows},null,2),'application/json')}>Export wind study</button></>}
  </details>;
}
