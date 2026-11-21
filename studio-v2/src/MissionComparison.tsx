import {useRef,useState} from 'react';
import {exportBlob} from './engineering';
import {compareMission,limits,syntheticRun} from './missionReport';
import type {Receipt} from './missionReport';
import {handoffFixture} from './rehearsalHandoff';
import type {Point,Preview} from './types';
import routeUrl from '../../session_2_robust_landing/05_Evidence/Post_Disarm_Proof/trajectory.csv?url';
import recordingUrl from '../../validation/2026-10-03/cases/raised-10m-final/telemetry_and_truth.csv.gz?url';
import archivedManifest from '../../validation/2026-10-03/cases/raised-10m-final/run_manifest_20261003T062757218606Z.json';
type Report=Awaited<ReturnType<typeof compareMission>>;

async function recordingText(blob:Blob,gzip:boolean){
  if(blob.size>64*1024**2)throw Error('Select a recording up to 64 MiB.');
  const stream=gzip?blob.stream().pipeThrough(new DecompressionStream('gzip')):blob.stream();
  const reader=stream.getReader(),decoder=new TextDecoder('utf-8',{fatal:true,ignoreBOM:true});
  const text:string[]=[];let total=0;
  try{
    for(;;){const {value,done}=await reader.read();if(done)break;total+=value.byteLength;
      if(total>64*1024**2)throw Error('Expanded recording exceeds 64 MiB.');
      text.push(decoder.decode(value,{stream:true}));}
    text.push(decoder.decode());return text.join('');
  }finally{await reader.cancel();reader.releaseLock();}
}
async function asset(url:string,gzip=false){
  const res=await fetch(url);
  if(!res.ok)throw Error('Archived example could not load.');
  return recordingText(await res.blob(),gzip);
}
export default function MissionComparison({result,points}:{result:Preview|null;points:Point[]}){
  const [report,setReport]=useState<Report|null>(null),[error,setError]=useState(''),[busy,setBusy]=useState(false);
  const [label,setLabel]=useState(''),[radius,setRadius]=useState('5');
  const [trace,setTrace]=useState<Awaited<ReturnType<typeof handoffFixture>>|null>(null);
  const [log,setLog]=useState<File|null>(null),[receipt,setReceipt]=useState<File|null>(null);
  const generation=useRef(0);
  const route=result?.trajectory_csv;
  function invalidate(){generation.current++;setReport(null);setTrace(null);setError('');setBusy(false);}
  async function run(mode:'archive'|'fixture'|'import'|'handoff'){
    const seq=++generation.current;setBusy(true);setError('');setReport(null);setTrace(null);
    try{
      const policy={...limits,landingRadiusM:Number(radius)};
      let r:Report,title:string;
      if(mode==='handoff'){
        if(!route)throw Error('Verify a route first.');
        const test=await handoffFixture(route,String(result?.summary?.controller_trajectory_sha256));
        if(seq===generation.current)setTrace(test);return;
      }else if(mode==='archive'){
        const [csv,recording]=await Promise.all([asset(routeUrl),asset(recordingUrl,true)]);
        const manifest={...archivedManifest,source:'retained_sitl',
          telemetry_sha256:'129b0b24f49291b1668d546da65e5be0397e85e8891172fea7e279b5fe4bcd6d',
          landing_target_local_ne_m:[0.0375346475,0.0921971667] as [number,number]};
        r=await compareMission(csv,recording,manifest,null,policy);title='Archived X500 · raised landing surface';
      }else{
        if(!route)throw Error('Verify the active route first.');
        if(mode==='fixture'){
          const mock=await syntheticRun(route);
          r=await compareMission(route,mock.telemetry,mock.receipt,points,policy);title='Synthetic run of the active route';
        }else{
          if(!log)throw Error('Select aligned telemetry CSV.');
          let manifest:Receipt|null=null;
          if(receipt){
            if(receipt.size>2*1024**2)throw Error('The run manifest exceeds 2 MiB.');
            manifest=JSON.parse(await receipt.text()) as Receipt;
            if(!manifest||typeof manifest!=='object'||Array.isArray(manifest))throw Error('Expected a run manifest object.');
          }
          r=await compareMission(route,await recordingText(log,log.name.toLowerCase().endsWith('.gz')),manifest,points,policy);
          title=log.name;
        }
      }
      if(seq===generation.current){setReport(r);setLabel(title);}
    }catch(e){if(seq===generation.current)setError((e as Error).message);}
    finally{if(seq===generation.current)setBusy(false);}
  }
  return <section className="data-section"><h3>Mission comparison</h3>
    <p>Compare an exact route CSV with independent position measurements, the route clock, required stops and the landing target.</p>
    <label>Landing radius (m)<input type="number" step="any" value={radius} onChange={e=>{invalidate();setRadius(e.target.value);}}/></label>
    <button disabled={busy} onClick={()=>run('archive')}>Review archived X500 run</button>{' '}
    <button disabled={busy||!route} onClick={()=>run('fixture')}>Test report with synthetic run</button>{' '}
    <button disabled={busy||!route} onClick={()=>run('handoff')}>Test simulator handoff offline</button>
    <details><summary>Compare an imported run with this plan</summary>
      <label>Aligned telemetry CSV or CSV.gz <input type="file" accept=".csv,.gz" onChange={e=>{invalidate();setLog(e.target.files?.[0]??null);}}/></label>
      <label>Run manifest (optional) <input type="file" accept=".json,application/json" onChange={e=>{invalidate();setReceipt(e.target.files?.[0]??null);}}/></label>
      <button disabled={busy||!log||!route} onClick={()=>run('import')}>Compare imported run</button>
      <p>Use aligned NED truth, logged commands and a mission clock. A scored run needs both the controller’s route digest and the telemetry digest. Estimator-only ULogs can be viewed in Recorded flight replay.</p>
    </details>
    {error&&<p role="alert">{error}</p>}
    {trace&&<p role="status">Synthetic handoff · {trace.state} · no simulator ran. The client checked submission, status and the controller receipt for this route.</p>}
    {report&&<><h4>{label}</h4><p>{report.evidence} · {report.verdict} · route identity from {report.identity}</p>
      <table className="point-table"><thead><tr><th>Check</th><th>Result</th><th>Measurement</th></tr></thead>
        <tbody>{report.checks.map(c=><tr key={c.name}><td>{c.name}</td><td>{c.verdict}</td><td>{c.detail}</td></tr>)}</tbody></table>
      {report.visits.length>0&&<details><summary>Individual stops ({report.visits.length})</summary><table className="point-table"><thead><tr>
        {['Point','Result','Planned (s)','Error (m)','Speed (m/s)'].map(x=><th key={x}>{x}</th>)}</tr></thead><tbody>
        {report.visits.map((v,i)=><tr key={i}><td>{v.point}</td><td>{v.verdict}</td><td>{v.plannedS?.toFixed(2)??'—'}</td>
        <td>{v.errorM?.toFixed(3)??'—'}</td><td>{v.speedMs?.toFixed(3)??'—'}</td></tr>)}</tbody></table></details>}
      <p>Study criteria use {report.policy.trackingRmsM} m tracking RMS, ±{report.policy.timingFraction*100}% elapsed route time, and a {report.policy.stopRadiusM} m stop radius at ≤{report.policy.stopSpeedMs} m/s.</p>
      {report.evidence==='retained simulator recording'&&report.runId===archivedManifest.run_id&&<p>The archive identifies the input CSV. Its manifest predates runtime route receipts. The target is the final recorded horizontal landing setpoint.</p>}
      <button onClick={()=>exportBlob('mission-comparison.json',JSON.stringify({title:label,...report},null,2),'application/json')}>Export mission comparison</button>
    </>}
  </section>;
}
