import {useEffect, useRef, useState} from 'react';
import {buildRequest, exportBlob, getPreviewSummary} from './engineering';
import type {Project, Preview} from './types';

type Row = {
  requested: number;
  status: 'verified'|'failed';
  peak?: number;
  duration?: number;
  reason: string;
  digest?: unknown;
  geometryStrategy?: unknown;
};
type Props = {project: Project; disabled: boolean; onBusy: (value: boolean) => void};

async function evaluateSpeed(project:Project, requested:number, signal:AbortSignal):Promise<Row>{
  const response=await fetch('/api/preview',{
    method:'POST', headers:{'Content-Type':'application/json'}, signal,
    body:JSON.stringify(buildRequest({...project, cruise:requested}))
  });
  const result:Preview=await response.json();
  if(!response.ok||!result.accepted)throw Error(result.error||'Planner unavailable');
  const {speed}=getPreviewSummary(result);
  const peak=Number(speed.planned_route_peak_sampled_m_s);
  const duration=Number(speed.route_duration_s);
  if(!Number.isFinite(peak)||!Number.isFinite(duration))throw Error('Missing verified speed or duration');
  return {
    requested,status:'verified',peak,duration,digest:result.summary?.controller_trajectory_sha256,
    geometryStrategy:result.planning_adjustments?.method,
    reason:Array.isArray(speed.reasons)?speed.reasons.join(', ').replaceAll('_',' '):''
  };
}

export default function Sweep({project,disabled,onBusy}:Props){
  const [speeds,setSpeeds]=useState('1, 5, 25, 50');
  const [scenario,setScenario]=useState<Project|null>(null);
  const [rows,setRows]=useState<Row[]>([]);
  const [error,setError]=useState('');
  const [running,setRunning]=useState(false);
  const controller=useRef<AbortController|null>(null);
  useEffect(()=>()=>{controller.current?.abort();onBusy(false);},[onBusy]);

  async function run(){
    const requests=speeds.split(',').map(Number);
    if(requests.some(value=>!Number.isFinite(value)||value<=0)){
      setError('Enter comma-separated positive speeds.');
      return;
    }
    const job=new AbortController();
    controller.current=job;
    setScenario(project);
    setRows([]);
    setError('');
    onBusy(true);
    setRunning(true);
    try{
      for(const requested of requests){
        let row:Row;
        try{
          row=await evaluateSpeed(project,requested,job.signal);
        }catch(e){
          row={requested,status:'failed',reason:e instanceof Error?e.message:String(e)};
        }
        if(job.signal.aborted)break;
        setRows(old=>[...old,row]);
      }
    }finally{
      controller.current=null;
      setRunning(false);
      onBusy(false);
    }
  }

  function save(){
    const evidence={schema:'trajectory-studio.sweep.v1',solverSourceSha256:import.meta.env.VITE_SOLVER_DIGEST,
      scope:'offline route predictions',project:scenario,requests:speeds.split(',').map(Number),rows};
    exportBlob('speed-sweep.json',JSON.stringify(evidence,null,2),'application/json');
  }
  return <section className="data-section"><h3>Compare requested speeds</h3>
    <label>Requests (m/s)<input value={speeds} disabled={running} onChange={e=>{setSpeeds(e.target.value);setRows([]);}}/></label>
    <button disabled={disabled||running} onClick={run}>Run speed sweep</button>
    {running&&<button onClick={()=>controller.current?.abort()}>Cancel sweep</button>}
    {error&&<p role="alert">{error}</p>}
    {rows.length>0&&<><table className="point-table"><thead><tr>
      {['Requested (m/s)','Planned peak (m/s)','Route time (s)','Limits or error'].map(k=><th key={k}>{k}</th>)}
    </tr></thead><tbody>{rows.map((r,i)=><tr key={i}>
      <td>{r.requested}</td><td>{r.peak?.toFixed(2)??'—'}</td><td>{r.duration?.toFixed(2)??'—'}</td><td>{r.reason}</td>
    </tr>)}</tbody></table><button onClick={save}>Export sweep</button></>}
  </section>;
}
