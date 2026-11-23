import {useEffect, useState} from 'react';
import { CircleCheck, Play, ShieldCheck } from 'lucide-react';
import {assessmentMatches,localRehearsalTransport,type Binding,type Job} from './rehearsalHandoff';
import type {Preview} from './types';

const key='px4-studio-last-rehearsal';
function saved():Binding|null{
  try{
    const value=JSON.parse(sessionStorage.getItem(key)||'null');
    return /^[a-f0-9]{32}$/.test(value?.plan_id)&&/^[a-f0-9]{64}$/.test(value?.controller_trajectory_sha256)?value:null;
  }catch{return null;}
}
export default function ConnectedRehearsal({plan,onJob}:{plan:Preview|null;onJob:(job:Job)=>void}){
  const [binding,setBinding]=useState<Binding|null>(saved);
  const [job,setJob]=useState<Job|null>(null);
  const [pending,setPending]=useState(false);
  const [error,setError]=useState('');
  useEffect(()=>{
    if(!binding)return;
    const controller=new AbortController();
    let timer:ReturnType<typeof setTimeout>;
    async function poll(){
      try{
        const result=await localRehearsalTransport('/'+binding!.plan_id,null,controller.signal);
        if(result.plan_id!==binding!.plan_id||result.controller_trajectory_sha256!==binding!.controller_trajectory_sha256)
          throw Error('The simulator returned a different plan.');
        setJob(result);onJob(result);setError('');
        if(!['STARTING','RUNNING'].includes(result.stage))return;
      }catch(e){
        if(controller.signal.aborted)return;
        setError(String(e));
        if(e instanceof Error&&e.message.includes('plan is unavailable')){
          sessionStorage.removeItem(key);setBinding(null);setJob(null);return;
        }
      }
      timer=setTimeout(poll,1500);
    }
    void poll();
    return ()=>{controller.abort();clearTimeout(timer);};
  },[binding,onJob]);
  const running=!!binding&&(!job||['STARTING','RUNNING'].includes(job.stage));
  async function submit(cancel:boolean){
    const target=cancel?binding:plan?.plan_id&&typeof plan.summary?.controller_trajectory_sha256==='string'
      ?{plan_id:plan.plan_id,controller_trajectory_sha256:plan.summary.controller_trajectory_sha256}:null;
    if(!target)return;
    setPending(true);setError('');
    try{
      const result=await localRehearsalTransport(cancel?'/cancel':'',cancel?target:{...target,confirm_interpretation:true},new AbortController().signal);
      if(result.plan_id!==target.plan_id||result.controller_trajectory_sha256!==target.controller_trajectory_sha256)throw Error('Simulator response has a different trajectory.');
      setJob(result);onJob(result);setBinding({...target});sessionStorage.setItem(key,JSON.stringify(target));
    }catch(e){setError(String(e));}finally{setPending(false);}
  }
  const passed=job?.stage==='COMPLETED'&&assessmentMatches(job,job.controller_trajectory_sha256);
  const status=passed?'Verified rehearsal passed'
    :job?.stage==='FAILED'?'Rehearsal failed'
    :job?.stage==='CANCELLED'?'Rehearsal cancelled'
    :job?.stage==='COMPLETED'?'Review the simulator checks'
    :job?.cancel_requested?'Landing requested'
    :job?.stage==='STARTING'?'Starting simulator':'Simulation in progress';
  return <section className="data-section rehearsal-card" aria-labelledby="rehearsal-title">
    <div className="section-heading">
      <div><strong id="rehearsal-title">Test in Gazebo</strong><span>Run a recorded flight in the local simulator.</span></div>
      <div className="section-actions">
        <button className="simulate-button" disabled={pending||running||!plan?.simulation_compatible||!plan.plan_id||plan.plan_id===job?.plan_id}
          onClick={()=>void submit(false)}><Play size={16}/> Run verified route in Gazebo</button>
        {running&&<button className="landing-button" disabled={pending||job?.cancel_requested} onClick={()=>void submit(true)}><ShieldCheck size={16}/> Request landing</button>}
      </div>
    </div>
    {!job&&!plan&&<p className="rehearsal-status">Build a route and choose Check route to continue.</p>}
    {job&&<p className={`rehearsal-status ${passed?'passed':''}`} role="status">
      {passed&&<CircleCheck size={18}/>} <strong>{status}</strong> · Run {job.plan_id.slice(0,8)} {running&&job.phase? '· '+job.phase.replaceAll('_',' ').toLowerCase():''}
    </p>}
    {job?.assessment&&<details><summary>View simulator checks</summary><ul>{job.assessment.checks?.map(c=><li key={c.name}>{c.passed?'Pass':'Fail'} · {c.name}</li>)}</ul></details>}
    {(error||job?.error)&&<p className="rehearsal-status rehearsal-error" role="alert">{error||job?.error}</p>}
  </section>;
}
