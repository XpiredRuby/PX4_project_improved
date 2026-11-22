import {useEffect, useState} from 'react';
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
      }catch(e){if(controller.signal.aborted)return;setError(String(e));}
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
  return <section className="data-section"><h3>PX4 / Gazebo rehearsal</h3>
    <button disabled={pending||running||!plan?.simulation_compatible||!plan.plan_id||plan.plan_id===job?.plan_id}
      onClick={()=>void submit(false)}>Run verified route in Gazebo</button>
    {running&&<button disabled={pending||job?.cancel_requested} onClick={()=>void submit(true)}>Request landing</button>}
    {job&&<p role="status">Run {job.plan_id.slice(0,8)} · {job.stage==='COMPLETED'&&assessmentMatches(job,job.controller_trajectory_sha256)?'Verified rehearsal passed':job.stage} {job.phase||''}</p>}
    {job?.assessment&&<details><summary>View simulator checks</summary><ul>{job.assessment.checks?.map(c=><li key={c.name}>{c.passed?'Pass':'Fail'} · {c.name}</li>)}</ul></details>}
    {(error||job?.error)&&<p role="alert">{error||job?.error}</p>}
  </section>;
}
