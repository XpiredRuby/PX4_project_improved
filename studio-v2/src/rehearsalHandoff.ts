import {sha256} from './missionReport.ts';

export type Binding={plan_id:string;controller_trajectory_sha256:string};
type Job=Binding & {stage:string;cancel_confirmed?:boolean;error?:string;assessment?:{
  sampled_rehearsal_passed?:boolean;checks?:{name:string;passed:boolean;detail?:{
    sha256?:string; selected_sha256?:string; controller_sha256?:string}}[]}};
export type Transport=(path:string,payload:object|null,signal:AbortSignal)=>Promise<Job>;
const digestCheck='Executed CSV digest matches retained plan and runner manifest';
export const localRehearsalTransport:Transport=async(path,payload,signal)=>{
  const res=await fetch('/api/rehearsal'+path,{method:payload?'POST':'GET',signal,
    ...(payload?{headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}:{cache:'no-store'})});
  const data=await res.json();
  if(!res.ok)throw Error(data.error||'Rehearsal request failed');
  return data;
};
export async function checkHandoff(input:{routeCsv:string;binding:Binding},send:Transport,
    {signal=new AbortController().signal,timeoutMs=30000,pollMs=300}:{signal?:AbortSignal;timeoutMs?:number;pollMs?:number}={}){
  const binding=input.binding,digest=await sha256(input.routeCsv);
  if(!/^[a-f0-9]{32}$/.test(binding.plan_id)||digest!==binding.controller_trajectory_sha256)
    throw Error('Reviewed plan identity differs from the route bytes');
  const controller=new AbortController();
  const cancel=()=>controller.abort();
  const timer=setTimeout(cancel,timeoutMs);
  signal.addEventListener('abort',cancel,{once:true});
  if(signal.aborted)cancel();
  let submitted=false;
  const match=(job:Job)=>{
    if(job.plan_id!==binding.plan_id||job.controller_trajectory_sha256!==digest)throw Error('Status belongs to a different plan');
    return job;
  };
  try{
    controller.signal.throwIfAborted();
    submitted=true;
    let job=match(await send('',{...binding,confirm_interpretation:true},controller.signal));
    while(['STARTING','RUNNING'].includes(job.stage)){
      await new Promise<void>((resolve,reject)=>{
        const done=()=>{controller.signal.removeEventListener('abort',aborted);resolve();};
        const timer=setTimeout(done,pollMs);
        function aborted(){clearTimeout(timer);reject(Error('Handoff interrupted'));}
        if(controller.signal.aborted)aborted();else controller.signal.addEventListener('abort',aborted,{once:true});
      });
      job=match(await send('/'+binding.plan_id,null,controller.signal));
    }
    controller.signal.throwIfAborted();
    if(job.stage==='CANCELLED')return {state:job.cancel_confirmed===true?'cancelled':'outcome unknown',digest};
    if(job.stage==='FAILED')return {state:'failed',digest};
    if(job.stage!=='COMPLETED')throw Error('Unknown rehearsal stage '+job.stage);
    const proof=job.assessment?.checks?.find(c=>c.name===digestCheck);
    const detail=proof?.detail;
    const verified=job.assessment?.sampled_rehearsal_passed===true&&proof?.passed===true&&
      detail?.sha256===digest&&detail.selected_sha256===digest&&detail.controller_sha256===digest;
    return {state:verified?'assessment matched':'execution unverified',digest};
  }catch(e){
    if(!controller.signal.aborted)throw e;
    let confirmed=false;
    if(submitted){
      const stop=new AbortController(),stopTimer=setTimeout(()=>stop.abort(),Math.min(timeoutMs,3000));
      try{
        const ack=match(await send('/cancel',binding,stop.signal));
        confirmed=ack.stage==='CANCELLED'&&ack.cancel_confirmed===true;
      }catch{/* No confirmed terminal evidence was received. */}finally{clearTimeout(stopTimer);}
    }
    return {state:confirmed?'cancelled':submitted?'outcome unknown':'submission skipped',
      reason:signal.aborted?'cancel requested':'deadline exceeded',digest};
  }finally{clearTimeout(timer);signal.removeEventListener('abort',cancel);}
}

export async function handoffFixture(csv:string,digest:string){
  const binding={plan_id:'00000000000000000000000000000001',controller_trajectory_sha256:digest};
  const assessment={sampled_rehearsal_passed:true,checks:[{name:digestCheck,passed:true,
    detail:{sha256:digest,selected_sha256:digest,controller_sha256:digest}}]};
  const trace:string[]=[];
  const reply:Transport=async(path,payload)=>{
    trace.push(path||'submit');
    if(path===''){
      if(JSON.stringify(payload)!==JSON.stringify({...binding,confirm_interpretation:true}))throw Error('Start payload is invalid');
      return {...binding,stage:'STARTING'};
    }
    if(path!=='/'+binding.plan_id||payload!==null)throw Error('Unexpected protocol request');
    return {...binding,stage:'COMPLETED',assessment};
  };
  return {source:'synthetic protocol fixture',simulationRan:false,trace,
    ...await checkHandoff({routeCsv:csv,binding},reply,{pollMs:0})};
}
