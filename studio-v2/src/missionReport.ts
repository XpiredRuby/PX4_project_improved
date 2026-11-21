import {parseCsv} from './engineering.ts';
import type {Point, Sample} from './types';

export type Vec=[number,number,number];
type Pair=[number,number];
export type Receipt={
  run_id?:string; source?:string; telemetry_sha256?:string;selected_trajectory_sha256?:string;
  controller_runtime?:{trajectory_sha256?:string;[name:string]:unknown}; source_sha256?:Record<string,string>;
  landing_target_local_ne_m?:Pair;
};
export const limits={trackingRmsM:0.5, stopRadiusM:0.5, stopSpeedMs:0.25, stopWindowS:1,
  timingFraction:0.10, landingRadiusM:5, maxGapS:0.25, clockEndpointS:0.10, referenceAgreementM:0.01};
type Limits=typeof limits;
type Verdict='pass'|'fail'|'insufficient'|'not applicable';
export type Check={name:string; verdict:Verdict; detail:string};
export const sha256=async(text:string)=>Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256',
  new TextEncoder().encode(text))),n=>n.toString(16).padStart(2,'0')).join('');
export async function openSavedCase(raw:string){
  const file=JSON.parse(raw);
  if(file?.schema!=='trajectory-studio.case.v1'||file.result?.accepted!==true||
     typeof file.result.trajectory_csv!=='string'||typeof file.project?.title!=='string')
    throw Error('Select a verified JSON file from Download case.');
  const route=file.result.trajectory_csv as string,digest=await sha256(route);
  if(digest!==file.metrics?.route_sha256||digest!==file.result.summary?.controller_trajectory_sha256)
    throw Error('Saved case route identity mismatch.');
  const points=file.request?.document?.path?.points;
  if(!Array.isArray(points)||points.length<2)throw Error('Saved case is missing its waypoint request.');
  const declared:Point[]=points.map((p:Record<string,unknown>)=>{
    if(!p||typeof p.point_id!=='string'||typeof p.hard!=='boolean'||
       ![p.north_m,p.east_m,p.down_m].every(Number.isFinite))throw Error('Invalid saved stop declaration.');
    return {id:p.point_id,north:p.north_m as number,east:p.east_m as number,altitude:-(p.down_m as number),
      hard:p.hard,exact:p.exact===true};
  });
  return {route,digest,points:declared,title:file.project.title as string};
}
const pos=(s:Sample):Vec=>[s.north,s.east,-s.altitude];
const dist=(a:number[],b:number[])=>Math.hypot(...a.map((x,i)=>x-b[i]));
const hasPair=(v:unknown):v is Pair=>Array.isArray(v)&&v.length===2&&v.every(Number.isFinite);
function table(raw:string){
  const [header,...lines]=raw.trim().split(/\r?\n/);
  const fields=header.replace(/^\uFEFF/,'').split(',').map(x=>x.trim());
  if(new Set(fields).size!==fields.length||!fields.includes('elapsed_s')||!fields.includes('phase'))
    throw Error('Select aligned NED telemetry with elapsed_s and phase columns.');
  const clock=fields.includes('mission_time_s')?'mission_time_s':'phase_clock_s';
  const idx=new Map(fields.map((key,i)=>[key,i]));
  let previous=-Infinity;
  const rows=lines.map((line,i)=>{
    const cells=line.split(',');
    if(cells.length!==fields.length)throw Error('Invalid column count at row '+(i+2));
    const field=(key:string)=>cells[idx.get(key)??-1]?.trim();
    const num=(key:string)=>{const v=field(key);return v&&Number.isFinite(Number(v))?Number(v):null;};
    const vec=(prefix:string):Vec|null=>{
      const v=['x','y','z'].map(k=>num(prefix+'_'+k));
      return v.every(n=>n!==null)?v as Vec:null;
    };
    const elapsed=num('elapsed_s');
    if(elapsed===null||elapsed<=previous)throw Error('Elapsed time must increase at row '+(i+2));
    previous=elapsed;
    const a=num('native_landing_reference_x'),b=num('native_landing_reference_y');
    return {elapsed,phase:field('phase'),clock:num(clock),desired:vec('desired'),truth:vec('truth'),
      segment:field('segment')??'0', landed:num('landed_state'),target:a===null||b===null?null:[a,b] as Pair};
  });
  return {rows,clock};
}
function positionAt(plan:Sample[],t:number):Vec{
  if(t<=plan[0].time)return pos(plan[0]);
  if(t>=plan.at(-1)!.time)return pos(plan.at(-1)!);
  let a=0,b=plan.length-1;
  while(b-a>1){const m=(a+b)>>1;if(plan[m].time<=t)a=m;else b=m;}
  const f=(t-plan[a].time)/(plan[b].time-plan[a].time),u=pos(plan[a]),v=pos(plan[b]);
  return u.map((x,i)=>x+(v[i]-x)*f) as Vec;
}
function distribution(values:number[]){
  if(!values.length)return null;
  values.sort((a,b)=>a-b);
  return {samples:values.length,rmsM:Math.sqrt(values.reduce((n,x)=>n+x*x,0)/values.length),
    p95M:values[Math.ceil(.95*values.length)-1],peakM:values.at(-1)!};
}
function stationaryEvents(plan:Sample[]){
  const events:Sample[][]=[];
  let event:Sample[]=[];
  for(const sample of plan){
    if(sample.speed<=1e-4)event.push(sample);
    else if(event.length){events.push(event);event=[];}
  }
  if(event.length)events.push(event);
  return events.filter(e=>e[0]!==plan[0]&&e.at(-1)!==plan.at(-1));
}

export async function compareMission(routeCsv:string,telemetry:string,receipt:Receipt|null,stops:Point[]|null,
    policy:Limits=limits){
  for(const [name,value] of Object.entries(policy))if(!Number.isFinite(value)||value<=0)throw Error('Invalid '+name);
  const [routeSha,logSha]=await Promise.all([sha256(routeCsv),sha256(telemetry)]);
  const runtime=receipt?.controller_runtime?.trajectory_sha256,archived=receipt?.source_sha256?.['trajectory.csv'];
  const logId=receipt?.telemetry_sha256;
  // Legacy manifests hash the default source-tree CSV, even if the runner selected another file.
  for(const digest of [runtime??archived,receipt?.selected_trajectory_sha256])if(digest!==undefined&&digest!==routeSha)
    throw Error('Route identity mismatch. The recording manifest names a different route CSV.');
  if(logId!==undefined&&logId!==logSha)throw Error('Telemetry identity mismatch. The manifest names a different recording.');
  const source=receipt?.source??'unspecified';
  const identity=runtime?'controller runtime':archived?'archived input':'unbound';
  const bound=Boolean(runtime&&logId&&receipt?.run_id);
  const plan=parseCsv(routeCsv);
  if(plan.length<2)throw Error('The route needs at least two samples.');
  const {rows,clock}=table(telemetry);
  const mission=rows.filter(r=>r.phase==='TRAJECTORY');
  const first=mission[0],last=mission.at(-1);
  const start=plan[0].time,end=plan.at(-1)!.time,duration=end-start;
  const contactIndex=rows.findIndex(r=>(r.phase==='PX4_LAND'||r.phase==='PX4_FAILSAFE')&&r.landed===1);
  const contact=rows[contactIndex];
  const prefix=contact?rows.slice(0,contactIndex+1):rows;
  const frameContinuous=new Set(prefix.map(r=>r.segment)).size===1;
  const phaseContinuous=Boolean(first&&last&&rows.slice(rows.indexOf(first),rows.indexOf(last)+1).length===mission.length);
  const clockOk=Boolean(first&&last&&first.clock!==null&&last.clock!==null&&
    Math.abs(first.clock-start)<=policy.clockEndpointS&&Math.abs(last.clock-end)<=policy.clockEndpointS&&
    mission.every((r,i)=>r.clock!==null&&(i===0||r.clock>=mission[i-1].clock!)));
  const maxGap=(series:typeof rows)=>series.reduce((n,r,i)=>i?Math.max(n,r.elapsed-series[i-1].elapsed):0,0);
  const gap=maxGap(mission),routeTruth=mission.filter(r=>r.truth);
  const coverage=mission.length>=2&&clockOk&&phaseContinuous&&frameContinuous&&
    gap<=policy.maxGapS&&routeTruth.length===mission.length;
  // Frame translation is estimated exclusively from the logged command. Measured truth never sets the origin.
  const base=first?.clock!=null?positionAt(plan,first.clock):null;
  const offset=base&&first?.desired?first.desired.map((v,i)=>v-base[i]) as Vec:null;
  const errors:number[]=[],commandErrors:number[]=[];
  for(const r of mission){
    if(r.clock===null||!offset)continue;
    const target=positionAt(plan,r.clock).map((v,i)=>v+offset[i]);
    if(r.truth)errors.push(dist(r.truth,target));
    if(r.desired)commandErrors.push(dist(r.desired,target));
  }
  const agreement=distribution(commandErrors),tracking=distribution(errors);
  const aligned=Boolean(coverage&&agreement&&agreement.samples===mission.length&&agreement.peakM<=policy.referenceAgreementM);
  const required=stops?.slice(1,-1).filter(p=>p.hard);
  const events=stationaryEvents(plan);
  let nextEvent=0,afterObserved=-Infinity;
  function firstAt(time:number){
    let lo=0,hi=mission.length;
    while(lo<hi){const mid=(lo+hi)>>1;if(mission[mid].clock!<time)lo=mid+1;else hi=mid;}
    return lo;
  }
  const visits=(required??[]).map(p=>{
    const target:Vec=[p.north,p.east,-p.altitude];
    // Each measured stop must correspond to a distinct, ordered stationary event.
    let best:Sample|undefined;
    for(let e=nextEvent;e<events.length;e++){
      let nearest:Sample|undefined,bestDistance=.02;
      for(const sample of events[e]){
        const d=dist(target,pos(sample));
        if(d<=bestDistance){nearest=sample;bestDistance=d;}
      }
      if(nearest){best=nearest;nextEvent=e+1;break;}
    }
    if(!best)return {point:p.id,verdict:'insufficient' as Verdict,plannedS:null,elapsedS:null,errorM:null,speedMs:null};
    let hit:{elapsedS:number;errorM:number;speedMs:number;score:number}|undefined;
    const translated=offset?target.map((v,i)=>v+offset[i]):null;
    if(aligned&&translated)for(let i=Math.max(1,firstAt(best.time-policy.stopWindowS));i<mission.length;i++){
      const r=mission[i],prev=mission[i-1],dt=r.elapsed-prev.elapsed;
      if(r.clock!>best.time+policy.stopWindowS)break;
      if(!r.truth||!prev.truth||r.clock===null||
        r.elapsed<=afterObserved||dt>policy.maxGapS||r.segment!==prev.segment)continue;
      const errorM=dist(r.truth,translated),speedMs=dist(r.truth,prev.truth)/dt;
      const score=Math.max(errorM/policy.stopRadiusM,speedMs/policy.stopSpeedMs);
      if(!hit||score<hit.score)hit={elapsedS:r.elapsed,errorM,speedMs,score};
    }
    if(hit)afterObserved=hit.elapsedS;
    const verdict:Verdict=!aligned||!hit?'insufficient':hit.score<=1?'pass':'fail';
    return {point:p.id,verdict,plannedS:best.time,elapsedS:hit?.elapsedS??null,errorM:hit?.errorM??null,speedMs:hit?.speedMs??null};
  });
  let target=contact?.target??null, targetSource=target?'logged native landing target':'';
  if(receipt?.landing_target_local_ne_m!==undefined){
    if(!hasPair(receipt.landing_target_local_ne_m))throw Error('Landing target must be finite local N/E metres.');
    target=receipt.landing_target_local_ne_m;targetSource='declared local N/E target';
  }
  const touchdownError=target&&contact?.truth?dist(target,contact.truth.slice(0,2)):null;
  const previous=rows[contactIndex-1];
  const arrival=Boolean(contact&&previous&&last&&contact.elapsed>last.elapsed&&previous.landed!==null&&
    [2,3,4].includes(previous.landed)&&contact.elapsed-previous.elapsed<=policy.maxGapS&&frameContinuous&&
    rows.slice(rows.indexOf(last),contactIndex+1).every((r,i,a)=>i===0||r.elapsed-a[i-1].elapsed<=policy.maxGapS));
  const wall=first&&last?last.elapsed-first.elapsed:null;
  const checks:Check[]=[
    {name:'Route coverage and frame',verdict:coverage?'pass':'insufficient',detail:routeTruth.length+'/'+mission.length+' route samples with truth; maximum recorded gap '+gap.toFixed(3)+' s'},
    {name:'Reference agreement',verdict:!coverage||!agreement||agreement.samples!==mission.length?'insufficient':aligned?'pass':'fail',detail:agreement?'Peak logged command difference '+agreement.peakM.toPrecision(3)+' m after one translation':'No aligned reference'},
    {name:'Tracking',verdict:!aligned||!tracking?'insufficient':tracking.rmsM<=policy.trackingRmsM?'pass':'fail',detail:tracking?'3D RMS '+tracking.rmsM.toFixed(3)+' m · p95 '+tracking.p95M.toFixed(3)+' m · peak '+tracking.peakM.toFixed(3)+' m':'Independent truth unavailable'},
    {name:'Route timing',verdict:!aligned||wall===null?'insufficient':Math.abs(wall-duration)<=policy.timingFraction*duration?'pass':'fail',detail:'Planned '+duration.toFixed(2)+' s · elapsed '+(wall?.toFixed(2)??'—')+' s'},
    {name:'Required interior stops',verdict:required===undefined?'insufficient':!visits.length?'not applicable':visits.some(s=>s.verdict==='fail')?'fail':visits.every(s=>s.verdict==='pass')?'pass':'insufficient',
      detail:required===undefined?'Stop declarations unavailable':visits.filter(s=>s.verdict==='pass').length+'/'+visits.length+' met position and speed together'},
    {name:'Landing band',verdict:!arrival||touchdownError===null?'insufficient':contact.phase==='PX4_FAILSAFE'||touchdownError>policy.landingRadiusM?'fail':'pass',
      detail:touchdownError===null?'Contact truth or landing target unavailable':touchdownError.toFixed(3)+' m horizontal error · '+policy.landingRadiusM+' m radius'}
  ];
  const evidence=source==='synthetic'?'synthetic fixture':source==='retained_sitl'?'retained simulator recording':source==='hardware'?'user-declared hardware recording':'user recording';
  const eligible=bound&&['synthetic','hardware','retained_sitl'].includes(source);
  const verdict=!eligible?'diagnostic only':checks.some(c=>c.verdict==='fail')?'fail':checks.some(c=>c.verdict==='insufficient')?'insufficient':'pass';
  return {schema:'trajectory-studio.mission-comparison.v1',evidence,verdict,runId:receipt?.run_id??null,identity,telemetryBound:Boolean(logId),
    routeSha256:routeSha,telemetrySha256:logSha,policy:{...policy},checks,visits,tracking,clock,translationNedM:offset,
    runPhaseTimeS:{reference:duration,elapsed:wall,missionClockStart:first?.clock??null,missionClockEnd:last?.clock??null},
    referencePeakDifferenceM:agreement?.peakM??null,coverage:{truthSamples:routeTruth.length,routeSamples:mission.length,maxGapS:gap,clockOk,frameContinuous,phaseContinuous},
    landing:{targetLocalNeM:target,targetSource,elapsedS:contact?.elapsed??null,horizontalErrorM:touchdownError,source:'First on-ground sample during native landing'},
    assumptions:['Local NED metres','Linear interpolation between exported route samples','One constant command-derived frame translation',
      'Stop speed uses successive truth positions in elapsed time','Study thresholds are research criteria']};
}
export async function syntheticRun(routeCsv:string){
  const plan=parseCsv(routeCsv),lines=['elapsed_s,phase,mission_time_s,desired_x,desired_y,desired_z,truth_x,truth_y,truth_z,landed_state'];
  for(const s of plan)lines.push([s.time,'TRAJECTORY',s.time,...pos(s),s.north+.1,s.east,-s.altitude,2].join(','));
  const end=plan.at(-1)!.time,home=plan[0];
  for(const [dt,landed] of [[.05,4],[.10,1]])lines.push([end+dt,'PX4_LAND','','','','',home.north+.1,home.east,0,landed].join(','));
  const telemetry=lines.join('\n')+'\n';
  const receipt:Receipt={source:'synthetic',run_id:'generated-route-fixture',controller_runtime:{trajectory_sha256:await sha256(routeCsv)},
    telemetry_sha256:await sha256(telemetry),landing_target_local_ne_m:[home.north,home.east]};
  return {telemetry,receipt};
}
