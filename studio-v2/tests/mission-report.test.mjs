import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {compareMission,limits,openSavedCase,sha256,syntheticRun} from '../src/missionReport.ts';

const read=p=>readFileSync(new URL(p,import.meta.url));
const archive='../../validation/2026-10-03/cases/raised-10m-final/';
const route=read('../../evidence/archived-x500-flight/trajectory.csv').toString();
const telemetry=gunzipSync(read(archive+'telemetry_and_truth.csv.gz')).toString();
const receipt={...JSON.parse(read(archive+'run_manifest_20261003T062757218606Z.json')),source:'retained_sitl',
  telemetry_sha256:'129b0b24f49291b1668d546da65e5be0397e85e8891172fea7e279b5fe4bcd6d',landing_target_local_ne_m:[.0375346475,.0921971667]};
const check=(v,name)=>v.checks.find(c=>c.name===name).verdict;
test('archived source identity, reference alignment, independent touchdown and both clocks',async()=>{
  const r=await compareMission(route,telemetry,receipt,null);
  assert.equal(r.verdict,'diagnostic only');assert.equal(r.identity,'archived input');
  assert.equal(r.coverage.truthSamples,2539);
  assert.ok(Math.abs(r.tracking.rmsM-.1600507)<1e-6);
  const audited=JSON.parse(read(archive+'independent_verification.json')).physical_touchdown_xy_error_m;
  assert.ok(Math.abs(r.landing.horizontalErrorM-audited)<1e-7);
  assert.ok(Math.abs(r.runPhaseTimeS.elapsed-126.9001029)<1e-5);
  assert.equal(check(r,'Route timing'),'fail');assert.equal(check(r,'Required interior stops'),'insufficient');
  assert.equal(check(r,'Landing band'),'pass');
});
test('changing route line endings or telemetry bytes invalidates a supplied digest',async()=>{
  await assert.rejects(compareMission(route.replaceAll('\r\n','\n'),telemetry,receipt,null),/Route identity mismatch/);
  await assert.rejects(compareMission(route,telemetry+'\n',receipt,null),/Telemetry identity mismatch/);
  await assert.rejects(compareMission(route,telemetry,{...receipt,controller_runtime:{trajectory_sha256:'00'}},null),/Route identity mismatch/);
});
// Deliberately long enough to exercise wall/mission time, stop sampling and gap detection.
const plan='time,x,y,z,vx,vy,vz\n'+Array.from({length:81},(_,i)=>{
  const x=i<=40?i/40:(80-i)/40;
  return [i/20,x,0,-10,[0,40,80].includes(i)?0:i<40?.5:-.5,0,0].join(',');
}).join('\n')+'\n';
const points=[{id:'A',north:0},{id:'B',north:1},{id:'C',north:0}].map(p=>({...p,east:0,altitude:10,hard:true}));
const fixture=await syntheticRun(plan);
async function edit(fn,opts={}){
  const data=fixture.telemetry.trimEnd().split('\n').map(s=>s.split(','));
  const csv=fn(data).map(r=>r.join(',')).join('\n')+'\n';
  return compareMission(plan,csv,{...fixture.receipt,telemetry_sha256:await sha256(csv)},points,{...limits,...opts});
}
test('required stops use observed velocity and position together',async()=>{
  const r=await compareMission(plan,fixture.telemetry,fixture.receipt,points);
  assert.equal(r.evidence,'synthetic fixture');assert.equal(check(r,'Required interior stops'),'fail');
  assert.equal(r.visits[0].point,'B');assert.equal(check(r,'Tracking'),'pass');
  const stationary=await edit(d=>{d[41][6]=d[40][6];return d;});
  assert.equal(stationary.verdict,'pass');
  assert.ok(stationary.visits[0].speedMs<1e-6);
});
test('missing truth, coordinate reset, duplicate elapsed time and wrong clock cannot pass',async()=>{
  assert.equal(check(await edit(d=>{d[11][6]='';return d;}),'Route coverage and frame'),'insufficient');
  assert.equal(check(await edit(d=>d.map((row,i)=>[...row,i===0?'segment':i<32?'0':'1'])),'Tracking'),'insufficient');
  assert.equal(check(await edit(d=>{d.splice(15,9);return d;}),'Tracking'),'insufficient');
  assert.equal(check(await edit(d=>{d[20][2]='0';return d;}),'Tracking'),'insufficient');
  await assert.rejects(edit(d=>{d[15][0]=d[14][0];return d;}),/Elapsed time must increase/);
  const unbound=await compareMission(plan,fixture.telemetry,null,points);
  assert.equal(unbound.verdict,'diagnostic only');assert.equal(unbound.identity,'unbound');
  const est=await compareMission(plan,fixture.telemetry.replaceAll('truth_','estimate_'),null,points);
  assert.equal(est.tracking,null);assert.equal(check(est,'Landing band'),'insufficient');
});
test('landing target, contact transition and logged commands are independent checks',async()=>{
  assert.equal(check(await edit(d=>{d.at(-1)[6]='8';return d;}),'Landing band'),'fail');
  assert.equal(check(await edit(d=>d.slice(0,-1)),'Landing band'),'insufficient');
  assert.equal(check(await edit(d=>{d.at(-2)[9]='';return d;}),'Landing band'),'insufficient');
  assert.equal(check(await edit(d=>{d.at(-1)[1]='PX4_FAILSAFE';return d;}),'Landing band'),'fail');
  const wrong=await edit(d=>{d[24][3]='30';return d;});
  assert.equal(check(wrong,'Reference agreement'),'fail');assert.equal(check(wrong,'Tracking'),'insufficient');
  assert.equal(check(await edit(d=>{d[12][1]='TAKEOFF';return d;}),'Tracking'),'insufficient');
});

test('the same dwell cannot confirm two ordered stops and the selected runtime CSV takes precedence over source defaults',async()=>{
  const r=await compareMission(plan,fixture.telemetry,{...fixture.receipt,selected_trajectory_sha256:await sha256(plan),
    source_sha256:{'trajectory.csv':'0'.repeat(64)}},[points[0],points[1],points[1],points[2]]);
  assert.equal(r.visits[1].plannedS,null);
  await assert.rejects(compareMission(plan,fixture.telemetry,{...fixture.receipt,selected_trajectory_sha256:'c'.repeat(64)},points),/Route identity mismatch/);
});

test('a saved case uses its own exact route and declared stops without recompilation',async()=>{
  const digest=await sha256(plan);
  const payload={schema:'trajectory-studio.case.v1',project:{title:'frozen 50 m/s'},
    request:{document:{path:{points:points.map(p=>({point_id:p.id,hard:p.hard,north_m:p.north,east_m:p.east,down_m:-p.altitude}))}}},
    metrics:{route_sha256:digest},result:{accepted:true,trajectory_csv:plan,summary:{controller_trajectory_sha256:digest}}};
  const opened=await openSavedCase(JSON.stringify(payload));
  assert.equal(opened.digest,digest);assert.equal(opened.route,plan);assert.equal(opened.points[1].hard,true);
  payload.result.trajectory_csv+='\n';
  await assert.rejects(openSavedCase(JSON.stringify(payload)),/identity mismatch/);
});

test('dense elapsed samples cannot conceal a gap in route-clock coverage',async()=>{
 const result=await edit(d=>{
   for(let i=1;i<d.length-2;i++)d[i][2]=String(i<=40?(i-1)/20:2.45+(i-41)*(1.55/40));
   return d;
 });
 assert.ok(result.coverage.maxGapS<.1);
 assert.ok(Math.abs(result.coverage.maxRouteClockGapS-.5)<1e-12);
 assert.equal(check(result,'Route coverage and frame'),'insufficient');
});

test('an explicitly selected route takes precedence over an archived default even without a runtime receipt',async()=>{
 const result=await compareMission(plan,fixture.telemetry,{...fixture.receipt,controller_runtime:undefined,
   selected_trajectory_sha256:await sha256(plan),source_sha256:{'trajectory.csv':'0'.repeat(64)}},points);
 assert.equal(result.identity,'selected input');
 assert.equal(result.verdict,'diagnostic only');
});
