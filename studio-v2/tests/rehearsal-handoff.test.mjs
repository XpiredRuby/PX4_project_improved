import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import {checkHandoff,handoffFixture} from '../src/rehearsalHandoff.ts';
import {sha256} from '../src/missionReport.ts';
const routeCsv='time,x,y,z,vx,vy,vz\n0,0,0,-1,0,0,0\n';
const digest=await sha256(routeCsv),binding={plan_id:'a'.repeat(32),controller_trajectory_sha256:digest},input={routeCsv,binding};
test('offline fixture goes through submission and poll with exact receipt',async()=>{
  const r=await handoffFixture(routeCsv,digest);
  assert.equal(r.state,'assessment matched');assert.equal(r.simulationRan,false);assert.equal(r.trace.length,2);
});
test('edited input never submits, stale job or stale plan response cannot pass',async()=>{
  let calls=0;
  await assert.rejects(checkHandoff({...input,routeCsv:routeCsv+'\n'},async()=>{calls++;}),/route bytes/);
  assert.equal(calls,0);
  await assert.rejects(checkHandoff(input,async path=>path?{...binding,plan_id:'b'.repeat(32),stage:'COMPLETED'}:{...binding,stage:'RUNNING'},{pollMs:0}),/different plan/);
});
test('replays the retained failed worker status and rejects completion with missing or mismatched evidence',async()=>{
  const past=JSON.parse(readFileSync(new URL('../../validation/2026-10-25-conference/sitl-final-attempt/status.json',import.meta.url)));
  const replay=async(path)=>({...binding,...(path?past:{stage:'RUNNING'})});
  assert.equal((await checkHandoff(input,replay,{pollMs:0})).state,'failed');
  for(const detail of [{sha256:digest},{sha256:digest,selected_sha256:digest,controller_sha256:'b'.repeat(64)}]){
    const reply={...binding,stage:'COMPLETED',assessment:{sampled_rehearsal_passed:true,checks:[{
      name:'Executed CSV digest matches retained plan and runner manifest',passed:true,detail}]}};
    assert.equal((await checkHandoff(input,async()=>reply)).state,'execution unverified');
  }
});
test('a timeout during submission requests cancellation once and keeps an unconfirmed outcome open',async()=>{
  const calls=[];
  const send=async(path,payload,signal)=>{
    calls.push({path,payload});
    if(path==='/cancel')return {...binding,stage:'RUNNING',cancel_requested:true};
    await new Promise((_,reject)=>signal.addEventListener('abort',()=>reject(Error('deadline')),{once:true}));
  };
  const r=await checkHandoff(input,send,{timeoutMs:40});
  assert.equal(r.state,'outcome unknown');assert.equal(r.reason,'deadline exceeded');
  assert.deepEqual(calls.map(c=>c.path),['','/cancel']);assert.deepEqual(calls[1].payload,binding);
});
test('user cancellation before submission skips it; subsequent confirmed cancellation is distinguished',async()=>{
  const stopped=new AbortController();stopped.abort();
  assert.equal((await checkHandoff(input,async()=>assert.fail(),{signal:stopped.signal})).state,'submission skipped');
  const stop=new AbortController();
  const send=async path=>{
    if(!path){stop.abort();return {...binding,stage:'STARTING'};}
    assert.equal(path,'/cancel');return {...binding,stage:'CANCELLED',cancel_confirmed:true};
  };
  assert.equal((await checkHandoff(input,send,{signal:stop.signal})).state,'cancelled');
});
