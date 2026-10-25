const test=require('node:test'),assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm');
function setup(){
 const requests=[],workers=[],timers=new Map();let clock=0;
 class FakeWorker{constructor(){workers.push(this);}postMessage(value){this.request=value;}terminate(){this.stopped=true;}}
 const window={fetch:async(...args)=>{requests.push(args);return new Response('{}');},addEventListener(){},dispatchEvent(){}};
 vm.runInNewContext(fs.readFileSync(require('node:path').join(__dirname,'../web/browser-api.js'),'utf8'),{window,location:{href:'https://studio.example/',origin:'https://studio.example'},URL,Response,TextEncoder,Worker:FakeWorker,CustomEvent:class{},document:{getElementById(){return null;}},setTimeout(fn){const key=++clock;timers.set(key,fn);return key;},clearTimeout(key){timers.delete(key);}});
 return {window,requests,workers,timers};
}
test('public capabilities do not enable flight or fabricate simulator status',async()=>{
 const s=setup(),caps=await(await s.window.fetch('/api/capabilities')).json();
 assert.equal(caps.sitl_enabled,false);assert.equal(caps.hardware_enabled,false);
 assert.equal((await s.window.fetch('/api/rehearsal',{method:'POST',body:'{}'})).status,404);assert.equal(s.requests.length,0);assert.equal(s.workers.length,0);
});
test('one worker returns its result, with concurrent and stale responses rejected',async()=>{
 const s=setup(),first=s.window.fetch('/api/preview',{body:'{"source":"drawing","document":{}}'});
 assert.equal((await s.window.fetch('/api/preview',{body:'{}'})).status,503);assert.equal(s.workers.length,1);
 const worker=s.workers[0];assert.equal(worker.request.operation,'preview');
 worker.onmessage({data:{id:worker.request.id+1,result:{accepted:false}}});assert.equal(s.timers.size,1);
 worker.onmessage({data:{id:worker.request.id,result:{accepted:true,simulation_status:'NOT_RUN'}}});
 assert.equal((await(await first).json()).accepted,true);assert.equal(s.timers.size,0);assert.equal(s.requests.length,0);
});
test('timeout destroys computation and permits a fresh retry',async()=>{
 const s=setup(),first=s.window.fetch('/api/preview',{body:'{}'});[...s.timers.values()][0]();
 assert.equal((await first).status,503);assert.equal(s.workers[0].stopped,true);
 const retry=s.window.fetch('/api/preview',{body:'{}'});assert.equal(s.workers.length,2);
 const worker=s.workers[1];worker.onmessage({data:{id:worker.request.id,result:{accepted:false,error:'Route refused'}}});assert.equal((await retry).status,400);
});
test('unknown and oversized requests cannot reach a worker or server',async()=>{
 const s=setup();assert.equal((await s.window.fetch('/api/arm')).status,404);
 assert.equal((await s.window.fetch('/api/preview',{body:' '.repeat(2*1024*1024+1)})).status,400);assert.equal(s.workers.length,0);assert.equal(s.requests.length,0);
});
test('dense handwriting payload reaches the worker instead of the old 64 KiB limit',async()=>{
 const s=setup(),body=JSON.stringify({source:'drawing',planning_mode:'closest',document:{samples:'x'.repeat(100000)}});
 const result=s.window.fetch('/api/preview',{body});assert.equal(s.workers.length,1);
 const worker=s.workers[0];worker.onmessage({data:{id:worker.request.id,result:{accepted:true}}});
 assert.equal((await result).status,200);assert.equal(s.requests.length,0);
});
test('cancellation terminates only the active worker and stale errors cannot kill a retry',async()=>{
 const s=setup(),controller=new AbortController();
 const first=s.window.fetch('/api/preview',{body:'{}',signal:controller.signal});
 const old=s.workers[0];controller.abort();
 assert.equal((await first).status,503);assert.equal(old.stopped,true);assert.equal(s.timers.size,0);
 const retry=s.window.fetch('/api/preview',{body:'{}'}),current=s.workers[1];
 old.onmessage({data:{id:old.request.id,result:{accepted:true}}});old.onerror();
 assert.equal(current.stopped,undefined);assert.equal(s.timers.size,1);
 current.onmessage({data:{id:current.request.id,result:{accepted:true}}});
 assert.equal((await retry).status,200);
});
test('already aborted request does not allocate a worker',async()=>{
 const s=setup(),controller=new AbortController();controller.abort();
 assert.equal((await s.window.fetch('/api/preview',{body:'{}',signal:controller.signal})).status,503);
 assert.equal(s.workers.length,0);
});
