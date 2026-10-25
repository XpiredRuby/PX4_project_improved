const test=require('node:test'),assert=require('node:assert/strict');
const {policy}=require('../web/speed-settings.js');
const ui=require('../web/simple-ui.js');const {RouteDraft}=require('../mission_planner/draft.js');
test('aircraft mode ignores a stale 1.25 draft cap while preserving declared model ceiling',()=>{
 const p=policy(2,1.25,3,'aircraft');assert.equal(p.cap,3);assert.equal(p.planning,1.8);assert.equal(p.conflict,false);
 const high=policy(5,99,3,'aircraft');assert.equal(high.planning,2.7);assert.match(high.message,/10% margin/);
});
test('intentional custom caps are binding and a higher cruise is a conflict',()=>{
 assert.equal(policy(2,1.25,3,'custom').conflict,true);assert.equal(policy(1,1.25,3,'custom').conflict,false);
 assert.equal(policy(2,5,3,'custom').planning,1.8);
 for(const value of [0,-1,NaN,Infinity])assert.throws(()=>policy(value,3,3));
 assert.throws(()=>policy(1,0,3,'custom'));assert.throws(()=>policy(1,3,3,'unknown'));
});
test('version 2 drafts persist explicit speed policy and do not restore certificates',()=>{
 const value={version:2,speed_cap_mode:'aircraft',mode:'drawing',tool:'points',route:{points:[[0,0],[2,0]],kinds:['ordinary','ordinary'],closure:'open_stop'},fields:Object.fromEntries(ui.fields.map(k=>[k,'1']))};
 assert.equal(ui.decode(JSON.stringify(value),new RouteDraft()).speed_cap_mode,'aircraft');
 value.speed_cap_mode='unbounded';assert.throws(()=>ui.decode(JSON.stringify(value),new RouteDraft()));
});
test('dense draft thinning retains every stop and exact point in original order',()=>{
 const d=new RouteDraft({bestEffort:true,maxPoints:7});const points=Array.from({length:20},(_,i)=>[i,0]),kinds=points.map((_,i)=>i===3?'stop':i===13?'exact':'ordinary');
 d.commit(points,kinds);assert.ok(d.points.length<=7);assert.deepEqual(d.points.filter((_,i)=>d.kinds[i]!=='ordinary'),[[3,0],[13,0]]);assert.deepEqual(d.points.at(-1),[19,0]);
});
test('marked-point overflow leaves the previous drawing and undo history intact',()=>{
 const d=new RouteDraft({bestEffort:true,maxPoints:3}),before=d.snapshot();
 assert.throws(()=>d.commit([[0,0],[1,0],[2,0],[3,0]],['ordinary','stop','exact','ordinary']),/marked points/);assert.deepEqual(d.snapshot(),before);assert.equal(d.past.length,0);
});
test('browser initialization and legacy restoration resolve the actual DOM policy',()=>{
 const vm=require('node:vm'),fs=require('node:fs'),elements={},callbacks={};
 for(const id of ['targetSpeed','maxSpeed','speedCapMode','speedInputHint','legacySpeedNotice','useAircraftSpeed'])elements[id]={value:id==='targetSpeed'?'2':id==='maxSpeed'?'1.25':'aircraft',classList:{toggle(){},add(){}},addEventListener(){},hidden:false};
 const window={PX4Editor:{aircraftSpeedLimit:()=>3}},document={getElementById:id=>elements[id],addEventListener:(event,fn)=>callbacks[event]=fn,querySelector:()=>null};
 vm.runInNewContext(fs.readFileSync(require('node:path').join(__dirname,'../web/speed-settings.js'),'utf8'),{window,document,Number,Error,Event});
 callbacks.DOMContentLoaded();assert.equal(elements.maxSpeed.value,'3');assert.match(elements.speedInputHint.textContent,/planning cap 1.80/);assert.equal(elements.maxSpeed.readOnly,true);
 window.PX4Editor.aircraftSpeedLimit=()=>{throw Error('Complete Aircraft setup');};assert.doesNotThrow(()=>window.PX4SpeedSettings.refresh());assert.match(elements.speedInputHint.textContent,/Complete Aircraft setup/);assert.equal(elements.maxSpeed.value,'');
 window.PX4Editor.aircraftSpeedLimit=()=>2.5;window.PX4SpeedSettings.refresh();assert.equal(elements.maxSpeed.value,'2.5');
 window.PX4Editor.aircraftSpeedLimit=()=>3;elements.maxSpeed.value='1.25';window.PX4SpeedSettings.restore({version:1});assert.equal(elements.speedCapMode.value,'custom');assert.equal(elements.legacySpeedNotice.hidden,false);assert.throws(()=>window.PX4SpeedSettings.validate(),/custom Never exceed/);
});
test('malformed dense commits fail before thinning without hanging or changing history',()=>{
 const d=new RouteDraft({bestEffort:true,maxPoints:3}),before=d.snapshot();
 assert.throws(()=>d.commit([[0,0],[1,0],[2,0],[3,0]],['ordinary']));assert.deepEqual(d.snapshot(),before);assert.equal(d.past.length,0);
});
