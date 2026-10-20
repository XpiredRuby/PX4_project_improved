const assert=require('node:assert/strict');
const test=require('node:test');
const {RouteDraft}=require('../mission_planner/draft.js');
const ui=require('../web/simple-ui.js');
const valid=()=>({version:1,mode:'drawing',tool:'points',route:{points:[[0,0],[0,12],[8,12]],kinds:['ordinary','stop','ordinary'],closure:'open_stop'},fields:Object.fromEntries(ui.fields.map(k=>[k,k==='yaw'?'tangent':k==='pattern'?'circle':'1']))});
test('draft recovery preserves route order, marked stops and settings',()=>{
 const original=valid(),checked=ui.decode(JSON.stringify(original),new RouteDraft());
 assert.deepEqual(checked,original);
 const draft=new RouteDraft();draft.restore(checked.route);
 assert.deepEqual(draft.points,original.route.points);assert.equal(draft.kinds[1],'stop');
});
test('recovery applies the current backend radius and point limit',()=>{
 const value=valid();
 assert.throws(()=>ui.decode(JSON.stringify(value),new RouteDraft({radius:5})));
 assert.throws(()=>ui.decode(JSON.stringify(value),new RouteDraft({maxPoints:2})));
});
test('corrupt and incompatible drafts cannot overwrite the route',()=>{
 for(const change of [
  v=>{v.version=99},v=>{v.mode='hardware'},v=>{v.tool='code'},
  v=>{v.route.points[0]=[1,0]},v=>{v.route.points[1]=['12',0]},
  v=>{v.route.kinds.pop()},v=>{v.route.kinds[1]='unknown'},
  v=>{v.route.closure='automatic'},v=>{v.fields.altitude={value:1}},
  v=>{v.fields.waypoints='x'.repeat(10001)},
  v=>{delete v.fields.yaw},v=>{v.route.closure='periodic'}
 ]){
  const value=valid(),draft=new RouteDraft(),before=draft.snapshot();change(value);
  assert.throws(()=>ui.decode(JSON.stringify(value),draft));assert.deepEqual(draft.snapshot(),before);
 }
 assert.throws(()=>ui.decode('{',new RouteDraft()));
 assert.throws(()=>ui.decode(' '.repeat(32769),new RouteDraft()));
});
test('saved certificates and simulator state are never restored',()=>{
 const value=valid();value.accepted=true;value.plan_id='old';value.route.past=['unsafe'];
 const checked=ui.decode(JSON.stringify(value),new RouteDraft());
 assert.equal(checked.accepted,undefined);assert.equal(checked.plan_id,undefined);assert.equal(checked.route.past,undefined);
});
test('duration labels and error recovery use everyday language',()=>{
 assert.equal(ui.duration(333.7),'5 min 34 sec');assert.equal(ui.duration(120),'2 min');
 assert.equal(ui.duration(4.25),'4.3 sec');assert.equal(ui.duration(NaN),'—');
 assert.match(ui.errorMessage({limiting_constraint:'duration',minimum_feasible_duration_s:125}),/2 min 5 sec/);
 assert.match(ui.errorMessage({error:'solver infeasible NaN'}),/Review Points/);
 assert.match(ui.errorMessage({error:'exceeded 120 seconds'}),/reduce the number of points/);
});
