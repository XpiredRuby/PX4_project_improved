const test=require('node:test'),assert=require('node:assert/strict');
const compare=require('../web/comparison.js');
function fixture(samples,points=[[0,0],[0,2],[2,2]],margin=.2){return {intent:{path:{points:points.map(p=>({east_m:p[0],north_m:p[1]})),maximum_deviation_m:.5}},samples:samples.map(p=>({y:p[0],x:p[1]})),verification:{accepted:true,checks:[{requirement_id:'INT-007',status:'PASS',units:'m',margin}]},display_samples_decimated:true};}
test('gap is the nearest segment distance, not distance to a drawn vertex',()=>{
 const r=compare.analyze(fixture([[0,0],[0,1],[.2,1.8],[2,2]]));assert.ok(Math.abs(r.worst.distance-.2)<1e-10);assert.ok(Math.abs(r.bound-.3)<1e-10);assert.equal(r.decimated,true);
});
test('overlapping paths have zero displayed change and repeated vertices stay finite',()=>{
 const r=compare.analyze(fixture([[0,0],[0,1],[0,2]],[[0,0],[0,0],[0,2]]));assert.equal(r.worst.distance,0);assert.ok(Number.isFinite(compare.nearest([0,1],[[0,0],[0,0],[0,2]]).distance));
});
test('verified bound is unavailable when certificate is absent, failed or invalid',()=>{
 for(const margin of [null,NaN,-1,1]){assert.equal(compare.analyze(fixture([[0,0],[1,1]],undefined,margin)).bound,null);}
 const r=fixture([[0,0],[1,1]]);r.verification.accepted=false;assert.equal(compare.analyze(r).bound,null);delete r.verification;assert.equal(compare.analyze(r).bound,null);
});
test('layer rendering isolates state and uses visible widths at phone scale',()=>{
 const calls=[];const ctx=new Proxy({},{get:(t,k)=>{if(k in t)return t[k];return (...args)=>calls.push([k,...args]);},set:(t,k,v)=>{calls.push([k,v]);t[k]=v;return true;}});
 compare.view='overlay';compare.drawOriginal(ctx,[[0,0],[1,1]],p=>p,2);assert.ok(calls.some(c=>c[0]==='lineWidth'&&c[1]===24));assert.equal(calls[0][0],'save');assert.equal(calls.at(-1)[0],'restore');
 calls.length=0;compare.view='plan';compare.drawOriginal(ctx,[[0,0],[1,1]],p=>p,2);assert.equal(calls.length,0);
 compare.view='input';compare.drawPlanned(ctx,[[0,0],[1,1]],p=>p,2);assert.equal(calls.length,0);compare.view='overlay';
});
