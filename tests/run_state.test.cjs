const test=require('node:test'),assert=require('node:assert/strict');
const {next,labels}=require('../web/run-state.js');
test('planner states distinguish adjusted references, cancellation and errors',()=>{
 for(const final of ['verified','adjusted','cancelled','error']){
  let s=next('draft','starting');s=next(s,'loading');s=next(s,'checking');s=next(s,final);
  assert.equal(s,final);assert.equal(next(s,'checking'),final);assert.ok(labels[s]);
  assert.equal(next(s,'draft'),'draft');
 }
});
test('editing cannot be overwritten by late progress; stale result needs a new check',()=>{
 let s=next('checking','stale');assert.equal(next(s,'checking'),'stale');
 assert.equal(next(s,'loading'),'stale');assert.equal(next(s,'draft'),'draft');
 assert.equal(next('draft','verified'),'draft');assert.throws(()=>next('draft','unknown'));
});
