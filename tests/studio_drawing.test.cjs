const {test} = require('node:test');
const assert = require('node:assert/strict');
const {RouteDraft} = require('../mission_planner/draft.js');

test('fresh route starts only at launch; explicit points preserve short details', () => {
  const d = new RouteDraft(); assert.deepEqual(d.points, [[0,0]]);
  d.add([0,1]); d.add([.03,1.02]);
  assert.deepEqual(d.points.at(-1), [.03,1.02]);
});
test('release endpoint is saved even without a movement event', () => {
  const d = new RouteDraft(); d.begin([0,0],1,.5); d.move([0,1],1);
  d.finish([.03,1.03],1); assert.deepEqual(d.points.at(-1),[.03,1.03]);
  assert.equal(d.past.length,1);
});
test('disconnected stroke requires explicit connection; discard leaves old route', () => {
  const d = new RouteDraft();d.begin([4,4],1,.5);d.finish([5,5],1);
  assert.deepEqual(d.points,[[0,0]]);assert.ok(d.pending);
  assert.throws(()=>d.add([1,1]),/Connect or discard/);
  d.connect();assert.deepEqual(d.points,[[0,0],[4,4],[5,5]]);
  d.begin([10,10],1,.5);d.finish([11,11],1);d.discard();
  assert.deepEqual(d.points.at(-1),[5,5]);
});
test('cancelled gesture and unrelated pointer cannot change route', () => {
  const d = new RouteDraft();d.begin([0,0],1,.5);d.move([0,4],2);
  assert.equal(d.finish([2,2],2),false);d.cancel(1);
  assert.equal(d.gesture,null);assert.deepEqual(d.points,[[0,0]]);
});
test('a boundary excursion rejects the entire gesture even after reentry', () => {
  const d = new RouteDraft({radius:10});d.add([0,1]);
  d.begin([0,1],1,.5);d.move([0,11],1);d.move([0,2],1);
  assert.throws(()=>d.finish([0,2],1),/within 10 m/);
  assert.deepEqual(d.points,[[0,0],[0,1]]);
});
test('point cap includes generated loop closure, rejects without clipping', () => {
  const d = new RouteDraft({maxPoints:4});d.add([0,1]);d.add([1,1]);
  d.setClosure('periodic');assert.throws(()=>d.add([2,1]),/too many points/);
  assert.equal(d.points.length,3);
});
test('undo and redo restore entire stroke, annotation and ending', () => {
  const d = new RouteDraft();d.begin([0,0],1,.5);d.move([0,1],1);d.finish([1,1],1);
  d.setKind(1,'stop');d.undo();assert.equal(d.kinds[1],'ordinary');
  d.redo();assert.equal(d.kinds[1],'stop');d.undo();d.undo();
  assert.deepEqual(d.points,[[0,0]]);d.redo();assert.equal(d.points.length,3);
  d.setClosure('periodic');d.undo();assert.equal(d.closure,'open_stop');
});
test('new edits clear redo; clear is undoable', () => {
  const d = new RouteDraft();d.add([0,1]);d.add([1,1]);d.undo();d.add([2,2]);
  assert.equal(d.redo(),false);d.clear();assert.equal(d.points.length,1);
  d.undo();assert.deepEqual(d.points.at(-1),[2,2]);
});
test('unsupported loop annotations and conflicting types are refused', () => {
  const d = new RouteDraft();d.add([0,2]);d.add([2,2]);d.setKind(1,'stop');
  assert.throws(()=>d.setClosure('periodic'),/Loops cannot/);
  assert.throws(()=>d.setKind(0,'stop'),/interior/);
  assert.throws(()=>d.setKind(1,'both'),/supported point/);
  d.setKind(1,'exact');assert.equal(d.kinds[1],'exact');
});
test('repeated visits and point order are preserved', () => {
  const d = new RouteDraft();for(const p of [[1,1],[2,0],[1,1],[0,2]])d.add(p);
  assert.deepEqual(d.points,[[0,0],[1,1],[2,0],[1,1],[0,2]]);
});
test('duplicate clicks and invalid coordinates do not mutate history', () => {
  const d = new RouteDraft();assert.equal(d.add([0,0]),false);
  assert.throws(()=>d.add([NaN,1]),/valid numbers/);assert.equal(d.past.length,0);
});

test('best-effort handwriting connects separate letters and retains a dense stroke', () => {
  const d = new RouteDraft({bestEffort:true,maxPoints:8192,radius:Infinity});
  for (const [id,shape] of [
    [1,[[0,8],[2,0],[4,8]]],
    [2,[[6,8],[6,1]]],
    [3,[[8,1],[8,5],[10,5],[10,1]]]
  ]) {
    d.begin(shape[0],id,.1);
    for (const p of shape.slice(1)) d.move(p,id);
    d.finish(shape.at(-1),id);
    assert.equal(d.pending,null);
  }
  assert.ok(d.points.length>8);assert.deepEqual(d.points.at(-1),[10,1]);
  const before=d.points.length;
  d.begin([10,1],4,.1);
  for(let i=1;i<=1200;i++)d.move([10+i/100,1+Math.sin(i/30)],4);
  d.finish([23,2],4);
  assert.ok(d.points.length>before+1000);assert.equal(d.pending,null);
  d.undo();assert.equal(d.points.length,before);d.redo();assert.deepEqual(d.points.at(-1),[23,2]);
});
test('best-effort capture stays bounded without throwing away a long gesture', () => {
  const d = new RouteDraft({bestEffort:true,maxPoints:64,radius:Infinity});
  d.begin([0,0],1,.1);
  for(let i=1;i<=10000;i++)d.move([i/100,Math.sin(i/20)],1);
  d.finish([100,0],1);
  assert.ok(d.points.length<=64);assert.deepEqual(d.points.at(-1),[100,0]);
  assert.match(d.samplingNotice,/thinned in order/);
});
