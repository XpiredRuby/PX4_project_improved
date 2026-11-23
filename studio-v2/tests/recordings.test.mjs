import test from 'node:test';
import assert from 'node:assert/strict';
import {readFlight,tracking} from '../src/recordings.ts';

test('3D errors use paired samples and preserve missing evidence',()=>{
  const f=readFlight('elapsed_s,truth_x,truth_y,truth_z,desired_x,desired_y,desired_z\n2,0,0,0,3,4,0\n3,,0,0,0,0,0\n5,1,2,3,,,');
  assert.deepEqual(tracking(f),{rows:2,paired:1,omitted:1,maxGap:3,rms:5,p95:5,max:5});
  assert.equal(f.frames[1].time,3);
});
test('separate local frames never contribute to the same error or plot',()=>{
  const text='elapsed_s,estimate_x,estimate_y,estimate_z,segment\n0,0,0,0,2\n.1,0,0,0,2\n.2,200,0,0,3\n.3,200,0,0,3';
  assert.deepEqual(readFlight(text).segments,['2','3']);
  const f=readFlight(text,'3');
  assert.equal(f.frames[0].measured[0],200);
  assert.equal(f.frames.length,2);
  assert.equal(tracking(f).rms,null);
});
test('timestamps must increase and hardware XYZ retains source frame',()=>{
  const data='t,px,py,pz,sp_ctrltarget_x,sp_ctrltarget_y,sp_ctrltarget_z\n1,2,3,4,2,3,4\n';
  assert.equal(readFlight(data+'2,2,3,4,2,3,4').kind,'Hardware motion capture');
  assert.throws(()=>readFlight(data+'1,2,3,4,2,3,4'),/timestamp/);
});
