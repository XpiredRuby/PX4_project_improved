import test from 'node:test';
import assert from 'node:assert/strict';
import { airspeed, buildRequest, initialEnvironment, interpolate, metrics, parseCsv, sampleProject, validateProject, windNE } from '../src/engineering.ts';

const rows='time,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw\n0,0,0,-12,0,0,0,0,0,0,0,0,0,0\n0.05,1,0,-12,2,0,0,1,0,0,0.1,0,0,0.2\n0.1,2,0,-12,2,0,0,0,0,0,0,0,0,0.4\n';
test('first timestamp, NED height and endpoint are preserved',()=>{
 const s=parseCsv(rows);
 assert.equal(s[0].time,0);assert.equal(s[0].altitude,12);
 assert.equal(s[2].north,2);assert.equal(s[2].time,.1);
 assert.equal(interpolate(s,0).north,0);assert.equal(interpolate(s,.1).north,2);
 assert.equal(interpolate(s,.075).north,1.5);
});
test('non-monotonic or nonfinite reference is rejected',()=>{
 assert.throws(()=>parseCsv(rows.replace('0.05,1','0,1')),/strictly increasing/);
 assert.throws(()=>parseCsv(rows.replace('0.05,1','0.05,NaN')),/Non-finite/);
});
test('meteorological FROM direction is converted into actual wind velocity',()=>{
 const north=windNE(5,0);
 assert.ok(Math.abs(north.north+5)<1e-10);assert.ok(Math.abs(north.east)<1e-10);
 const s=parseCsv(rows)[1];assert.equal(airspeed(s,{...initialEnvironment,windSpeed:5,windDirection:0}),7);
 assert.equal(airspeed(s,{...initialEnvironment,windSpeed:5,windDirection:180}),3);
});
test('first sample is included in metric computations',()=>{
 const m=metrics(parseCsv(rows),initialEnvironment);
 assert.equal(m.duration,.1);assert.equal(m.distance,2);
 assert.equal(m.peak,2);assert.equal(m.accel,1);
});
test('research speed cap is retained in request; model limit is not silently raised',()=>{
 const p={...sampleProject,cruise:10,speedCap:20};
 validateProject(p);
 const req=buildRequest(p);
 assert.equal(req.document.motion.maximum_speed_m_s,20);
 assert.equal(req.document.motion.target_speed_m_s,10);
 assert.equal(req.document.vehicle_model_id,'px4-x500-sitl-v1');
 assert.equal(req.document.path.points[0].down_m,-12);
});
test('invalid geometry and conflicting cruise cap are rejected',()=>{
 assert.throws(()=>validateProject({...sampleProject,cruise:5,speedCap:3}),/Cruise cannot/);
 assert.throws(()=>validateProject({...sampleProject,points:[{...sampleProject.points[0],north:999},sampleProject.points[1]]}),/North coordinate/);
});
