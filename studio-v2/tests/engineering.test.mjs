import test from 'node:test';
import assert from 'node:assert/strict';
import { airspeed, dryAirDensity, buildRequest, initialEnvironment, interpolate, metrics, parseCsv, sampleProject, septicAt, validateProject, windNE } from '../src/engineering.ts';

const rows='time,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,yaw\n0,0,0,-12,0,0,0,0,0,0,0,0,0,0\n0.05,1,0,-12,2,0,0,1,0,0,0.1,0,0,0.2\n0.1,2,0,-12,2,0,0,0,0,0,0,0,0,0.4\n';
test('first timestamp, NED height and endpoint are preserved',()=>{
 const s=parseCsv(rows);
 assert.equal(s[0].time,0);assert.equal(s[0].altitude,12);
 assert.equal(s[2].north,2);assert.equal(s[2].time,.1);
 assert.equal(interpolate(s,0).north,0);assert.equal(interpolate(s,.1).north,2);
 assert.ok(Number.isFinite(interpolate(s,.075).north));
});
test('C3 septic reference interpolation exactly reconstructs rest-to-rest 7th order motion',()=>{
 const start=parseCsv(rows)[0];
 const finish={...start,time:1,north:1,yaw:Math.PI};
 const middle=interpolate([start,finish],.5);
 assert.ok(Math.abs(middle.north-.5)<1e-12);
 assert.ok(Math.abs(middle.yaw-Math.PI/2)<1e-11);
 assert.ok(Math.abs(middle.vx-2.1875)<1e-11);
 assert.ok(Math.abs(middle.ax)<1e-11);
 assert.ok(Math.abs(interpolate([start,finish],0).north)<1e-12);
 assert.ok(Math.abs(interpolate([start,finish],1).north-1)<1e-12);
 const q=septicAt(0,0,0,0,1,0,0,0,1,.25);
 assert.ok(Math.abs(q.position-(35*.25**4-84*.25**5+70*.25**6-20*.25**7))<1e-11);
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
test('ideal-gas density and analytical dynamic pressure use weather snapshot',()=>{
 const env={...initialEnvironment,temperature:15,pressure:1013.25,windSpeed:0};
 const rho=dryAirDensity(env);
 assert.ok(rho>1.22&&rho<1.23);
 const m=metrics(parseCsv(rows),env);
 assert.ok(Math.abs(m.peakDynamicPressure-(.5*rho*4))<1e-9);
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
 assert.equal(req.document.path.points[0].hard,true);
 assert.equal(req.document.path.points[0].exact,false);
 assert.equal(req.document.path.points.at(-1).hard,true);
 assert.equal(req.document.path.points.at(-1).exact,false);
 assert.ok(req.document.path.points.every(p=>!(p.hard&&p.exact)));
});
test('oversized hand drawing remains original and is submitted for documented repair',()=>{
 const p={...sampleProject,points:sampleProject.points.map((pt,i)=>i===2?{...pt,north:100,east:100}:pt)};
 const req=buildRequest(p);
 assert.equal(req.document.path.points[2].north_m,100);
 assert.equal(req.document.path.points[2].east_m,100);
});
test('all marked stops override incompatible exact-flythrough requests',()=>{
 const project={...sampleProject,points:sampleProject.points.map((p,i)=>i===1?{...p,hard:true,exact:true}:p)};
 const req=buildRequest(project);
 assert.equal(req.document.path.points[1].hard,true);
 assert.equal(req.document.path.points[1].exact,false);
});
test('invalid geometry and conflicting cruise cap are rejected',()=>{
 assert.throws(()=>validateProject({...sampleProject,cruise:5,speedCap:3}),/Cruise cannot/);
 assert.throws(()=>validateProject({...sampleProject,points:[{...sampleProject.points[0],north:999},sampleProject.points[1]]}),/North coordinate/);
});
