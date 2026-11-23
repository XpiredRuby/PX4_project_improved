import test from 'node:test';
import assert from 'node:assert/strict';
import {predictTrack} from '../src/prediction.ts';
import {initialEnvironment} from '../src/engineering.ts';
const route=Array.from({length:201},(_,i)=>({time:i*.05,north:0,east:0,altitude:10,vx:0,vy:0,vz:0,ax:0,ay:0,az:0}));
const vehicle={mass_kg:2,maximum_thrust_n:35,maximum_tilt_deg:25};
const settings={responseTime:.8,dragArea:.08};
test('hover stays stationary in calm air, east wind displaces west, removing drag removes displacement',()=>{
  const calm=predictTrack(route,initialEnvironment,vehicle,settings);
  assert.equal(calm.maxError,0);
  const env={...initialEnvironment,windSpeed:5,windDirection:90};
  const wind=predictTrack(route,env,vehicle,settings);
  assert.ok(wind.points.at(-1).east<-.1);
  assert.ok(Math.abs(wind.points.at(-1).north)<1e-12);
  assert.equal(predictTrack(route,env,vehicle,{...settings,dragArea:0}).maxError,0);
});
test('insufficient thrust accumulates height loss and pending forecasts cannot produce a prediction',()=>{
  const limited=predictTrack(route,initialEnvironment,{...vehicle,maximum_thrust_n:5},settings);
  assert.ok(limited.limitedTime>9.99);
  assert.ok(limited.points.at(-1).height<10);
  assert.throws(()=>predictTrack(route,{...initialEnvironment,mode:'live'},vehicle,settings),/Fetch weather/);
});
