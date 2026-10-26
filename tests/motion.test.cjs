const {test}=require('node:test'),assert=require('node:assert/strict');
const {parseCsv,measurementRows,sampleAt,attitude,eventsFrom,stateLabel,advance,headingArrow}=require('../web/motion.js');
const near=(a,b,tol=1e-8)=>assert.ok(Math.abs(a-b)<tol,`${a} != ${b}`);
const base=t=>({time:t,x:0,y:0,z:-5,yaw:0,vx:0,vy:0,vz:0,ax:0,ay:0,az:0,jx:0,jy:0,jz:0,yaw_rate:0,yaw_acceleration:0,yaw_jerk:0,phase:'ROUTE'});
test('CSV retains every exported sample and rejects ambiguous or corrupt clocks',()=>{
 const text='time,x,y,z,yaw,phase\n0,0,0,0,0,TAKEOFF\n0.05,0,0,-0.1,0,TAKEOFF\n0.1,0,0,-0.2,0,TAKEOFF\n';
 const rows=parseCsv(text);assert.equal(rows.length,3);assert.equal(rows[1].z,-.1);
 for(const bad of [text.replace('\n0.1,','\n0.05,'),text.replace('-0.2','NaN'),text.replace('time,x','time,time'),text.replace('-0.2',''),text.replace('time,x','t,x')])assert.throws(()=>parseCsv(bad));
});
test('sample stepping is exact at knots, clamps endpoints and uses physical timestamps',()=>{
 const rows=[base(0),{...base(.0499),x:1,yaw:Math.PI},base(.0998)];
 assert.equal(sampleAt(rows,.0499).x,1);assert.equal(sampleAt(rows,.0499).interpolated,false);
 assert.equal(sampleAt(rows,-1).time,0);assert.equal(sampleAt(rows,100).time,.0998);
 assert.equal(sampleAt(rows,.02).interpolated,true);
});
test('septic interpolation reconstructs known position and derivative polynomial',()=>{
 const coefficients=[.4,-.2,.8,-.5,.1,.3,-.15,.07];
 function polynomial(t,order){let result=0;for(let k=order;k<coefficients.length;k++){let c=coefficients[k];for(let j=0;j<order;j++)c*=k-j;result+=c*t**(k-order);}return result;}
 const row=t=>({...base(t),x:polynomial(t,0),vx:polynomial(t,1),ax:polynomial(t,2),jx:polynomial(t,3),yaw:2*polynomial(t,0),yaw_rate:2*polynomial(t,1),yaw_acceleration:2*polynomial(t,2),yaw_jerk:2*polynomial(t,3)});
 for(const t of [.251,.263,.279,.299]){const r=sampleAt([row(.25),row(.3)],t);near(r.x,polynomial(t,0));near(r.vx,polynomial(t,1));near(r.ax,polynomial(t,2),1e-7);near(r.jx,polynomial(t,3),1e-5);near(r.yaw,2*polynomial(t,0));}
});
test('a stopped turn changes yaw while holding position',()=>{
 const rows=[{...base(0),yaw:0},{...base(2),yaw:Math.PI}];
 const mid=sampleAt(rows,1);near(mid.x,0);near(mid.y,0);near(mid.z,-5);near(mid.yaw,Math.PI/2);assert.ok(mid.yaw_rate>0);
});
test('ideal attitude follows acceleration, with an orthonormal frame',()=>{
 const hover=attitude(base(0));near(hover.roll,0);near(hover.pitch,0);
 const tilt=attitude({...base(0),ax:1});near(tilt.tilt,Math.atan(1/9.80665));assert.ok(tilt.pitch<0);
 for(const a of [tilt.x,tilt.y,tilt.z])near(Math.hypot(...a),1);
 near(tilt.x.reduce((s,v,i)=>s+v*tilt.z[i],0),0);
 const east=attitude({...base(0),yaw:Math.PI/2});near(east.x[0],0);near(east.x[1],1);
});
test('events retain separate repeated visits and route-relative stop times',()=>{
 const result={mission:{total_duration_s:30,phases:[{name:'TAKEOFF',start_time_s:0,end_time_s:10},{name:'ROUTE',start_time_s:10,end_time_s:20},{name:'LAND',start_time_s:20,end_time_s:30}]},summary:{route_phases:[{name:'LEG',start_time_s:0,end_time_s:2},{name:'TURN',start_time_s:2,end_time_s:4},{name:'LEG',start_time_s:4,end_time_s:10}]}};
 const events=eventsFrom(result),turn=events.find(e=>e.kind==='TURN');assert.equal(turn.time,12);assert.equal(turn.end,14);
 assert.match(stateLabel({...base(13)},events),/Stop & turn/);assert.equal(stateLabel(base(30),events),'Ground endpoint');
 assert.equal(events.filter(e=>e.kind==='STOP').length,2);
});
test('1x is wall-clock time and fast playback does not alter mission times',()=>{
 near(advance(10,.2,1,30),10.2);near(advance(10,.2,16,30),13.2);assert.equal(advance(29,5,1,30),30);assert.equal(advance(10,-3,1,30),10);
});
test('a turn at the first route timestamp takes priority over the route phase',()=>{
 const events=[{time:10,end:13,kind:'TURN',label:'Stop & turn 1'},{time:10,end:30,kind:'ROUTE',label:'Follow route'},{time:30,end:30,kind:'END',label:'Ground endpoint'}];
 assert.equal(stateLabel(base(11),events),'Stop & turn 1');
 assert.equal(stateLabel(base(13),events),'Follow route');
});
test('measurement with position only is interpolated without fabricating dynamics',()=>{
 const rows=parseCsv('time,x,y,z,yaw\n0,0,0,0,0\n1,2,3,-4,1');
 const mid=sampleAt(rows,.5);near(mid.x,1);near(mid.y,1.5);near(mid.z,-2);near(mid.yaw,.5);
});
test('wrapped measured headings take the short turn across north',()=>{
 const rows=measurementRows('time,x,y,z,yaw\n0,0,0,0,3.13\n1,0,0,0,-3.13');
 assert.ok(Math.abs(rows[1].yaw-rows[0].yaw)<.03);near(sampleAt(rows,.5).yaw,Math.PI);
});
test('arrow nose faces north, east, south and west in NED yaw order',()=>{
 for(const [yaw,x,y] of [[0,0,-20],[Math.PI/2,20,0],[Math.PI,0,20],[3*Math.PI/2,-20,0]]){const nose=headingArrow(yaw)[0];near(nose[0],x);near(nose[1],y);}
});
test('arrow rotates continuously through north and during a stationary turn',()=>{
 const a=headingArrow(2*Math.PI-.001)[0],b=headingArrow(2*Math.PI+.001)[0];assert.ok(Math.hypot(a[0]-b[0],a[1]-b[1])<.05);
 const r=sampleAt([{...base(0),yaw:0},{...base(2),yaw:Math.PI}],1),nose=headingArrow(r.yaw)[0];near(r.x,0);near(r.y,0);near(nose[0],20);near(nose[1],0);
});
test('measurement outages remain blank and missing derivatives remain absent',()=>{
 const {measurementAt}=require('../web/motion.js');
 const rows=measurementRows('time,x,y,z,yaw\n0,0,0,0,0\n0.1,1,0,0,0\n10,10,0,0,0');
 const mid=measurementAt(rows,.05);near(mid.x,.5);assert.equal(mid.vx,undefined);assert.equal(mid.ax,undefined);
 assert.equal(measurementAt(rows,5),null);assert.equal(measurementAt(rows,-1),null);assert.equal(measurementAt(rows,11),null);assert.equal(measurementAt(rows,10).x,10);
});
test('display decimation keeps the last exported endpoint without changing playback rows',()=>{
 const {displayRows}=require('../web/motion.js');const rows=Array.from({length:10001},(_,i)=>base(i*.05)),copy=JSON.stringify(rows),display=displayRows(rows,100);
 assert.ok(display.length<=102);assert.equal(display.at(-1),rows.at(-1));assert.equal(JSON.stringify(rows),copy);assert.equal(sampleAt(rows,400).time,400);
});
