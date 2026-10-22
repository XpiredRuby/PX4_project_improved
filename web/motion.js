/* Time-based reference playback. Full CSV samples are the source, never the
 * decimated display path. An imported measurement stays a separate data set. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PX4Motion=api;})(typeof window==='undefined'?globalThis:window,()=>{
 'use strict';
 const G=9.80665,DEG=180/Math.PI;
 const axes=[['x','vx','ax','jx'],['y','vy','ay','jy'],['z','vz','az','jz'],['yaw','yaw_rate','yaw_acceleration','yaw_jerk']];
 function parseCsv(text){
  if(typeof text!=='string'||text.length>24*1024*1024)throw Error('Use a trajectory CSV under 24 MB.');
  const lines=text.trim().split(/\r?\n/),head=lines.shift().replace(/^\uFEFF/,'').split(',');
  if(new Set(head).size!==head.length||!['time','x','y','z','yaw'].every(k=>head.includes(k)))throw Error('CSV needs time,x,y,z,yaw in seconds, metres and radians (LOCAL_NED).');
  if(lines.length<2||lines.length>100000)throw Error('CSV needs 2–100000 samples.');
  const rows=lines.map(line=>{const cells=line.split(',');if(cells.length!==head.length)throw Error('CSV row does not match its header.');const row={};head.forEach((k,i)=>{if(k==='phase'){row[k]=cells[i];return;}if(!cells[i].trim()||!Number.isFinite(Number(cells[i])))throw Error('CSV contains an empty or non-finite value.');row[k]=Number(cells[i]);});return row;});
  if(rows[0].time<0||rows.some((r,i)=>i&&r.time<=rows[i-1].time))throw Error('CSV time must be nonnegative and strictly increasing.');
  return rows;
 }
 function indexAt(rows,t){let lo=0,hi=rows.length-1;while(lo<hi){const mid=Math.ceil((lo+hi)/2);if(rows[mid].time<=t)lo=mid;else hi=mid-1;}return lo;}
 function measurementRows(text){const rows=parseCsv(text);for(let i=1;i<rows.length;i++){const previous=rows[i-1].yaw,delta=rows[i].yaw-previous;rows[i].yaw=previous+Math.atan2(Math.sin(delta),Math.cos(delta));}return rows;}
 function hermite(a,b,keys,u,dt){
  const [p,v,acc,j]=keys;
  if(!keys.every(k=>Number.isFinite(a[k])&&Number.isFinite(b[k]))){const value=a[p]+u*(b[p]-a[p]);return [value,(b[p]-a[p])/dt,0,0];}
  const c=[a[p],dt*a[v],dt*dt*a[acc]/2,dt**3*a[j]/6];
  const B=b[p]-c.reduce((s,n)=>s+n,0),D=dt*b[v]-c[1]-2*c[2]-3*c[3],A=dt*dt*b[acc]-2*c[2]-6*c[3],J=dt**3*b[j]-6*c[3];
  c.push(35*B-15*D+2.5*A-J/6,-84*B+39*D-7*A+J/2,70*B-34*D+6.5*A-J/2,-20*B+10*D-2*A+J/6);
  return [0,1,2,3].map(order=>{let sum=0;for(let k=c.length-1;k>=order;k--){let multiplier=1;for(let n=0;n<order;n++)multiplier*=k-n;sum=sum*u+c[k]*multiplier;}return sum/dt**order;});
 }
 function sampleAt(rows,t){
  if(t<=rows[0].time)return {...rows[0],sampleIndex:0,interpolated:false};
  const i=indexAt(rows,t),a=rows[i],b=rows[i+1];
  if(!b||Math.abs(t-a.time)<1e-10)return {...a,sampleIndex:i,interpolated:false};
  const dt=b.time-a.time,u=(t-a.time)/dt,out={time:t,phase:a.phase,sampleIndex:i,interpolated:true};
  for(const keys of axes){const values=hermite(a,b,keys,u,dt);keys.forEach((k,n)=>out[k]=values[n]);}
  return out;
 }
 function attitude(row){
  // Ideal no-drag flatness reference, not measured roll/pitch. Body z is down.
  const normalize=v=>{const n=Math.hypot(...v);return v.map(x=>x/n);},cross=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
  const z=normalize([-(row.ax||0),-(row.ay||0),G-(row.az||0)]),yHeading=[-Math.sin(row.yaw),Math.cos(row.yaw),0];
  const x=normalize(cross(yHeading,z)),y=cross(z,x);
  return {x,y,z,roll:Math.atan2(y[2],z[2]),pitch:Math.asin(Math.max(-1,Math.min(1,-x[2]))),tilt:Math.acos(Math.max(-1,Math.min(1,z[2])))};
 }
 const phaseNames={TAKEOFF:'Takeoff',ROUTE:'Follow route',TURN_HOME:'Turn toward home',RETURN_HOME:'Return home',LAND:'Landing',TURN:'Turn in place',LEG:'Route leg',HOLD:'Hover'};
 function eventsFrom(result){
  const phases=result.mission.phases,route=phases.find(p=>p.name==='ROUTE'),events=[];
  for(const p of phases)events.push({time:p.start_time_s,end:p.end_time_s,kind:p.name,label:phaseNames[p.name]||p.name});
  let turn=0,point=0;
  for(const p of (result.summary.route_phases||[])){
   if(p.name==='TURN'||p.name==='HOLD')events.push({time:route.start_time_s+p.start_time_s,end:route.start_time_s+p.end_time_s,kind:p.name,label:p.name==='TURN'?'Stop & turn '+(++turn):'Hover'});
   else if(p.name==='LEG')events.push({time:route.start_time_s+p.end_time_s,end:route.start_time_s+p.end_time_s,kind:'STOP',label:'Reach point '+(++point)});
  }
  events.push({time:result.mission.total_duration_s,end:result.mission.total_duration_s,kind:'END',label:'Ground endpoint'});
  return events.sort((a,b)=>a.time-b.time||a.end-b.end).map((e,i)=>({...e,id:i+1}));
 }
 function stateLabel(row,events){
  const activeEvents=events.filter(e=>e.time<=row.time+1e-9&&e.end>row.time+1e-9);
  const active=activeEvents.find(e=>e.kind==='TURN'||e.kind==='HOLD')||activeEvents.at(-1);
  if(active)return active.label;
  if(row.time>=events.at(-1).time)return 'Ground endpoint';
  return Math.hypot(row.vx||0,row.vy||0,row.vz||0)<.01?'Stopped':'Moving';
 }
 function advance(time,wallSeconds,rate,end){return Math.min(end,time+Math.max(0,wallSeconds)*rate);}
 const api={parseCsv,measurementRows,indexAt,hermite,sampleAt,attitude,eventsFrom,stateLabel,advance,ready:false};
 if(typeof document==='undefined')return api;
 const $=id=>document.getElementById(id),make=(tag,cls,text)=>{const e=document.createElement(tag);if(cls)e.className=cls;if(text)e.textContent=text;return e;};
 let panel,transport,overlay,scene,rows=[],routeRows=[],events=[],result,measured=null,current=0,playing=false,frame=0,lastWall=null,mapState=null,sceneView='3d',fit=null,generation=0;
 const clock=t=>{const m=Math.floor(t/60),s=(t-m*60).toFixed(2).padStart(5,'0');return m+':'+s;};
 function pause(){playing=false;cancelAnimationFrame(frame);lastWall=null;if($('motionPlay'))$('motionPlay').textContent='Play';}
 function seek(t){if(!rows.length)return;current=Math.max(rows[0].time,Math.min(rows.at(-1).time,t));render();}
 function tick(wall){if(!playing)return;if(lastWall!==null)current=advance(current,(wall-lastWall)/1000,Number($('motionRate').value),rows.at(-1).time);lastWall=wall;render();if(current>=rows.at(-1).time){pause();return;}frame=requestAnimationFrame(tick);}
 function togglePlay(){if(playing){pause();return;}if(current>=rows.at(-1).time)current=rows[0].time;playing=true;lastWall=null;$('motionPlay').textContent='Pause';frame=requestAnimationFrame(tick);}
 function step(direction){pause();const i=indexAt(rows,current),next=direction>0?i+1:(Math.abs(rows[i].time-current)<1e-8?i-1:i);seek(rows[Math.max(0,Math.min(rows.length-1,next))].time);}
 function line(ctx,data,project,color,width=2){if(!data.length)return;ctx.beginPath();for(const [i,row] of data.entries()){const q=project(row);i?ctx.lineTo(...q):ctx.moveTo(...q);}ctx.strokeStyle=color;ctx.lineWidth=width;ctx.stroke();}
 function marker(ctx,point,row,ui,color='#f2cf86'){
  ctx.save();ctx.translate(...point);ctx.rotate(row.yaw);ctx.fillStyle='#12232b';ctx.strokeStyle=color;ctx.lineWidth=2*ui;
  ctx.beginPath();ctx.moveTo(0,-13*ui);ctx.lineTo(9*ui,11*ui);ctx.lineTo(0,6*ui);ctx.lineTo(-9*ui,11*ui);ctx.closePath();ctx.fill();ctx.stroke();ctx.restore();
 }
 function drawOverlay(){
  if(!overlay||!mapState)return;const ctx=overlay.getContext('2d');ctx.clearRect(0,0,900,630);if(!api.ready||window.PX4Compare?.view==='input')return;
  const {map,ui,scale}=mapState,row=sampleAt(rows,current),project=r=>map([r.y,r.x]);
  ctx.save();ctx.lineCap='round';
  ctx.setLineDash([5*ui,6*ui]);line(ctx,rows.filter(r=>r.phase==='RETURN_HOME'),project,'#8bac9c',1.5*ui);ctx.setLineDash([]);
  // Only a short recent trail is bright; the full route remains in the base map.
  const from=indexAt(rows,Math.max(0,current-8)),to=indexAt(rows,current);
  line(ctx,[...rows.slice(from,to+1),row],project,'#f4d08b',3*ui);
  const marked=events.filter(e=>e.kind==='TURN'||['TAKEOFF','RETURN_HOME','LAND','END'].includes(e.kind));
  const stride=Math.max(1,Math.ceil(marked.length/8));
  const groups=[];
  for(let i=0;i<marked.length;i+=stride){const e=marked[i],p=project(sampleAt(rows,e.time));if(p[0]<25||p[0]>875||p[1]<40||p[1]>570)continue;const group=groups.find(g=>Math.hypot(g.p[0]-p[0],g.p[1]-p[1])<20*ui);if(group)group.ids.push(e.id);else groups.push({p,ids:[e.id]});}
  for(const {p,ids} of groups){const text=ids.join(', ');ctx.font=`${10*ui}px system-ui`;const w=ctx.measureText(text).width+12*ui,left=Math.max(2,Math.min(898-w,p[0]-w/2));ctx.fillStyle='#1a2833';ctx.strokeStyle='#8a9ca6';ctx.lineWidth=ui;ctx.beginPath();ctx.moveTo(...p);ctx.lineTo(p[0],p[1]-12*ui);ctx.stroke();ctx.fillRect(left,p[1]-30*ui,w,18*ui);ctx.strokeRect(left,p[1]-30*ui,w,18*ui);ctx.fillStyle='#d1dce4';ctx.textAlign='center';ctx.fillText(text,left+w/2,p[1]-17*ui);}
  if(measured){const off=Number($('motionOffset').value)||0,mt=current+off;if(mt>=measured[0].time&&mt<=measured.at(-1).time){const m=sampleAt(measured,mt);line(ctx,measured.filter((_,i)=>i%Math.max(1,Math.ceil(measured.length/1200))===0),project,'#ce9ff4',1.5*ui);marker(ctx,project(m),m,ui,'#ce9ff4');}}
  marker(ctx,project(row),row,ui);ctx.textAlign='left';
  const desired=100*ui/scale,power=10**Math.floor(Math.log10(desired)),length=([1,2,5,10].filter(v=>v*power<=desired).at(-1)||1)*power;
  const left=30*ui,y=570;ctx.fillStyle='#0e1822e8';ctx.fillRect(left-8*ui,y-25*ui,length*scale+80*ui,36*ui);ctx.strokeStyle='#b9cbd5';ctx.lineWidth=1.5*ui;ctx.beginPath();ctx.moveTo(left,y-5*ui);ctx.lineTo(left,y);ctx.lineTo(left+length*scale,y);ctx.lineTo(left+length*scale,y-5*ui);ctx.stroke();ctx.fillStyle='#b9cbd5';ctx.font=`${10*ui}px system-ui`;ctx.fillText(length+' m · equal N/E scale',left,y-10*ui);ctx.restore();
 }
 const iso=row=>[.8660254*(row.y-row.x),.5*(row.y+row.x)+row.z];
 function sceneFit(){const projected=rows.map(iso),min=[Infinity,Infinity],max=[-Infinity,-Infinity];for(const p of projected)for(let k=0;k<2;k++){min[k]=Math.min(min[k],p[k]);max[k]=Math.max(max[k],p[k]);}for(const p of [iso({x:0,y:0,z:0}),iso({x:5,y:0,z:0}),iso({x:0,y:5,z:0})])for(let k=0;k<2;k++){min[k]=Math.min(min[k],p[k]);max[k]=Math.max(max[k],p[k]);}const scale=Math.min(760/Math.max(5,max[0]-min[0]),275/Math.max(5,max[1]-min[1]));return {scale,project:r=>{const p=iso(r);return [450+(p[0]-(min[0]+max[0])/2)*scale,190+(p[1]-(min[1]+max[1])/2)*scale];}};}
 function drawScene(row){
  const c=scene.getContext('2d'),ui=Math.max(1,900/(scene.getBoundingClientRect().width||900));c.clearRect(0,0,900,380);c.fillStyle='#101c29';c.fillRect(0,0,900,380);c.font=`${12*ui}px system-ui`;c.fillStyle='#9cb3c0';
  if(sceneView==='height'){
   const top=rows.reduce((v,r)=>Math.max(v,-r.z),1),end=rows.at(-1).time,map=r=>[55+r.time/end*790,320+r.z/top*260];
   c.strokeStyle='#344652';for(let i=0;i<=4;i++){const y=320-i*65;c.beginPath();c.moveTo(55,y);c.lineTo(845,y);c.stroke();c.fillText((top*i/4).toFixed(1)+' m',5,y+4);c.fillText(clock(end*i/4),40+790*i/4,345);}
   line(c,rows,map,'#6fa7b0',2);const p=map(row);c.fillStyle='#f2cf86';c.beginPath();c.arc(...p,6,0,Math.PI*2);c.fill();c.fillText('Height above launch · time in min:sec',55,25);return;
  }
  const project=fit.project,origin=project({x:0,y:0,z:0});
  for(const [axis,color,end] of [['N','#9db9bb',{x:5,y:0,z:0}],['E','#9db9bb',{x:0,y:5,z:0}],['Up','#9db9bb',{x:0,y:0,z:-5}]]){const p=project(end);line(c,[{x:0,y:0,z:0},end],project,color);c.fillText(axis+' 5 m',p[0]+5,p[1]);}
  line(c,rows.filter((_,i)=>i%Math.max(1,Math.ceil(rows.length/1800))===0),project,'#46717b',1.7);
  const pose=project(row),shadow=project({...row,z:0});c.setLineDash([4,5]);line(c,[{...row,z:0},row],project,'#819388',1);c.setLineDash([]);c.fillStyle='#e2bd6c33';c.beginPath();c.ellipse(...shadow,9,4,0,0,Math.PI*2);c.fill();
  const att=attitude(row),symbol=18/fit.scale;
  for(const vector of [att.x,att.y]){const end=sign=>({x:row.x+sign*symbol*vector[0],y:row.y+sign*symbol*vector[1],z:row.z+sign*symbol*vector[2]});line(c,[end(-1),end(1)],project,'#f2cf86',3);for(const sign of [-1,1]){const p=project(end(sign));c.strokeStyle='#e9d5b2';c.beginPath();c.ellipse(...p,7,3,0,0,Math.PI*2);c.stroke();}}
  const nose=project({x:row.x+symbol*1.5*att.x[0],y:row.y+symbol*1.5*att.x[1],z:row.z+symbol*1.5*att.x[2]});line(c,[row,{x:row.x+symbol*1.5*att.x[0],y:row.y+symbol*1.5*att.x[1],z:row.z+symbol*1.5*att.x[2]}],project,'#8ce1cf',3);
  c.fillStyle='#d8e4e6';c.fillText((-row.z).toFixed(2)+' m',Math.min(690,pose[0]+26),Math.max(40,pose[1]-10));c.fillStyle='#9cb3c0';c.fillText('Equal axes · drone symbol enlarged',24,360);
 }
 function render(){
  if(!api.ready)return;const row=sampleAt(rows,current),att=attitude(row),speed=Math.hypot(row.vx||0,row.vy||0,row.vz||0);
  $('motionTime').textContent=clock(current)+' / '+clock(rows.at(-1).time);$('motionSeek').value=String(current);$('motionSeek').setAttribute('aria-valuetext',clock(current)+' · '+stateLabel(row,events));
  $('motionPhase').textContent=stateLabel(row,events);$('motionHeight').textContent=(-row.z).toFixed(2)+' m';$('motionSpeed').textContent=speed.toFixed(2)+' m/s';$('motionHeading').textContent=((row.yaw*DEG%360+360)%360).toFixed(1)+'°';
  $('motionLive').textContent=`${stateLabel(row,events)} · Height ${(-row.z).toFixed(2)} m · Speed ${speed.toFixed(2)} m/s · Heading ${((row.yaw*DEG%360+360)%360).toFixed(1)}°`;
  $('motionPosition').textContent=`N ${row.x.toFixed(3)} · E ${row.y.toFixed(3)} · Up ${(-row.z).toFixed(3)} m`;
  $('motionAttitude').textContent=`Roll ${(att.roll*DEG).toFixed(2)}° · Pitch ${(att.pitch*DEG).toFixed(2)}° · Yaw rate ${((row.yaw_rate||0)*DEG).toFixed(2)}°/s`;
  $('motionSample').textContent=`Sample ${row.sampleIndex+1} / ${rows.length}${row.interpolated?' · interpolated between exported samples':' · exact exported sample'}`;
  $('motionMeasured').textContent=measured?'Imported measurement loaded; purple marker appears only during overlapping times.':'Measured motion: no telemetry loaded.';
  if(measured){const mt=current+(Number($('motionOffset').value)||0);if(mt>=measured[0].time&&mt<=measured.at(-1).time){const m=sampleAt(measured,mt);$('motionMeasured').textContent=`Imported measured position · separation ${Math.hypot(m.x-row.x,m.y-row.y,m.z-row.z).toFixed(3)} m at this time. Source and clock alignment supplied by you.`;}}
  drawOverlay();drawScene(row);
 }
 api.mapChanged=(map,ui,scale)=>{mapState={map,ui,scale};drawOverlay();};
 api.routeSamples=()=>api.ready?routeRows:null;
 api.clear=()=>{generation++;pause();api.ready=false;rows=[];routeRows=[];events=[];result=null;measured=null;if(panel){panel.hidden=true;transport.hidden=true;$('motionOffset').value='0';$('motionFile').value='';}if(overlay)overlay.getContext('2d').clearRect(0,0,900,630);};
 api.show=value=>{
  api.clear();if(!panel||!value.mission?.trajectory_csv)return;result=value;
  try{rows=parseCsv(value.mission.trajectory_csv);const route=value.mission.phases.find(p=>p.name==='ROUTE');routeRows=rows.filter(r=>r.time>=route.start_time_s&&r.time<=route.end_time_s);events=eventsFrom(value);fit=sceneFit();current=rows[0].time;api.ready=true;panel.hidden=false;transport.hidden=false;
   $('motionSeek').max=String(rows.at(-1).time);$('motionSeek').min=String(rows[0].time);$('motionEvents').replaceChildren();
   const first=make('option','', 'Jump to a flight event…');first.value='';$('motionEvents').append(first);
   for(const e of events){const o=make('option','',`${e.id}. ${clock(e.time)} · ${e.label}${e.end>e.time?' · '+(e.end-e.time).toFixed(1)+' s':''}`);o.value=String(e.time);$('motionEvents').append(o);}
   $('motionSource').textContent=`Planned reference · ${rows.length.toLocaleString()} full-resolution CSV samples · ${value.simulation_status==='NOT_RUN'?'Gazebo has not run':'no measured track attached'}`;render();
  }catch(error){panel.hidden=false;$('motionSource').textContent='Playback unavailable: '+error.message;}
 };
 document.addEventListener('DOMContentLoaded',()=>{
  const editor=document.querySelector('.map-card');if(!editor)return;
  overlay=make('canvas','motion-overlay');overlay.width=900;overlay.height=630;overlay.setAttribute('aria-hidden','true');$('route').parentElement.append(overlay);
  panel=make('section','motion-panel');panel.id='motionPanel';panel.hidden=true;panel.setAttribute('aria-label','Full mission playback');
  panel.innerHTML='<div class="motion-title"><div><p class="overline">FLIGHT PLAYBACK</p><h2>Follow every movement</h2></div><span id="motionPhase" class="motion-phase"></span></div><p id="motionSource" class="muted"></p><div class="motion-controls"><button id="motionPrevious" aria-label="Previous exported sample">‹ Sample</button><button id="motionPlay">Play</button><button id="motionNext" aria-label="Next exported sample">Sample ›</button><label>Playback speed<select id="motionRate"><option value="0.25">0.25× slow</option><option value="1" selected>1× real time</option><option value="4">4×</option><option value="16">16×</option></select></label><output id="motionTime"></output></div><label class="sr-only" for="motionSeek">Mission time</label><input id="motionSeek" type="range" min="0" max="1" step="0.01" value="0"><div class="motion-readouts"><div><span>Height</span><strong id="motionHeight"></strong></div><div><span>Speed</span><strong id="motionSpeed"></strong></div><div><span>Heading · 0° north</span><strong id="motionHeading"></strong></div></div><p id="motionPosition" class="motion-position"></p><label class="sr-only" for="motionEvents">Jump to flight event</label><select id="motionEvents" aria-label="Jump to flight event"></select><p class="muted">Numbered map markers match this event list. Repeated visits keep their own timestamps.</p><div class="motion-views" role="group" aria-label="Motion view"><button id="motion3d" aria-pressed="true">3D overview</button><button id="motionHeightView" aria-pressed="false">Height over time</button></div><canvas id="motionScene" width="900" height="380" role="img" aria-label="Planned three-dimensional flight position and attitude"></canvas><p id="motionAttitude" class="muted"></p><p id="motionSample" class="muted"></p><details class="motion-method"><summary>Scale, accuracy and measured motion</summary><p>1× means one mission second per real second. Map north/east use the same metres-to-pixels scale. The 3D view uses equal world-axis scale with no amplified height. The drone symbol is enlarged for visibility, not an airframe dimension model.</p><p>Playback uses every exported CSV sample, including takeoff, stops, heading changes, return and descent. Sample buttons select exact exported states; between samples, position and heading are interpolated using their available derivatives. Roll and pitch are ideal references inferred from commanded acceleration and yaw without aerodynamic drag. They are not measured attitudes.</p><p>This shows the planned reference. Exact physical motion requires a matching PX4/Gazebo or aircraft telemetry recording. Ground height zero alone does not prove touchdown or disarm.</p><label for="motionFile">Compare measured CSV (optional, stays on this device)</label><input id="motionFile" type="file" accept=".csv,text/csv"><p>Required: time,x,y,z,yaw. LOCAL_NED metres, time in seconds, yaw in radians; zero time must align with mission start. Velocity/acceleration/jerk columns are optional. This imports your measurements; it does not start Gazebo.</p><label for="motionOffset">Measurement clock offset (s): measured time = mission time + offset</label><input id="motionOffset" type="number" value="0" step="any"><button id="motionRemove">Remove measured track</button></details><p id="motionMeasured" class="muted" role="status"></p>';
  const legend=make('div','motion-legend');legend.innerHTML='<span class="motion-key-current">Drone now + recent motion</span><span class="motion-key-return">Return home</span><span class="motion-key-measured">Imported measurement</span>';panel.insertBefore(legend,panel.querySelector('#motionEvents'));
  transport=make('div','motion-transport');transport.hidden=true;transport.setAttribute('aria-label','Playback controls');transport.append(make('p','motion-live',''));transport.firstChild.id='motionLive';transport.append(panel.querySelector('.motion-controls'),panel.querySelector('label[for="motionSeek"]'),panel.querySelector('#motionSeek'));document.querySelector('.map-surface').before(transport);
  editor.append(panel);scene=$('motionScene');$('motionPlay').onclick=togglePlay;$('motionPrevious').onclick=()=>step(-1);$('motionNext').onclick=()=>step(1);$('motionSeek').oninput=()=>{pause();seek(Number($('motionSeek').value));};
  $('motionEvents').onchange=()=>{if($('motionEvents').value!==''){pause();seek(Number($('motionEvents').value));}};
  for(const [id,view] of [['motion3d','3d'],['motionHeightView','height']])$(id).onclick=()=>{sceneView=view;$('motion3d').setAttribute('aria-pressed',String(view==='3d'));$('motionHeightView').setAttribute('aria-pressed',String(view==='height'));render();};
  $('motionRate').onchange=()=>{lastWall=null;};$('motionOffset').oninput=render;$('motionRemove').onclick=()=>{measured=null;$('motionFile').value='';render();};
  $('motionFile').onchange=async e=>{pause();const submitted=generation;try{const file=e.target.files[0];if(!file)return;if(file.size>24*1024*1024)throw Error('Use a CSV under 24 MB.');const parsed=measurementRows(await file.text());if(submitted!==generation)return;measured=parsed;render();}catch(error){if(submitted!==generation)return;measured=null;render();$('motionMeasured').textContent='Measurement import failed: '+error.message;}};
  document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
 });
 return api;
});
