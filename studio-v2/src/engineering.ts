import type { Environment, Preview, Project, Sample } from './types';

export const initialEnvironment: Environment = {
  mode:'manual', latitude:30.62, longitude:-96.34,
  timestamp:new Date().toISOString(), windSpeed:0, windDirection:0,
  gust:0, temperature:20, pressure:1013.25, source:'User-specified manual scenario'
};
export const sampleProject: Project = {
  schema:1, title:'Untitled mission', cruise:1.5, speedCap:3, altitude:12,
  tolerance:0.5, yaw:0, environment:initialEnvironment,
  points:[
    {id:'P01',north:0,east:0,altitude:12,hard:true,exact:true},
    {id:'P02',north:12,east:9,altitude:12,hard:false,exact:false},
    {id:'P03',north:22,east:-7,altitude:12,hard:false,exact:false},
    {id:'P04',north:32,east:4,altitude:12,hard:true,exact:true}
  ]
};
export function finite(value:number, label:string, min=-Infinity, max=Infinity) {
  if (!Number.isFinite(value)||value<min||value>max) throw Error(label+' must be in ['+min+', '+max+']');
  return value;
}
export function validateProject(project: Project) {
  if (!project.points||project.points.length<2||project.points.length>8192) throw Error('Add 2–8,192 route points');
  finite(project.altitude,'Altitude',0.1,30);
  finite(project.cruise,'Preferred cruise',0.01,20);
  finite(project.speedCap,'Never-exceed speed',0.01,20);
  if (project.cruise>project.speedCap) throw Error('Cruise cannot exceed the never-exceed speed limit');
  finite(project.tolerance,'Path tolerance',0.01,10);
  finite(project.yaw,'Yaw',-3600,3600);
  if (!project.environment || !Number.isFinite(project.environment.windSpeed) || project.environment.windSpeed<0 || !Number.isFinite(project.environment.windDirection)) throw Error('Environment wind must be finite and nonnegative');
  if (!Number.isFinite(Date.parse(project.environment.timestamp))) throw Error('Invalid environment timestamp');
  finite(project.environment.latitude,'Latitude',-90,90);
  finite(project.environment.longitude,'Longitude',-180,180);
  finite(project.environment.windSpeed,'Wind speed',0,75);
  finite(project.environment.gust,'Wind gust',0,100);
  finite(project.environment.windDirection,'Wind from',-3600,3600);
  finite(project.environment.temperature,'Temperature',-100,80);
  finite(project.environment.pressure,'Pressure',100,1100);
  for(const p of project.points) {
    finite(p.north,'North coordinate',-500,500); finite(p.east,'East coordinate',-500,500);
    finite(p.altitude,'Waypoint altitude',0.1,30);
    // The closest-feasible planner can project oversize requests into the model envelope.
    // Preserve the original geometry and let the verifier disclose the adjustment.
  }
  return project;
}
export function buildRequest(project:Project) {
  validateProject(project);
  return { source:'drawing', planning_mode:'closest', document:{
    schema_version:1, release_class:'SITL_ONLY', mission_id:'studio-v2-draft',
    vehicle_model_id:'px4-x500-sitl-v1', environment_id:'empty-world-v1',
    frame:{name:'LOCAL_NED',units:'m',altitude_datum:'launch',origin_id:'launch-001'},
    objective_priority:['legal_and_safety','vehicle_and_environment','intent_fidelity','tracking_margin_and_energy','duration'],
    path:{kind:'polyline',closure:'open_stop',maximum_deviation_m:project.tolerance,
      points:project.points.map((p,i)=>({
        point_id:p.id||'point-'+i,north_m:p.north,east_m:p.east,down_m:-p.altitude,
        exact:i===0||i===project.points.length-1||p.exact,
        hard:i===0||i===project.points.length-1||p.hard
      }))},
    motion:{maximum_speed_m_s:project.speedCap,target_speed_m_s:project.cruise,
      time_mode:'minimize',requested_duration_s:null,yaw_mode:'fixed',fixed_yaw_deg:project.yaw,
      yaw_profile:null,look_at_north_m:null,look_at_east_m:null,look_at_down_m:null}
  }};
}
export function parseCsv(text:string):Sample[] {
  const lines=text.trim().split(/\r?\n/);
  if(lines.length<2) throw Error('CSV is empty');
  const headers=lines[0].split(',').map(x=>x.trim());
  const required=['time','x','y','z','vx','vy','vz'];
  if(required.some(key=>!headers.includes(key))) throw Error('CSV missing required trajectory fields: '+required.join(', '));
  const rows:Sample[]=[];
  let lastTime=-Infinity;
  for (let i=1;i<lines.length;i++){
    const values=lines[i].split(',');
    if(values.length!==headers.length) throw Error('Malformed trajectory row '+(i+1));
    const get=(key:string)=>{const raw=values[headers.indexOf(key)];if(raw===undefined||raw.trim()==='')throw Error('Missing '+key+' in row '+(i+1));const n=Number(raw);if(!Number.isFinite(n))throw Error('Non-finite '+key+' in row '+(i+1));return n;};
    const maybe=(key:string)=>headers.includes(key)?get(key):0;
    const time=get('time');
    if(time<=lastTime) throw Error('CSV time must be strictly increasing at row '+(i+1));
    lastTime=time;
    const north=get('x'),east=get('y'),altitude=-get('z');
    const vx=get('vx'),vy=get('vy'),vz=get('vz');
    const ax=maybe('ax'),ay=maybe('ay'),az=maybe('az');
    const jx=maybe('jx'),jy=maybe('jy'),jz=maybe('jz');
    rows.push({time,north,east,altitude,vx,vy,vz,ax,ay,az,jx,jy,jz,
      yaw:maybe('yaw'),yawRate:maybe('yaw_rate'),yawAcceleration:maybe('yaw_acceleration'),yawJerk:maybe('yaw_jerk'),speed:Math.hypot(vx,vy,vz),
      acceleration:Math.hypot(ax,ay,az),jerk:Math.hypot(jx,jy,jz)});
    if(rows.length>250000)throw Error('CSV exceeds 250,000 sample display budget');
  }
  return rows;
}
// Meteorological wind direction denotes the direction FROM which wind arrives.
// NED north/east velocity uses the opposite direction; vertical component is unknown.
export function windNE(speed:number, directionFromDeg:number) {
  const angle=directionFromDeg*Math.PI/180;
  return {north:-speed*Math.cos(angle),east:-speed*Math.sin(angle)};
}
// Dry ideal-gas approximation using the supplied near-surface pressure and temperature.
// Not a vertical atmosphere model, pitot measurement or dynamic flight simulator.
export function dryAirDensity(environment:Environment) {
  return environment.pressure*100/(287.05*(environment.temperature+273.15));
}
export function airspeed(sample:Sample, environment:Environment) {
  const w=windNE(environment.windSpeed,environment.windDirection);
  return Math.hypot(sample.vx-w.north,sample.vy-w.east,sample.vz);
}
export function metrics(samples:Sample[], env:Environment) {
  if(!samples.length)return null;
  let path=0,peak=0,accel=0,jerk=0,air=0;
  const density=dryAirDensity(env);
  for(let i=0;i<samples.length;i++){
    const s=samples[i];peak=Math.max(peak,s.speed);accel=Math.max(accel,s.acceleration);
    jerk=Math.max(jerk,s.jerk);air=Math.max(air,airspeed(s,env));
    if(i)path+=Math.hypot(s.north-samples[i-1].north,s.east-samples[i-1].east,s.altitude-samples[i-1].altitude);
  }
  return {duration:samples[samples.length-1].time-samples[0].time,
    distance:path,peak,accel,jerk,air,density,peakDynamicPressure:0.5*density*air*air};
}
/**
 * Exact degree <=7 Hermite reconstruction when two serialized reference
 * samples belong to a single polynomial phase. The planner's per-sample
 * (position,velocity,acceleration,jerk) at BOTH endpoints uniquely determine
 * the septic polynomial, unlike inaccurate linear drawing interpolation.
 * Phase boundaries must be emitted as separate samples by the exporter.
 */
export function septicAt(p0:number,v0:number,a0:number,j0:number,
                         p1:number,v1:number,a1:number,j1:number,h:number,s:number) {
  if(!Number.isFinite(h)||h<=0)throw Error('Invalid time interval');
  const c0=p0,c1=v0*h,c2=a0*h*h/2,c3=j0*h*h*h/6;
  const A=p1-c0-c1-c2-c3;
  const B=v1*h-c1-2*c2-3*c3;
  const C=a1*h*h-2*c2-6*c3;
  const D=j1*h*h*h-6*c3;
  const c4=35*A-15*B+2.5*C-D/6;
  const c5=-84*A+39*B-7*C+D/2;
  const c6=70*A-34*B+6.5*C-D/2;
  const c7=-20*A+10*B-2*C+D/6;
  const position=c0+s*(c1+s*(c2+s*(c3+s*(c4+s*(c5+s*(c6+s*c7))))));
  const velocity=(c1+s*(2*c2+s*(3*c3+s*(4*c4+s*(5*c5+s*(6*c6+s*7*c7))))))/h;
  const acceleration=(2*c2+s*(6*c3+s*(12*c4+s*(20*c5+s*(30*c6+s*42*c7)))))/(h*h);
  const jerk=(6*c3+s*(24*c4+s*(60*c5+s*(120*c6+s*210*c7))))/(h*h*h);
  return {position,velocity,acceleration,jerk};
}
export function interpolate(samples:Sample[], t:number):Sample|null {
  if(samples.length===0)return null;
  if(t<=samples[0].time)return samples[0];
  if(t>=samples[samples.length-1].time)return samples[samples.length-1];
  let low=0,high=samples.length-1;
  while(high-low>1){const mid=(low+high)>>1;if(samples[mid].time<=t)low=mid;else high=mid;}
  const a=samples[low],b=samples[high],h=b.time-a.time,s=(t-a.time)/h;
  const n=septicAt(a.north,a.vx,a.ax,a.jx,b.north,b.vx,b.ax,b.jx,h,s);
  const e=septicAt(a.east,a.vy,a.ay,a.jy,b.east,b.vy,b.ay,b.jy,h,s);
  const up=septicAt(a.altitude,-a.vz,-a.az,-a.jz,b.altitude,-b.vz,-b.az,-b.jz,h,s);
  const yaw=septicAt(a.yaw,a.yawRate,a.yawAcceleration,a.yawJerk,
                     b.yaw,b.yawRate,b.yawAcceleration,b.yawJerk,h,s);
  return {...a,time:t,north:n.position,east:e.position,altitude:up.position,
    vx:n.velocity,vy:e.velocity,vz:-up.velocity,
    ax:n.acceleration,ay:e.acceleration,az:-up.acceleration,
    jx:n.jerk,jy:e.jerk,jz:-up.jerk,
    yaw:yaw.position,yawRate:yaw.velocity,yawAcceleration:yaw.acceleration,yawJerk:yaw.jerk,
    speed:Math.hypot(n.velocity,e.velocity,up.velocity),
    acceleration:Math.hypot(n.acceleration,e.acceleration,up.acceleration),
    jerk:Math.hypot(n.jerk,e.jerk,up.jerk)};
}
export function getPreviewSummary(result:Preview) {
  const summary=result.summary||{};
  const speed=summary.speed_preferences as Record<string,unknown>|undefined;
  return {speed: speed||{},certificate:result.verification||{}};
}
export function exportBlob(name:string, contents:string, mime:string) {
  const url=URL.createObjectURL(new Blob([contents],{type:mime}));
  const link=document.createElement('a');link.href=url;link.download=name;link.click();
  setTimeout(()=>URL.revokeObjectURL(url),2000);
}
export function projectFromJson(raw:unknown):Project {
  if(!raw||typeof raw!=='object')throw Error('Invalid project');
  const p=raw as Project;
  if(p.schema!==1)throw Error('Unsupported project version');
  if(typeof p.title!=='string'||!p.environment||!['manual','live'].includes(p.environment.mode))
    throw Error('Invalid project metadata');
  validateProject(p);return p;
}
