import type {Environment, Sample} from './types';

export type AircraftLimits={mass_kg:number; maximum_thrust_n:number; maximum_tilt_deg:number};
export type ResponseSettings={dragArea:number; responseTime:number};
export type TrackPoint={time:number; north:number; east:number; height:number; error:number};
type Vec=[number,number,number];
const gravity=9.80665;

export function predictTrack(samples:Sample[], env:Environment, vehicle:AircraftLimits, settings:ResponseSettings){
  if(samples.length<2)throw Error('Verify a route first.');
  if(env.mode==='live'&&!env.fetchedAt)throw Error('Fetch weather for the selected location and time.');
  const {dragArea,responseTime}=settings;
  const {mass_kg:mass,maximum_thrust_n:thrust,maximum_tilt_deg:tilt}=vehicle;
  if(![mass,thrust,tilt,dragArea,responseTime].every(Number.isFinite)||mass<=0||thrust<=0||
      tilt<=0||tilt>=90||dragArea<0||responseTime<=0)throw Error('Check aircraft limits, drag area and response time.');
  const density=100*env.pressure/(287.05*(env.temperature+273.15));
  const angle=env.windDirection*Math.PI/180;
  const wind:Vec=[-env.windSpeed*Math.cos(angle),-env.windSpeed*Math.sin(angle),0];
  const maxAcceleration=thrust/mass, maxSlope=Math.tan(tilt*Math.PI/180);
  const drag=density*dragArea/(2*mass);
  const start=samples[0];
  const position:Vec=[start.north,start.east,start.altitude],velocity:Vec=[start.vx,start.vy,-start.vz];
  const points:TrackPoint[]=[{time:start.time,north:start.north,east:start.east,height:start.altitude,error:0}];
  let maxError=0,squaredIntegral=0,limitedTime=0;
  const maxStep=Math.min(.02,responseTime/12);
  const duration=samples[samples.length-1].time-start.time;
  if(duration/maxStep+samples.length>3_000_000)throw Error('This response time and route exceed the offline integration budget.');
  for(let i=1;i<samples.length;i++){
    const a=samples[i-1],b=samples[i];
    const span=b.time-a.time, count=Math.ceil(span/maxStep), dt=span/count;
    if(!(span>0))throw Error('Reference timestamps must increase.');
    const p0=[a.north,a.east,a.altitude],p1=[b.north,b.east,b.altitude];
    const v0=[a.vx,a.vy,-a.vz],v1=[b.vx,b.vy,-b.vz];
    const a0=[a.ax,a.ay,-a.az],a1=[b.ax,b.ay,-b.az];
    for(let k=0;k<count;k++){
      const t=k/count,force:Vec=[0,0,0];
      for(let axis=0;axis<3;axis++){
        const pos=p0[axis]+t*(p1[axis]-p0[axis]),vel=v0[axis]+t*(v1[axis]-v0[axis]);
        const feedForward=a0[axis]+t*(a1[axis]-a0[axis]);
        force[axis]=feedForward+(pos-position[axis])/(responseTime**2)+2*(vel-velocity[axis])/responseTime;
      }
      // Up-positive specific thrust. Prioritize vertical support before lateral tilt.
      force[2]+=gravity;
      const up=Math.max(0,Math.min(maxAcceleration,force[2]));
      const lateral=Math.hypot(force[0],force[1]);
      const lateralCap=Math.min(up*maxSlope,Math.sqrt(Math.max(0,maxAcceleration**2-up**2)));
      if(up!==force[2]||lateral>lateralCap)limitedTime+=dt;
      const scale=lateral>lateralCap?lateralCap/lateral:1;
      const previous=[...velocity];
      velocity[0]+=dt*force[0]*scale;
      velocity[1]+=dt*force[1]*scale;
      velocity[2]+=dt*(up-gravity);
      const relative=velocity.map((v,axis)=>v-wind[axis]);
      const dragDivisor=1+drag*Math.hypot(...relative)*dt;
      for(let axis=0;axis<3;axis++){
        velocity[axis]=wind[axis]+relative[axis]/dragDivisor;
        position[axis]+=.5*dt*(previous[axis]+velocity[axis]);
      }
    }
    const error=Math.hypot(position[0]-b.north,position[1]-b.east,position[2]-b.altitude);
    if(!Number.isFinite(error))throw Error('The offline tracking model diverged.');
    squaredIntegral+=span*(points[points.length-1].error**2+error**2)/2;
    maxError=Math.max(maxError,error);
    points.push({time:b.time,north:position[0],east:position[1],height:position[2],error});
  }
  return {points,maxError,rms:Math.sqrt(squaredIntegral/duration),limitedTime,airDensity:density,maximumStep:maxStep};
}
