import { useId, useMemo } from 'react';
import type { Environment, Sample } from './types';
import { airspeed } from './engineering';

type Field = 'speed'|'airspeed'|'acceleration'|'jerk'|'altitude'|'yaw';
export const fields:Record<Field,{label:string;unit:string}> = {
  speed:{label:'3D speed',unit:'m/s'},
  airspeed:{label:'Wind-relative airspeed',unit:'m/s'},
  acceleration:{label:'Acceleration',unit:'m/s²'},
  jerk:{label:'Jerk',unit:'m/s³'},
  altitude:{label:'Height above launch',unit:'m'},
  yaw:{label:'Yaw',unit:'rad'}
};
export const fieldValue=(sample:Sample,field:Field,env:Environment):number=>field==='airspeed'?airspeed(sample,env):sample[field];
function extremaPreserving(samples:Sample[],field:Field,env:Environment,width:number): Sample[] {
  if(samples.length<=width*2)return samples;
  const each=Math.ceil(samples.length/width);
  const out:Sample[]=[samples[0]];
  for(let i=1;i<samples.length-1;i+=each){
    const slice=samples.slice(i,Math.min(i+each,samples.length-1));
    if(!slice.length)continue;
    const lo=slice.reduce((a,b)=>fieldValue(a,field,env)<fieldValue(b,field,env)?a:b);
    const hi=slice.reduce((a,b)=>fieldValue(a,field,env)>fieldValue(b,field,env)?a:b);
    if(lo.time<hi.time)out.push(lo,hi);else if(hi!==lo)out.push(hi,lo);else out.push(lo);
  }
  out.push(samples[samples.length-1]);return out;
}
export default function Plot({samples,field,environment,cursor,onCursor,ceiling}:{
  samples:Sample[];field:Field;environment:Environment;cursor:number;onCursor:(time:number)=>void;ceiling?:number
}){
  const W=680,H=198,L=60,R=15,T=17,B=37;
  const fillId=useId();
  const data=useMemo(()=>extremaPreserving(samples,field,environment,Math.round(W-L-R)),[samples,field,environment]);
  const first=samples[0]?.time??0, last=samples[samples.length-1]?.time??1;
  const duration=Math.max(0.001,last-first);
  const bounds=useMemo(()=>{
    let yMin=0,yMax=ceiling??0;
    for(const s of samples){const v=fieldValue(s,field,environment);yMin=Math.min(yMin,v);yMax=Math.max(yMax,v);}
    const spread=Math.max(0.01,yMax-yMin);
    return {min:yMin-spread*.08,max:yMax+spread*.08,spread};
  },[samples,field,ceiling,environment]);
  const {min,max,spread}=bounds;
  const X=(t:number)=>L+(t-first)/duration*(W-L-R);
  const Y=(v:number)=>T+(max-v)/(max-min)*(H-T-B);
  const path=useMemo(()=>data.map((s,i)=>(i?'L':'M')+X(s.time).toFixed(2)+' '+Y(fieldValue(s,field,environment)).toFixed(2)).join(' '),[data,field,environment,min,max,first,duration]);
  const area=samples.length>0?`${path} L ${X(last)} ${H-B} H ${L} Z`:'';
  const mouse=(event:React.PointerEvent<SVGSVGElement>)=>{
    const transform=event.currentTarget.getScreenCTM();
    if(!transform)return;
    const pointer=new DOMPoint(event.clientX,event.clientY).matrixTransform(transform.inverse());
    const x=pointer.x;
    onCursor(Math.max(first,Math.min(last,first+(x-L)/(W-L-R)*duration)));
  };
  return <div className="plot-box">
    <div className="plot-heading"><strong>{fields[field].label}</strong><span>{fields[field].unit} · planned reference</span></div>
    <svg viewBox={'0 0 '+W+' '+H} role="img" aria-label={fields[field].label+' against time'} onPointerDown={mouse} onPointerMove={e=>{if(e.buttons===1)mouse(e);}}>
      <defs><linearGradient id={fillId} x1="0" x2="0" y1="0" y2="1">
        <stop stopColor="var(--blue)" stopOpacity=".20"/><stop offset="1" stopColor="var(--blue)" stopOpacity="0"/>
      </linearGradient></defs>
      <rect className="plot-grid-frame" x={L} y={T} width={W-L-R} height={H-T-B}/>
      {Array.from({length:5},(_,i)=>{
        const v=min+(max-min)*i/4,y=Y(v);
        return <g key={i}><path className="plot-grid-line" d={'M'+L+' '+y+'H'+(W-R)}/>
        <text x={L-8} y={y+4} fontSize="10" textAnchor="end">{v.toFixed(spread<2?2:1)}</text></g>;
      })}
      {Array.from({length:6},(_,i)=>{
        const t=first+duration*i/5,x=X(t);
        return <g key={i}><path className="plot-grid-line" d={'M'+x+' '+T+'V'+(H-B)}/>
        <text x={x} y={H-16} fontSize="10" textAnchor="middle">{t.toFixed(1)}</text></g>;
      })}
      {ceiling!=null&&ceiling>=min&&ceiling<=max&&
        <g><path d={'M'+L+' '+Y(ceiling)+'H'+(W-R)} stroke="var(--warning)" strokeDasharray="6 3" strokeWidth="1.4"/>
        <text x={W-R-5} y={Y(ceiling)-5} textAnchor="end" fontSize="10" className="plot-bound">bound {ceiling.toFixed(2)}</text></g>}
      {samples.length>0&&<><path d={area} fill={`url(#${fillId})`}/>
      <path d={path} stroke="var(--blue)" fill="none" strokeWidth="2" strokeLinejoin="round"/>
      <path d={'M'+X(cursor)+' '+T+'V'+(H-B)} stroke="#c4d8e4" strokeWidth="1" strokeDasharray="2 4"/></>}
      <text x={W/2} y={H-2} fontSize="11" textAnchor="middle">Mission time (s)</text>
    </svg>
    {samples.length===0&&<div className="plot-empty">Compile a verified plan to populate the plot.</div>}
  </div>;
}
export type { Field };
