import {useEffect,useMemo,useRef,useState} from 'react';
import {readFlight,tracking} from './recordings';
import type {Flight} from './recordings';
import benchmark from '../benchmarks/nanobench.json';
import {exportBlob} from './engineering';
import exampleUrl from '../../validation/2026-10-03/cases/raised-10m-final/telemetry_and_truth.csv.gz?url';

async function readText(blob:Blob,gzip:boolean){
  if(!gzip)return blob.text();
  return new Response(blob.stream().pipeThrough(new DecompressionStream('gzip'))).text();
}
export default function FlightReplay({local}:{local:boolean}){
  const [flight,setFlight]=useState<Flight|null>(null),[title,setTitle]=useState(''),[error,setError]=useState('');
  const [view,setView]=useState<'map'|'height'>('map');
  const [pos,setPos]=useState(0),[play,setPlay]=useState(false);
  const csv=useRef('');
  const input=useRef<HTMLInputElement>(null),generation=useRef(0);
  const stats=useMemo(()=>flight?tracking(flight):null,[flight]);
  async function load(file?:File){
    const id=++generation.current;
    setError('');setPlay(false);setFlight(null);setPos(0);
    try{
      let text:string;
      if(file&&file.size>64*1024*1024)throw Error('Choose a recording under 64 MiB.');
      if(file?.name.toLowerCase().endsWith('.ulg')){
        if(!local)throw Error('Use the localhost studio for binary ULog import, or import CSV here.');
        const res=await fetch('/api/recording',{method:'POST',body:file});
        if(!res.ok)throw Error((await res.json()).error);
        text=await res.text();
      }else if(file){text=await readText(file,file.name.endsWith('.gz'));}
      else{
        const res=await fetch(exampleUrl);
        if(!res.ok)throw Error('Example could not load');
        text=await readText(await res.blob(),true);
      }
      const data=readFlight(text);
      if(id===generation.current){csv.current=text;setFlight(data);setTitle(file?.name??'Retained x500 SITL · raised landing surface');}
    }catch(e){if(id===generation.current)setError((e as Error).message);}
  }
  useEffect(()=>{
    if(!play||!flight)return;
    const frames=flight.frames, start=performance.now(), offset=frames[pos].time;
    let raf=0,index=pos;
    const tick=()=>{
      const now=offset+(performance.now()-start)/1000;
      while(index+1<frames.length&&frames[index+1].time<=now)index++;
      setPos(index);
      if(index===frames.length-1)setPlay(false);else raf=requestAnimationFrame(tick);
    };
    raf=requestAnimationFrame(tick);return()=>cancelAnimationFrame(raf);
  },[play,flight]);
  const chart=useMemo(()=>{
    if(!flight)return null;
    const frames=flight.frames;
    const coordinates=(time:number,v:number[])=>view==='map'?[v[0],v[1]]:[time,flight.axes.startsWith('Vicon')?v[2]:-v[2]];
    let minX=Infinity,minY=Infinity,maxX=-Infinity,maxY=-Infinity;
    for(const f of frames)for(const v of [f.measured,f.reference]){
      if(v){const [x,y]=coordinates(f.time,v);minX=Math.min(minX,x);maxX=Math.max(maxX,x);minY=Math.min(minY,y);maxY=Math.max(maxY,y);}
    }
    const scale=Math.min(600/Math.max(.1,maxX-minX),190/Math.max(.1,maxY-minY));
    const xy=(time:number,v:number[])=>{
      const [x,y]=coordinates(time,v);
      return view==='map'?[320+(x-(maxX+minX)/2)*scale,110-(y-(maxY+minY)/2)*scale]:
        [20+(x-minX)*600/Math.max(.001,maxX-minX),205-(y-minY)*190/Math.max(.1,maxY-minY)];
    };
    const path=(key:'reference'|'measured')=>{
      let d='',move=true;
      for(let i=0;i<frames.length;i++){
        const v=frames[i][key];
        if(!v){move=true;continue;}
        if(i&&frames[i].time-frames[i-1].time>.25)move=true;
        if(move||i%Math.ceil(frames.length/2000)===0||i===frames.length-1){
          d+=(move?'M':'L')+xy(frames[i].time,v).map(n=>n.toFixed(2)).join(' ');move=false;
        }
      }
      return d;
    };
    return {xy,reference:path('reference'),measured:path('measured')};
  },[flight,view]);
  const current=flight?.frames[pos];
  return <section className="data-section"><h3>Recorded flight replay</h3>
    <button onClick={()=>load()}>Load recorded x500 flight</button> <button onClick={()=>input.current?.click()}>Import flight log</button>
    <input type="file" ref={input} hidden accept=".csv,.gz,.ulg" onChange={e=>{const file=e.target.files?.[0];e.target.value='';if(file)load(file);}}/>
    <p className="hint">Replay uses the reference recorded with the flight. Import NanoBench CSV, aligned telemetry CSV, or a PX4 ULog on localhost.</p>
    {error&&<p role="alert">{error}</p>}
    {flight&&stats&&chart&&current&&<><strong>{title}</strong><p>{flight.kind} · {flight.axes}</p>
      {flight.segments.length>1&&<label>Continuous coordinate segment <select value={flight.segment} onChange={e=>{setPlay(false);setPos(0);setFlight(readFlight(csv.current,e.target.value));}}>
        {flight.segments.map(id=><option key={id} value={id}>{id}</option>)}</select></label>}
      <label>Recording view <select value={view} onChange={e=>setView(e.target.value as typeof view)}><option value="map">Horizontal track</option><option value="height">Height vs elapsed time</option></select></label>
      <svg viewBox="0 0 640 220" aria-label={view==='map'?'Horizontal recording and reference':'Recorded height along the timeline'} style={{width:'100%',background:'#10212e'}}>
        <path d={chart.reference} stroke="#ecbc72" fill="none"/><path d={chart.measured} stroke="#58c8da" fill="none"/>
        <circle cx={chart.xy(current.time,current.measured)[0]} cy={chart.xy(current.time,current.measured)[1]} r="4" fill="white"/>
      </svg><small>{view==='map'?(flight.axes.startsWith('Vicon')?'X right / Y up':'North right / East up'):'Seconds right / local height (m) up'}　 Gold · recorded reference　 Cyan · measured track　 White · current sample</small>
      <div><button onClick={()=>{if(pos===flight.frames.length-1)setPos(0);setPlay(!play);}}>{play?'Pause replay':'Play replay'}</button>
      <input aria-label="Recording sample" type="range" min="0" max={flight.frames.length-1} value={pos} onChange={e=>{setPlay(false);setPos(Number(e.target.value));}}/></div>
      <p>{current.time.toFixed(2)} s · {current.phase} · {current.measured.map(v=>v.toFixed(2)).join(' / ')} m</p>
      <p>3D tracking RMS {stats.rms?.toFixed(3)??'—'} m · p95 {stats.p95?.toFixed(3)??'—'} m · peak {stats.max?.toFixed(3)??'—'} m</p>
      <p>{stats.paired} paired / {stats.rows} measured samples · {stats.omitted} missing positions · longest sample gap {stats.maxGap.toFixed(3)} s</p>
      <button onClick={()=>exportBlob('recording-analysis.json',JSON.stringify({title,source:flight.kind,axes:flight.axes,segment:flight.segment,comparison:'recorded reference vs measurement',...stats},null,2),'application/json')}>Export recording analysis</button>
    </>}
    <details><summary>Public hardware benchmark · Crazyflie 2.1</summary>
      <p>Two circle flights set the response time of a first-order position model. These whole flights tested its predictions against Vicon measurements. Each prediction starts from one measured position.</p>
      <table className="point-table"><thead><tr><th>Held-out flight</th><th>Predicted vs measured RMS (m)</th><th>Setpoint vs measured RMS (m)</th></tr></thead><tbody>
      {benchmark.runs.filter(r=>r.split==='held_out').map(r=><tr key={r.flight}><td>{r.flight}</td><td>{r.predicted_minus_measured.rmse_m.toFixed(3)}</td><td>{r.reference_minus_measured.rmse_m.toFixed(3)}</td></tr>)}</tbody></table>
      <button onClick={()=>exportBlob('nanobench-evidence.json',JSON.stringify(benchmark,null,2),'application/json')}>Download benchmark evidence</button>
    </details>
  </section>;
}
