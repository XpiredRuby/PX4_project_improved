import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Activity, ArrowDownToLine, Box as BoxIcon, CheckCircle2, ChevronDown, CircleDot, CloudSun, Crosshair, FolderOpen, Grip, HelpCircle, Layers, Map as MapIcon, MousePointer2, Pause, Play, Plus, RotateCcw, Save, Settings2, ShieldCheck, SlidersHorizontal, Trash2, TriangleAlert, Wind, X } from 'lucide-react';
import type { Environment, Point, Preview, Project, Sample, Tool } from './types';
import { airspeed, buildRequest, exportBlob, getPreviewSummary, initialEnvironment, interpolate, metrics, parseCsv, projectFromJson, sampleProject, validateProject } from './engineering';
import Viewport from './Viewport';
import Plot, { type Field } from './Plot';

const round=(x:number,p=2)=>Number(x.toFixed(p));
const sid=()=>Math.random().toString(36).slice(2,10);
const example=(name:string,alt:number):Point[]=>{
  const coords: Record<string,number[][]>={
    Straight:[[0,0],[10,0],[20,0],[30,0]],
    L:[[0,0],[12,0],[12,12],[30,12]],
    Survey:[[0,0],[0,18],[8,18],[8,0],[16,0],[16,18],[24,18]],
    Circle:Array.from({length:25},(_,i)=>[15+12*Math.sin(i*Math.PI*2/24),12-12*Math.cos(i*Math.PI*2/24)])
  };
  return (coords[name]||coords.Straight).map(([north,east],i,arr)=>({id:'P'+String(i+1).padStart(2,'0'),north,east,altitude:alt,
    hard:i===0||i===arr.length-1,exact:i===0||i===arr.length-1}));
};
const downloadJson=(filename:string,data:unknown)=>exportBlob(filename,JSON.stringify(data,null,2)+'\n','application/json');
const display=(n:number,unit:string,digits=2)=>Number.isFinite(n)?n.toFixed(digits)+' '+unit:'—';
const asNum=(value:string,current:number)=>value.trim()===''?current:Number(value);

function ErrorPanel({message,onClose}:{message:string;onClose:()=>void}){
  return <div className="notice error" role="alert"><TriangleAlert size={16}/><span>{message}</span><button aria-label="Dismiss error" onClick={onClose}><X size={16}/></button></div>;
}
function TopMap({points,selected,onSelect,onChange,tool,onAdd,onDraw,onFinishDraw,range,setRange}:{
  points:Point[];selected:number;onSelect:(index:number)=>void;onChange:(index:number,point:Point)=>void;
  tool:Tool;onAdd:(north:number,east:number)=>void;onDraw:(north:number,east:number,first:boolean)=>void;
  onFinishDraw:()=>void;range:number;setRange:(v:number)=>void;
}){
  const ref=useRef<SVGSVGElement>(null);
  const dragging=useRef<number|null>(null);
  const drawing=useRef(false);
  const panning=useRef<{x:number;y:number}|null>(null);
  const [center,setCenter]=useState({north:0,east:0});
  const view=range*2;
  // Maintain an equal physical map scale on rectangular and square viewports.
  const [aspect,setAspect]=useState(1.65);
  useEffect(()=>{if(!ref.current)return;const obs=new ResizeObserver(es=>{const e=es[0];if(e.contentRect.height>0)setAspect(e.contentRect.width/e.contentRect.height);});obs.observe(ref.current);return()=>obs.disconnect();},[]);
  const visibleHeight=view/aspect;
  const pointerToNE=(event:React.PointerEvent<SVGSVGElement>)=>{
    const rect=event.currentTarget.getBoundingClientRect();
    return {east:center.east+((event.clientX-rect.left)/rect.width-.5)*view,
      north:center.north-((event.clientY-rect.top)/rect.height-.5)*visibleHeight};
  };
  const path=points.map((p,i)=>(i?'L':'M')+p.east+' '+(-p.north)).join(' ');
  const onDown=(e:React.PointerEvent<SVGSVGElement>)=>{
    const p=pointerToNE(e);
    if(tool==='waypoint')onAdd(round(p.north),round(p.east));
    if(tool==='draw'){drawing.current=true;onDraw(round(p.north),round(p.east),true);}
    if(tool==='pan')panning.current={x:e.clientX,y:e.clientY};
    if(tool==='select'&&dragging.current===null)onSelect(-1);
    e.currentTarget.setPointerCapture(e.pointerId);
  };
  const onMove=(e:React.PointerEvent<SVGSVGElement>)=>{
    if(dragging.current!==null&&tool==='select'){
      const p=pointerToNE(e),current=points[dragging.current];
      if(current)onChange(dragging.current,{...current,north:round(p.north),east:round(p.east)});
    }
    if(drawing.current&&tool==='draw'){
      const p=pointerToNE(e);onDraw(round(p.north),round(p.east),false);
    }
    if(panning.current&&tool==='pan'){
      const dx=e.clientX-panning.current.x,dy=e.clientY-panning.current.y;
      panning.current={x:e.clientX,y:e.clientY};
      setCenter(c=>({east:c.east-dx*view/e.currentTarget.getBoundingClientRect().width,
        north:c.north+dy*visibleHeight/e.currentTarget.getBoundingClientRect().height}));
    }
  };
  const onUp=()=>{dragging.current=null;panning.current=null;if(drawing.current)onFinishDraw();drawing.current=false;};
  return <div className="map-frame">
    <div className="map-controls"><span>TOP / N ↑ / E →</span><div>
      <button onClick={()=>setRange(Math.max(12,range/1.25))} title="Zoom in">+</button>
      <button onClick={()=>setRange(Math.min(120,range*1.25))} title="Zoom out">−</button>
      <button onClick={()=>{setRange(45);setCenter({north:0,east:0});}} title="Reset view"><Crosshair size={13}/></button>
    </div></div>
    <svg ref={ref} className={'map-svg tool-'+tool} viewBox={[center.east-range,-center.north-visibleHeight/2,view,visibleHeight].join(' ')}
      onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp} onPointerCancel={onUp} aria-label="Mission coordinate drawing canvas">
      <rect x={center.east-range} y={-center.north-visibleHeight/2} width={view} height={visibleHeight} fill="#10212e"/>
      {Array.from({length:17},(_,i)=>{
        const v=(i-8)*10;
        return <g key={i}>
          <path d={'M'+v+' '+(-center.north-visibleHeight/2)+'v'+visibleHeight} stroke={v===0?'#466879':'#294052'} strokeWidth={v===0?.28:.16}/>
          <path d={'M'+(center.east-range)+' '+(-v)+'h'+view} stroke={v===0?'#466879':'#294052'} strokeWidth={v===0?.28:.16}/>
          <text x={v+0.6} y={-center.north-visibleHeight/2+2} fill="#7290a5" fontSize="1.7">{v}</text>
        </g>;
      })}
      <circle cx="0" cy="0" r="40" fill="none" stroke="#bca068" strokeWidth=".2" strokeDasharray="1 1.5"/>
      <path d={path} stroke="#56bcd0" strokeWidth=".65" fill="none" strokeLinejoin="round" strokeLinecap="round" vectorEffect="non-scaling-stroke"/>
      {points.filter((_,i)=>points.length<80||i%Math.ceil(points.length/60)===0||i===points.length-1).map(p=>{
        const index=points.indexOf(p);
        return <g key={p.id+'-'+index} onPointerDown={e=>{if(tool!=='select')return;e.stopPropagation();dragging.current=index;onSelect(index);e.currentTarget.closest('svg')?.setPointerCapture(e.pointerId);}}>
          <circle cx={p.east} cy={-p.north} r={selected===index?.95:.65} stroke={selected===index?'#ffffff':p.hard?'#f2b96f':'#9ce0ef'}
            fill={selected===index?'#2198b4':'#162d3b'} strokeWidth=".22" vectorEffect="non-scaling-stroke"/>
          {(p.hard||selected===index||index===0||index===points.length-1)&&
            <text x={p.east+1.0} y={-p.north-1} fontSize="2.1" fill="#cfe0ec" pointerEvents="none">{p.id}</text>}
        </g>;
      })}
    </svg>
    <div className="viewport-status"><span>{points.length.toLocaleString()} input points</span><span>XY · local NED · meters</span><span>Reference radius 40 m</span></div>
  </div>;
}

export default function App(){
  const [project,setProject]=useState<Project>(()=>({...sampleProject,environment:{...initialEnvironment},points:sampleProject.points.map(p=>({...p}))}));
  const [tool,setTool]=useState<Tool>('select');
  const [selected,setSelected]=useState(1);
  const [range,setRange]=useState(45);
  const [active,setActive]=useState<'geometry'|'trajectory'|'analysis'>('geometry');
  const [field,setField]=useState<Field>('speed');
  const [environmentTab,setEnvironmentTab]=useState(true);
  const [advanced,setAdvanced]=useState(false);
  const [busy,setBusy]=useState(false);
  const [error,setError]=useState('');
  const [notice,setNotice]=useState('');
  const [result,setResult]=useState<Preview|null>(null);
  const [samples,setSamples]=useState<Sample[]>([]);
  const [cursor,setCursor]=useState(0);
  const [playing,setPlaying]=useState(false);
  const [engine,setEngine]=useState<'unknown'|'browser'|'local'>('unknown');
  const [weatherBusy,setWeatherBusy]=useState(false);
  const [weatherError,setWeatherError]=useState('');
  const [placeQuery,setPlaceQuery]=useState('');
  const [placeMatches,setPlaceMatches]=useState<Array<{id:number;name:string;admin1?:string;country?:string;latitude:number;longitude:number}>>([]);
  const [searchingPlace,setSearchingPlace]=useState(false);
  const [dirty,setDirty]=useState(false);
  const abort=useRef<AbortController|null>(null);
  const fileRef=useRef<HTMLInputElement>(null);
  const pointsCsvRef=useRef<HTMLInputElement>(null);
  const frame=useRef<number>(0);
  const clock=useRef<number>(0);
  const lastDraw=useRef<Point|null>(null);
  const geometryRevision=useRef(0);
  const history=useRef<Project[]>([]);

  useEffect(()=>{fetch('/api/capabilities').then(r=>r.json()).then(d=>setEngine(d.execution_location==='browser'?'browser':'local')).catch(()=>setEngine('unknown'));},[]);
  useEffect(()=>()=>{abort.current?.abort();cancelAnimationFrame(frame.current);},[]);
  const update=useCallback((f:(p:Project)=>Project)=>{
    geometryRevision.current++;
    abort.current?.abort();
    setProject(prev=>{const next=f(prev);if(next!==prev){history.current=[...history.current.slice(-19),prev];}return next;});
    setResult(null);setSamples([]);setCursor(0);setPlaying(false);setDirty(true);setError('');setNotice('');
  },[]);
  const changeValue=(key:keyof Project,value:number|string)=>{
    update(p=>({...p,[key]:value}));
  };
  const changeEnv=(patch:Partial<Environment>)=>{
    const stale=project.environment.mode==='live' && ['latitude','longitude','timestamp'].some(k=>Object.prototype.hasOwnProperty.call(patch,k));
    setProject(p=>({...p,environment:{...p.environment,...(stale?{windSpeed:0,windDirection:0,gust:0,source:'Awaiting new location/time forecast',fetchedAt:undefined}:{}),...patch}}));
    setDirty(true);
    if(patch.mode!==undefined)setWeatherError('');
  };
  const editPoint=(index:number,point:Point)=>update(p=>({...p,points:p.points.map((v,i)=>i===index?point:v)}));
  const newPoint=(north:number,east:number)=>{
    update(p=>({...p,points:[...p.points,{id:'P'+String(p.points.length+1).padStart(2,'0'),north,east,altitude:p.altitude,hard:false,exact:false}]}));
    setSelected(project.points.length);
  };
  const drawPoint=(north:number,east:number,first:boolean)=>{
    if(first)history.current=[...history.current.slice(-19),project];
    const previous=lastDraw.current;
    if(!first&&previous&&Math.hypot(previous.north-north,previous.east-east)<.6)return;
    const p:Point={id:'D'+sid(),north,east,altitude:project.altitude,hard:false,exact:false};
    lastDraw.current=p;
    geometryRevision.current++;
    abort.current?.abort();
    setProject(old=>old.points.length>=8192?old:{...old,points:[...old.points,p]});
    setDirty(true);setResult(null);setSamples([]);setPlaying(false);setCursor(0);
  };
  const finishDraw=()=>{lastDraw.current=null;setNotice('Sketch added. Run verification; the planner may adjust this route to satisfy constraints.');};
  const clearRoute=()=>{update(p=>({...p,title:'New mission',points:[]}));setTool('draw');setSelected(-1);};
  const undo=()=>{const previous=history.current.pop();if(previous){geometryRevision.current++;abort.current?.abort();setProject(previous);setResult(null);setSamples([]);setPlaying(false);setDirty(true);}};
  const applyExample=(name:string)=>{update(p=>({...p,title:name+' study',points:example(name,p.altitude)}));setSelected(-1);};
  const selectedPoint=project.points[selected];
  const stats=useMemo(()=>metrics(samples,project.environment),[samples,project.environment]);
  const current=useMemo(()=>interpolate(samples,cursor),[samples,cursor]);
  const duration=stats?.duration||0;
  const cap=Math.min(project.speedCap,3);

  useEffect(()=>{
    if(!playing||samples.length<2)return;
    clock.current=performance.now();
    const tick=(now:number)=>{
      const dt=Math.min(.1,Math.max(0,(now-clock.current)/1000));clock.current=now;
      setCursor(old=>{const next=old+dt;if(next>=samples[samples.length-1].time){setPlaying(false);return samples[samples.length-1].time;}return next;});
      frame.current=requestAnimationFrame(tick);
    };
    frame.current=requestAnimationFrame(tick);
    return()=>cancelAnimationFrame(frame.current);
  },[playing,samples]);

  const run=async()=>{
    setError('');setNotice('');setPlaying(false);
    try{
      const request=buildRequest(project);
      const submittedRevision=geometryRevision.current;
      const controller=new AbortController();abort.current=controller;
      setBusy(true);
      const response=await fetch('/api/preview',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(request),signal:controller.signal});
      const data=await response.json() as Preview;
      if(!data.accepted||!response.ok)throw Error(data.error||'Planner rejected the requested route');
      if(!data.trajectory_csv)throw Error('Planner accepted but returned no reference CSV');
      const parsed=parseCsv(data.trajectory_csv);
      if(submittedRevision!==geometryRevision.current){setNotice('Edited during compilation; discarded the obsolete result. Run verification again.');return;}
      if(!parsed.length)throw Error('Verified plan returned no usable samples');
      setResult(data);setSamples(parsed);setCursor(parsed[0].time);setActive('analysis');setDirty(false);
      setNotice('Verified mathematical reference received. This is not PX4/Gazebo or flight telemetry.');
    }catch(e){if((e as Error).name!=='AbortError')setError((e as Error).message);else setNotice('Planning cancelled or invalidated by an edit. Your mission was kept.');}
    finally{setBusy(false);abort.current=null;}
  };
  const fetchWeather=async()=>{
    setWeatherBusy(true);setWeatherError('');
    try{
      const env=project.environment;
      if(!Number.isFinite(env.latitude)||!Number.isFinite(env.longitude)||Math.abs(env.latitude)>90||Math.abs(env.longitude)>180)
        throw Error('Enter valid latitude and longitude');
      const q=new URLSearchParams({latitude:String(env.latitude),longitude:String(env.longitude),timezone:'GMT',wind_speed_unit:'ms',
        hourly:'wind_speed_10m,wind_direction_10m,wind_gusts_10m,temperature_2m,surface_pressure',forecast_days:'7'});
      const response=await fetch('https://api.open-meteo.com/v1/forecast?'+q.toString());
      if(!response.ok)throw Error('Forecast service returned '+response.status);
      const json=await response.json();
      if(json.error)throw Error(String(json.reason||'Forecast unavailable'));
      const target=new Date(env.timestamp).getTime();
      if(!Number.isFinite(target))throw Error('Choose a valid forecast time');
      const times:string[]=json.hourly?.time||[];
      let i=-1,best=Infinity;
      times.forEach((s,j)=>{const t=Date.parse(s+'Z'),d=Math.abs(t-target);if(d<best){i=j;best=d;}});
      if(i<0||best>3600000)throw Error('Selected time is outside the available seven-day forecast');
      const h=json.hourly;
      const vals=['wind_speed_10m','wind_direction_10m','wind_gusts_10m','temperature_2m','surface_pressure'];
      if(vals.some(k=>!Number.isFinite(h[k]?.[i])))throw Error('Forecast contains missing required weather values');
      changeEnv({windSpeed:h.wind_speed_10m[i],windDirection:h.wind_direction_10m[i],gust:h.wind_gusts_10m[i],
        temperature:h.temperature_2m[i],pressure:h.surface_pressure[i],
        timestamp:new Date(Date.parse(times[i]+'Z')).toISOString(),fetchedAt:new Date().toISOString(),
        source:'Open-Meteo forecast · 10 m modeled wind · hourly UTC snapshot'});
    }catch(e){setWeatherError((e as Error).message);}finally{setWeatherBusy(false);}
  };
  const searchPlace=async()=>{
    setSearchingPlace(true);setWeatherError('');setPlaceMatches([]);
    try{
      if(placeQuery.trim().length<3||placeQuery.length>100)throw Error('Enter a city or place name with at least 3 characters');
      const query=new URLSearchParams({name:placeQuery.trim(),count:'5',format:'json',language:'en'});
      const response=await fetch('https://geocoding-api.open-meteo.com/v1/search?'+query.toString());
      if(!response.ok)throw Error('Location search unavailable: '+response.status);
      const json=await response.json();
      if(json.error)throw Error(String(json.reason||'Location search failed'));
      const results=(json.results||[]) as Array<{id:number;name:string;admin1?:string;country?:string;latitude:number;longitude:number}>;
      setPlaceMatches(results.filter(p=>Number.isFinite(p.latitude)&&Number.isFinite(p.longitude)));
      if(!results.length)setWeatherError('No location matches. Enter latitude and longitude directly.');
    }catch(e){setWeatherError((e as Error).message);}finally{setSearchingPlace(false);}
  };
  const importWaypointCsv=async(event:React.ChangeEvent<HTMLInputElement>)=>{
    const file=event.target.files?.[0];event.target.value='';if(!file)return;
    try{
      if(file.size>2_000_000)throw Error('Waypoint CSV exceeds 2 MB');
      const rows=(await file.text()).trim().split(/\r?\n/);
      const headings=rows.shift()?.split(',').map(s=>s.trim().toLowerCase())||[];
      const keys=['north_m','east_m','altitude_m'];
      if(keys.some(k=>!headings.includes(k)))throw Error('Waypoint CSV requires north_m,east_m,altitude_m headers');
      if(rows.length<2||rows.length>8192)throw Error('Waypoint CSV requires 2–8,192 rows');
      const imported=rows.map((line,i)=>{
        const cells=line.split(',').map(s=>s.trim());
        if(cells.length!==headings.length)throw Error('Invalid CSV column count at row '+(i+2));
        const read=(k:string)=>cells[headings.indexOf(k)];
        const number=(k:string)=>{const raw=read(k);if(raw==='')throw Error('Empty '+k+' at row '+(i+2));const n=Number(raw);if(!Number.isFinite(n))throw Error('Invalid '+k+' at row '+(i+2));return n;};
        const safeId=(read('point_id')||'P'+String(i+1).padStart(2,'0')).replace(/[^a-zA-Z0-9_-]/g,'_').slice(0,50);
        return {id:safeId,north:number('north_m'),east:number('east_m'),altitude:number('altitude_m'),
          hard:read('hard')==='true'||read('hard')==='1'||i===0||i===rows.length-1,
          exact:read('exact')==='true'||read('exact')==='1'||i===0||i===rows.length-1} satisfies Point;
      });
      const identifiers=new Set(imported.map(p=>p.id));
      if(identifiers.size!==imported.length)throw Error('CSV point identifiers must be unique');
      const next={...project,title:file.name.replace(/\.csv$/i,''),points:imported};
      validateProject(next);
      update(()=>next);setSelected(-1);
      setNotice('Imported '+imported.length+' waypoint rows. Re-run verification to generate a reference.');
    }catch(e){setError((e as Error).message);}
  };
  const exportWaypointCsv=()=>{
    const lines=['point_id,north_m,east_m,altitude_m,hard,exact',
      ...project.points.map(p=>[p.id,p.north,p.east,p.altitude,String(p.hard),String(p.exact)].join(','))];
    exportBlob('mission-waypoints.csv',lines.join('\n')+'\n','text/csv');
  };
  const importProject=async(event:React.ChangeEvent<HTMLInputElement>)=>{
    const file=event.target.files?.[0];event.target.value='';if(!file)return;
    try{if(file.size>2_000_000)throw Error('File exceeds 2 MB limit');
      const imported=projectFromJson(JSON.parse(await file.text()));
      update(()=>imported);setSelected(-1);
    }catch(e){setError((e as Error).message);}
  };
  const saveProject=()=>downloadJson(project.title.replace(/[^a-z0-9_-]/gi,'_')+'.px4-project.json',project);
  const saveReference=()=>{if(result?.trajectory_csv)exportBlob('verified-route.csv',result.trajectory_csv,'text/csv');};
  const summary=getPreviewSummary(result||{accepted:false});
  return <div className="app-shell">
    <header className="topbar">
      <div className="brand"><div className="brand-mark"><Activity size={19}/></div>
        <div><strong>PX4 <span>TRAJECTORY STUDIO</span></strong><small>ENGINEERING WORKSPACE / 2.0 PREVIEW</small></div></div>
      <div className="top-center"><span className="project-dot"/>{project.title}{dirty&&<span className="changed-dot" title="Unverified changes">●</span>}</div>
      <div className="top-actions"><span className="status-tag"><ShieldCheck size={13}/> SITL ONLY</span>
        <button className="ghost icon-button" title="Open project" onClick={()=>fileRef.current?.click()}><FolderOpen size={17}/></button>
        <button className="ghost icon-button" title="Save editable project" onClick={saveProject}><Save size={17}/></button>
        <input ref={fileRef} hidden type="file" accept=".json" onChange={importProject}/>
        <input ref={pointsCsvRef} hidden type="file" accept=".csv,text/csv" onChange={importWaypointCsv}/></div>
    </header>
    <nav className="menubar" aria-label="Workspace sections">
      <div className="menu-buttons">
        {(['geometry','trajectory','analysis'] as const).map(t=><button key={t} className={active===t?'nav-active':''} onClick={()=>setActive(t)}>
          {t==='geometry'?<MapIcon size={15}/>:t==='trajectory'?<Layers size={15}/>:<Activity size={15}/>}
          {t==='geometry'?'Geometry':t==='trajectory'?'Trajectory':'Results & Analysis'}</button>)}
      </div>
      <div className="menu-right"><span><span className="live-dot"/> {engine==='browser'?'Browser verified planner':engine==='local'?'Local planner bridge':'Planner connection pending'}</span>
        <span className="menu-divider"/><span>LOCAL NED · SI UNITS</span></div>
    </nav>
    <div className="workbench">
      <aside className="left-rail">
        <div className="pane-title"><Layers size={14}/> MODEL EXPLORER</div>
        <div className="tree">
          <div className="tree-root"><ChevronDown size={14}/><FolderOpen size={15}/> {project.title}</div>
          <div className="tree-child"><CircleDot size={14}/> Geometry <span>{project.points.length}</span></div>
          <div className="tree-sub">Waypoints / path strokes</div>
          <button className="tree-child tree-button" onClick={()=>setEnvironmentTab(false)}><BoxIcon size={14}/> Aircraft <span>x500</span></button>
          <button className="tree-child tree-button" onClick={()=>setEnvironmentTab(true)}><CloudSun size={14}/> Environment <span>{project.environment.mode}</span></button>
          <div className="tree-child"><Activity size={14}/> Reference <span>{result?.accepted?'Verified':'Not verified'}</span></div>
          <div className="tree-sub">PX4 / Gazebo <span className="pending">Milestone 3</span></div>
        </div>
        <div className="pane-title second-title"><SlidersHorizontal size={14}/> OPERATIONS</div>
        <div className="operations">
          <button onClick={clearRoute}>+ New blank mission</button>
          <button onClick={()=>pointsCsvRef.current?.click()}>Import waypoint CSV</button>
          <button onClick={exportWaypointCsv}>Export waypoint CSV</button>
          <button onClick={()=>applyExample('Straight')}>Straight path</button>
          <button onClick={()=>applyExample('L')}>L-shaped route</button>
          <button onClick={()=>applyExample('Survey')}>Survey route</button>
          <button onClick={()=>applyExample('Circle')}>Closed-looking loop</button>
        </div>
        <div className="left-footer"><HelpCircle size={15}/><span>Draw or edit route points, then run the verified numerical planner. The shape may be adjusted to obey aircraft limits.</span></div>
      </aside>
      <main className="main-pane">
        <div className="workspace-header"><div><div className="eyebrow">WORKSPACE / {active.toUpperCase()}</div>
          <h1>{active==='geometry'?'Mission geometry':active==='trajectory'?'Trajectory inspection':'Engineering post-processing'}</h1>
          <p>{active==='geometry'?'Define flight geometry through direct manipulation or precise coordinates.'
            :active==='trajectory'?'Review the original request, verified reference and moving vehicle marker.'
            :'Inspect time-synchronized numerical results and environmental sensitivity.'}</p></div>
          <div className="workspace-actions"><button className="ghost" onClick={undo} disabled={history.current.length===0}><RotateCcw size={15}/> Undo</button>
            <button className="primary" onClick={run} disabled={busy||project.points.length<2}>
              {busy?<span className="spinner"/>:<ShieldCheck size={16}/>} {busy?'Verifying…':'Plan & verify'}</button>
            {busy&&<button className="ghost" onClick={()=>abort.current?.abort()}>Cancel</button>}
          </div></div>
        {error&&<ErrorPanel message={error} onClose={()=>setError('')}/>}
        {notice&&<div className="notice info"><CheckCircle2 size={16}/><span>{notice}</span><button onClick={()=>setNotice('')} aria-label="Dismiss notification"><X size={15}/></button></div>}
        <div className="toolstrip">
          <div className="tool-group">
            {([{id:'select',icon:MousePointer2,label:'Select / move'},{id:'waypoint',icon:Plus,label:'Add point'},{id:'draw',icon:Grip,label:'Freehand'},{id:'pan',icon:Crosshair,label:'Pan'}] as const).map(t=>
              <button className={tool===t.id?'selected-tool':''} key={t.id} onClick={()=>setTool(t.id)} title={t.label}><t.icon size={16}/><span>{t.label}</span></button>)}
          </div>
          <div className="tool-strip-info"><span className="dot-cyan"/> Original geometry <span className="dot-gold"/> Hard stops</div>
        </div>
        <div className="viewport-panel">
          {active==='geometry'?
            <TopMap points={project.points} selected={selected} onSelect={setSelected} onChange={editPoint} tool={tool}
              onAdd={newPoint} onDraw={drawPoint} onFinishDraw={finishDraw} range={range} setRange={setRange}/>
          : <Viewport points={project.points} samples={samples} cursor={cursor} field={field}/>}
        </div>
        {active==='geometry'?<section className="data-section">
          <div className="section-heading"><div><strong>Waypoint definition</strong><span>Double-check positions and required stops</span></div>
            <div className="section-actions"><button onClick={()=>newPoint(0,0)}><Plus size={14}/> Add row</button>
              <button disabled={selected<0||project.points.length<=2} onClick={()=>{update(p=>({...p,points:p.points.filter((_,i)=>i!==selected)}));setSelected(-1);}}><Trash2 size={14}/> Remove selected</button></div></div>
          <div className="table-scroll"><table className="point-table"><thead><tr><th>POINT</th><th>NORTH (m)</th><th>EAST (m)</th><th>ALT. (m)</th><th>STOP</th><th>EXACT</th></tr></thead>
          <tbody>{project.points.slice(0,250).map((p,i)=><tr key={p.id+'-'+i} className={selected===i?'row-selected':''} onClick={()=>setSelected(i)}>
            <td><span className="point-number">{String(i+1).padStart(2,'0')}</span> {p.id}</td>
            {(['north','east','altitude'] as const).map(k=><td key={k}><input aria-label={p.id+' '+k} type="number" step=".1" value={p[k]} onChange={e=>editPoint(i,{...p,[k]:asNum(e.target.value,p[k])})}/></td>)}
            <td><input aria-label={p.id+' hard stop'} type="checkbox" checked={p.hard||i===0||i===project.points.length-1} disabled={i===0||i===project.points.length-1} onChange={e=>editPoint(i,{...p,hard:e.target.checked})}/></td>
            <td><input aria-label={p.id+' exact point'} type="checkbox" checked={p.exact||i===0||i===project.points.length-1} disabled={i===0||i===project.points.length-1} onChange={e=>editPoint(i,{...p,exact:e.target.checked})}/></td>
          </tr>)}</tbody></table></div>
          {project.points.length>250&&<p className="small-note">Displaying the first 250 of {project.points.length} sketch samples. The complete drawing is preserved and submitted to the planner.</p>}
        </section>:
        <section className="data-section">
          <div className="section-heading"><div><strong>Reference timeline</strong><span>Play, scrub, and inspect the verified CSV at original time coordinates.</span></div>
          <div className="timeline-buttons"><button aria-label="Play or pause" onClick={()=>{if(cursor>= (samples[samples.length-1]?.time||0))setCursor(samples[0]?.time||0);setPlaying(!playing);}} disabled={!samples.length}>{playing?<Pause size={16}/>:<Play size={16}/>}</button>
            <button onClick={()=>setCursor(samples[0]?.time||0)} disabled={!samples.length} title="Reset time"><RotateCcw size={15}/></button></div></div>
          <div className="scrubber"><input aria-label="Reference time" type="range" min={samples[0]?.time||0} max={samples[samples.length-1]?.time||1} step=".01" value={cursor} disabled={!samples.length} onChange={e=>{setPlaying(false);setCursor(Number(e.target.value));}}/>
            <span>{display(cursor,'s')} / {display(duration,'s')}</span></div>
          {active==='analysis'&&<><div className="field-select">
              <label>Scalar</label><select value={field} onChange={e=>setField(e.target.value as Field)}>
                <option value="speed">Groundspeed</option><option value="acceleration">Acceleration</option><option value="jerk">Jerk</option>
                <option value="altitude">Altitude</option><option value="yaw">Yaw</option></select>
              <span>Computed field, not an FEA stress solution</span></div>
            <div className="plot-grid">{(['speed','acceleration','jerk','altitude','yaw'] as const).map(metric=>
              <Plot key={metric} samples={samples} field={metric} cursor={cursor} onCursor={t=>{setPlaying(false);setCursor(t);}}
                ceiling={metric==='speed'?cap:metric==='acceleration'?1:metric==='jerk'?1.5:undefined}/>)}</div>
          </>}
          {current&&<div className="sample-readout">
            <div><span>TIME</span><strong>{display(current.time,'s')}</strong></div>
            <div><span>N / E</span><strong>{round(current.north)} / {round(current.east)} m</strong></div>
            <div><span>HEIGHT</span><strong>{display(current.altitude,'m')}</strong></div>
            <div><span>GROUND SPEED</span><strong>{display(current.speed,'m/s')}</strong></div>
            <div><span>WIND-RELATIVE</span><strong>{display(airspeed(current,project.environment),'m/s')}</strong></div>
          </div>}
        </section>}
        {stats&&<section className="metrics-row">
          <div><span>REFERENCE DURATION</span><strong>{display(stats.duration,'s',1)}</strong></div>
          <div><span>PATH DISTANCE</span><strong>{display(stats.distance,'m',1)}</strong></div>
          <div><span>PEAK GROUND SPEED</span><strong>{display(stats.peak,'m/s')}</strong></div>
          <div><span>PEAK AIR-RELATIVE</span><strong>{display(stats.air,'m/s')}</strong></div>
          <div><span>PEAK ACCEL.</span><strong>{display(stats.accel,'m/s²')}</strong></div>
          <div><span>PEAK JERK</span><strong>{display(stats.jerk,'m/s³')}</strong></div>
        </section>}
        {result&&<section className="data-section">
          <div className="section-heading"><div><strong>Verification & artifact exports</strong><span>Mathematical reference — not simulated or physically measured flight</span></div>
            <div className="section-actions"><button onClick={saveReference}><ArrowDownToLine size={14}/> CSV</button>
              <button onClick={()=>downloadJson('planner-evidence.json',{result:{...result,trajectory_csv:undefined},environment:project.environment,scope:'planned reference; environment used only for air-relative analysis'})}><ArrowDownToLine size={14}/> Report JSON</button></div></div>
          <div className="verification-grid">
            <div><span>RESULT</span><strong className="pass-label">ACCEPTED REFERENCE</strong></div>
            <div><span>RELEASE CLASS</span><strong>{result.release_class||'SITL_ONLY'}</strong></div>
            <div><span>MODEL CAP</span><strong>3.00 m/s</strong></div>
            <div><span>REQUESTED NEVER-EXCEED</span><strong>{display(project.speedCap,'m/s')}</strong></div>
            <div><span>PLANNED CAP</span><strong>{String(summary.speed.planning_cap_m_s??'—')} m/s</strong></div>
            <div><span>GAZEBO CAMPAIGN</span><strong>NOT RUN</strong></div>
          </div>
          {result.planning_adjustments&&<div className="adjustments"><strong>Requested vs accepted trajectory</strong>
            <p>Original drawing tolerance: {display(result.planning_adjustments.requested_rounding_m??0,'m')}. Maximum documented requested-path bound: {display(result.planning_adjustments.curve_error_bound_from_requested_m??0,'m')}. Fidelity criterion: {result.planning_adjustments.original_rounding_met?'Met':'Not met; modified route requires review'}.</p>
            <ul>{(result.planning_adjustments.adjustments||[]).map((a,i)=><li key={i}><strong>{a.code}</strong> — {a.message}</li>)}</ul>
            {!result.planning_adjustments.adjusted&&<p>No requested-route adjustment was reported by the planner.</p>}
          </div>}
          <details><summary>Raw verification evidence and adjustment report</summary><pre>{JSON.stringify({verification:summary.certificate,adjustments:result.planning_adjustments,interpretation:result.interpretation},null,2)}</pre></details>
        </section>}
      </main>
      <aside className="inspector">
        <div className="pane-title"><Settings2 size={14}/> PROPERTIES</div>
        <div className="inspector-content">
          <div className="inspector-section">
            <div className="group-title">VEHICLE MODEL <ChevronDown size={14}/></div>
            <label>Aircraft preset</label><select aria-label="Aircraft model" value="x500" disabled><option value="x500">PX4 x500 / multirotor</option></select>
            <div className="hint">Simplified SITL model. Verified hard speed limit: 3.00 m/s. Other profiles require qualified model binding.</div>
            <div className="form-row"><label>Reference altitude (m)<input type="number" min=".1" max="30" step=".5" value={project.altitude}
              onChange={e=>{const value=asNum(e.target.value,project.altitude);update(p=>({...p,altitude:value,points:p.points.map(q=>({...q,altitude:value}))}));}}/></label></div>
          </div>
          <div className="inspector-section">
            <div className="group-title">TRAJECTORY SETTINGS <ChevronDown size={14}/></div>
            <div className="form-row"><label>Preferred cruise (m/s)<input type="number" min=".01" max="20" step=".1" value={project.cruise} onChange={e=>changeValue('cruise',asNum(e.target.value,project.cruise))}/></label>
              <label>Never exceed (m/s)<input type="number" min=".01" max="20" step=".1" value={project.speedCap} onChange={e=>changeValue('speedCap',asNum(e.target.value,project.speedCap))}/></label></div>
            {project.speedCap>3&&<div className="hint warn">Research request above x500's 3 m/s model limit. The planner will enforce its actual vehicle limit and disclose adjustments. No 20 m/s flight capability is implied.</div>}
            <label>Maximum geometric deviation (m)<input type="number" step=".05" min=".01" max="10" value={project.tolerance} onChange={e=>changeValue('tolerance',asNum(e.target.value,project.tolerance))}/></label>
            <label>Fixed yaw (degrees)<input type="number" step="5" value={project.yaw} onChange={e=>changeValue('yaw',asNum(e.target.value,project.yaw))}/></label>
            <button className="advanced-toggle" onClick={()=>setAdvanced(!advanced)}><ChevronDown size={13}/> {advanced?'Hide':'Show'} numerical context</button>
            {advanced&&<div className="hint">Planning uses the existing Python path geometry, constraint retiming, vehicle envelope and verification routines. Cruise is a preference, not an achieved-speed guarantee. Altitudes are launch-relative; positions are local NED.</div>}
          </div>
          <div className="inspector-section">
            <button className="group-title group-click" aria-expanded={environmentTab} onClick={()=>setEnvironmentTab(!environmentTab)}><Wind size={15}/> ENVIRONMENT <ChevronDown size={14}/></button>
            {environmentTab&&<><div className="mode-switch">
              <button className={project.environment.mode==='manual'?'mode-active':''} onClick={()=>changeEnv({mode:'manual',source:'User-specified manual scenario',fetchedAt:undefined})}>Manual</button>
              <button className={project.environment.mode==='live'?'mode-active':''} onClick={()=>changeEnv({mode:'live',source:'Awaiting forecast; last values are not current',fetchedAt:undefined})}>Live forecast</button>
            </div>
            <label>Search city or place name</label>
            <div className="place-search"><input aria-label="Search location" value={placeQuery} placeholder="e.g. College Station, Texas" onChange={e=>setPlaceQuery(e.target.value)} onKeyDown={e=>{if(e.key==='Enter')searchPlace();}}/><button disabled={searchingPlace} onClick={searchPlace}>{searchingPlace?'...':'Find'}</button></div>
            {placeMatches.length>0&&<div className="place-matches">{placeMatches.map(place=><button key={place.id} onClick={()=>{changeEnv({latitude:place.latitude,longitude:place.longitude});setPlaceMatches([]);setPlaceQuery([place.name,place.admin1,place.country].filter(Boolean).join(', '));}}>{[place.name,place.admin1,place.country].filter(Boolean).join(', ')}</button>)}</div>}
            <div className="hint">Search data: GeoNames via Open-Meteo. Choose a result to update coordinates.</div>
            <div className="form-row">
              <label>Latitude (°)<input type="number" step=".001" value={project.environment.latitude} onChange={e=>changeEnv({latitude:asNum(e.target.value,project.environment.latitude)})}/></label>
              <label>Longitude (°)<input type="number" step=".001" value={project.environment.longitude} onChange={e=>changeEnv({longitude:asNum(e.target.value,project.environment.longitude)})}/></label>
            </div>
            {project.environment.mode==='live'&&<><label>Forecast instant (UTC)<input type="datetime-local"
              value={new Date(project.environment.timestamp).toISOString().slice(0,16)}
              onChange={e=>{const ms=Date.parse(e.target.value+'Z');if(Number.isFinite(ms))changeEnv({timestamp:new Date(ms).toISOString(),fetchedAt:undefined});}}/></label>
              <button className="forecast-button" onClick={fetchWeather} disabled={weatherBusy}><CloudSun size={15}/>{weatherBusy?'Loading…':'Fetch live forecast'}</button>
              {weatherError&&<div className="hint warn" role="alert">{weatherError}</div>}
            </>}
            <div className="form-row"><label>Wind from (°)<input type="number" min="0" max="360" step="5" value={project.environment.windDirection}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({windDirection:asNum(e.target.value,project.environment.windDirection)})}/></label>
              <label>Wind speed (m/s)<input type="number" min="0" step=".1" value={project.environment.windSpeed}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({windSpeed:asNum(e.target.value,project.environment.windSpeed)})}/></label></div>
            <div className="form-row"><label>Gust (m/s)<input type="number" min="0" step=".1" value={project.environment.gust}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({gust:asNum(e.target.value,project.environment.gust)})}/></label>
              <label>Temperature (°C)<input type="number" step=".5" value={project.environment.temperature}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({temperature:asNum(e.target.value,project.environment.temperature)})}/></label></div>
            <div className="hint weather-source"><Wind size={14}/><span>{project.environment.source}<br/>{project.environment.fetchedAt?'Fetched: '+project.environment.fetchedAt:''}</span></div>
            <div className="hint warn">Weather is used for analytical air-relative speed only. It is NOT injected into the planner or Gazebo. Gust and vertical winds are not simulated.</div></>}
          </div>
          <div className="inspector-section">
            <div className="group-title">SELECTED POINT</div>
            {selectedPoint?<><div className="selected-id">{selectedPoint.id}</div>
              <div className="hint">North {round(selectedPoint.north)} m · East {round(selectedPoint.east)} m · Alt {round(selectedPoint.altitude)} m</div>
              <label className="checkbox-row"><input type="checkbox" checked={selectedPoint.hard} onChange={e=>editPoint(selected,{...selectedPoint,hard:e.target.checked})}/> Required full stop</label>
              <label className="checkbox-row"><input type="checkbox" checked={selectedPoint.exact} onChange={e=>editPoint(selected,{...selectedPoint,exact:e.target.checked})}/> Exact point</label>
            </>:<div className="hint">Select a point on the map or in the table.</div>}
          </div>
        </div>
      </aside>
    </div>
    <footer className="statusbar"><span><span className="status-indicator"/> {result?.accepted?'Reference verified':'No active verified result'}</span>
      <span>Display: NED → East/North/Up · SI</span><span>Weather: {project.environment.mode} · analysis only</span><span>PX4/Gazebo: Not run</span>
      <span>Milestones 1–2 preview / NOT FLIGHT APPROVAL</span></footer>
  </div>;
}
