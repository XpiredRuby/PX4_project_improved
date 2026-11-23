import { useEffect, useMemo, useRef, useState } from 'react';
import { Activity, ArrowDownToLine, ArrowLeft, ArrowRight, ChevronDown, CircleDashed, CloudSun, CornerUpRight, Crosshair, FolderOpen, Grip, HelpCircle, Layers, Map as MapIcon, MousePointer2, MoveUp, Navigation, Pause, Play, Plus, RotateCcw, Route, Save, ScanLine, Settings2, ShieldCheck, TriangleAlert, Wind, X } from 'lucide-react';
import type { Environment, Point, Preview, Project, Sample, Tool } from './types';
import { airspeed, buildRequest, exportBlob, getPreviewSummary, initialEnvironment, interpolate, metrics, nextWaypointId, parseCsv, projectFromJson, sampleProject, updateEnvironment, validateProject } from './engineering';
import fiveStopStudy from '../examples/200m-five-stops.px4-project.json';
import {sha256} from './missionReport';
import WorkspaceHelp, {type HelpTopic} from './WorkspaceHelp';
import WaypointDock from './WaypointDock';
import ConnectedRehearsal from './ConnectedRehearsal';
import type {Job} from './rehearsalHandoff';
import MissionComparison from './MissionComparison';
import StudyBatches from './StudyBatches';
import FlightReplay from './FlightReplay';
import Sweep from './Sweep';
import Prediction from './Prediction';
import WindStudy from './WindStudy';
import Viewport from './Viewport';
import Plot, { type Field } from './Plot';

const examples=[
  ['Straight','Straight path',MoveUp],['L','L-shaped route',CornerUpRight],['five-stops','200 m · five stops',Route],
  ['Survey','Survey route',ScanLine],['Circle','Loop route',CircleDashed]
] as const;
const workspaceSteps=[
  {id:'geometry',label:'Build route',icon:MapIcon},
  {id:'trajectory',label:'Preview flight',icon:Layers},
  {id:'analysis',label:'Review results',icon:Activity},
] as const;
type WorkspaceStep = typeof workspaceSteps[number]['id'];
const resultPanels=[
  ['checks','Checks'],['simulation','Simulator'],['charts','Charts'],['weather','Wind analysis'],
  ['studies','Saved studies'],['speeds','Speed sweep'],['recordings','Recordings'],['comparison','Compare flights'],
] as const;
type ResultPanel = typeof resultPanels[number][0];
const propertyPanels=[['setup','Aircraft'],['weather','Weather'],['point','Point']] as const;
type PropertyPanel = typeof propertyPanels[number][0];
const drawingHints: Record<Tool,string> = {
  select:'Drag a point to move it, or click it to edit its coordinates.',
  waypoint:'Click the map to add a point at the end of the route.',
  draw:'Drag across the map to draw a path.',
  pan:'Drag the map to move the view.',
};
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
    hard:i===0||i===arr.length-1,exact:false}));
};
const downloadJson=(filename:string,data:unknown)=>exportBlob(filename,JSON.stringify(data,null,2)+'\n','application/json');
const display=(n:number,unit:string,digits=2)=>Number.isFinite(n)?n.toFixed(digits)+' '+unit:'—';
const asNum=(value:string,current:number)=>value.trim()===''?current:Number(value);

function ErrorPanel({message,onClose}:{message:string;onClose:()=>void}){
  return <div className="notice error" role="alert"><TriangleAlert size={16}/><span>{message}</span><button aria-label="Dismiss error" onClick={onClose}><X size={16}/></button></div>;
}
function TopMap({points,selected,onSelect,onChange,tool,onAdd,onDraw,onFinishDraw,range,setRange,radius,fitRequest}:{
  points:Point[];selected:number;onSelect:(index:number)=>void;onChange:(index:number,point:Point)=>void;
  tool:Tool;onAdd:(north:number,east:number)=>void;onDraw:(north:number,east:number,first:boolean)=>void;
  onFinishDraw:()=>void;radius:number;range:number;setRange:(v:number)=>void;fitRequest:number;
}){
  const ref=useRef<SVGSVGElement>(null);
  const dragging=useRef<number|null>(null);
  const drawing=useRef(false);
  const lastFit=useRef<number|null>(null);
  const panning=useRef<{x:number;y:number}|null>(null);
  const [center,setCenter]=useState({north:0,east:0});
  const view=range*2;
  const scaleDistance = view >= 400 ? 100 : view >= 150 ? 50 : view >= 65 ? 20 : 10;
  const gridSpacing = view >= 400 ? 100 : view >= 160 ? 25 : 10;
  // Maintain an equal physical map scale on rectangular and square viewports.
  const [mapWidth,setMapWidth]=useState(600);
  const [aspect,setAspect]=useState(1.65);
  useEffect(()=>{if(!ref.current)return;const obs=new ResizeObserver(es=>{const e=es[0];if(e.contentRect.height>0&&e.contentRect.width>0){setAspect(e.contentRect.width/e.contentRect.height);setMapWidth(e.contentRect.width);}});obs.observe(ref.current);return()=>obs.disconnect();},[]);
  const visibleHeight=view/aspect;
  const unitsPerPixel=view/mapWidth;
  const left=center.east-range;
  const top=-center.north-visibleHeight/2;
  const bottom=top+visibleHeight;
  const heavyGrid=gridSpacing*2;
  const eastLines=Array.from({length:Math.ceil(view/gridSpacing)+1},(_,i)=>(Math.floor(left/gridSpacing)+i)*gridSpacing);
  const northLines=Array.from({length:Math.ceil(visibleHeight/gridSpacing)+1},(_,i)=>-(Math.floor(-bottom/gridSpacing)+i)*gridSpacing);
  const fitRoute=()=>{
    let minNorth=0,maxNorth=0,minEast=0,maxEast=0;
    for(const p of points){
      minNorth=Math.min(minNorth,p.north);maxNorth=Math.max(maxNorth,p.north);
      minEast=Math.min(minEast,p.east);maxEast=Math.max(maxEast,p.east);
    }
    setCenter({north:(minNorth+maxNorth)/2,east:(minEast+maxEast)/2});
    const bounds=ref.current?.getBoundingClientRect();
    const mapAspect=bounds&&bounds.height>0 ? bounds.width/bounds.height : aspect;
    setRange(Math.max(12,.6*Math.max(maxEast-minEast,(maxNorth-minNorth)*mapAspect)));
  };
  useEffect(()=>{
    if(lastFit.current===fitRequest)return;
    lastFit.current=fitRequest;
    fitRoute();
  });
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
      const bounds=e.currentTarget.getBoundingClientRect();
      setCenter(c=>({east:c.east-dx*view/bounds.width, north:c.north+dy*visibleHeight/bounds.height}));
    }
  };
  const onUp=()=>{dragging.current=null;panning.current=null;if(drawing.current)onFinishDraw();drawing.current=false;};
  return <div className="map-frame">
    <div className="map-controls"><span><Crosshair size={13} aria-hidden="true"/> N / E <span className="map-frame-label">· Local metres</span></span><div>
      <button onClick={()=>setRange(Math.max(12,range/1.25))} title="Zoom in" aria-label="Zoom in">+</button>
      <button onClick={()=>setRange(Math.min(Math.max(1200,1200*aspect),range*1.25))} title="Zoom out" aria-label="Zoom out">−</button>
      <button onClick={fitRoute} title="Fit route to map">Fit route</button>
      <button onClick={()=>{setRange(45);setCenter({north:0,east:0});}} title="Reset view" aria-label="Reset map view"><Crosshair size={13}/></button>
    </div></div>
    <svg ref={ref} className={'map-svg tool-'+tool} viewBox={[left,top,view,visibleHeight].join(' ')}
      onPointerDown={onDown} onPointerMove={onMove} onPointerUp={onUp} onPointerCancel={onUp} aria-label="Mission coordinate drawing canvas">
      <g className="map-grid" aria-hidden="true">
        {eastLines.map(east=><path key={east} d={`M${east} ${top}v${visibleHeight}`} className={east===0?'map-axis':undefined} vectorEffect="non-scaling-stroke"/>)}
        {northLines.map(y=><path key={y} d={`M${left} ${y}h${view}`} className={y===0?'map-axis':undefined} vectorEffect="non-scaling-stroke"/>)}
      </g>
      <g className="map-ticks" fontSize={10*unitsPerPixel} aria-hidden="true">
        {eastLines.filter(east=>east%heavyGrid===0&&east>left+20*unitsPerPixel&&east<left+view-42*unitsPerPixel)
          .map(east=><text key={east} x={east} y={top+17*unitsPerPixel} textAnchor="middle">{east}</text>)}
        {northLines.filter(y=>y%heavyGrid===0&&y>top+37*unitsPerPixel&&y<bottom-30*unitsPerPixel)
          .map(y=><text key={y} x={left+8*unitsPerPixel} y={y-4*unitsPerPixel}>{String(-y)}</text>)}
      </g>
      <circle className="map-radius" cx="0" cy="0" r={radius} fill="none" strokeWidth="1" strokeDasharray="5 5" vectorEffect="non-scaling-stroke"/>
      <path className="route-halo" d={path} strokeWidth="7" fill="none" strokeLinejoin="round" strokeLinecap="round" vectorEffect="non-scaling-stroke"/>
      <path className="route-trace" d={path} strokeWidth="2" fill="none" strokeLinejoin="round" strokeLinecap="round" vectorEffect="non-scaling-stroke"/>
      {points.map((p,index)=>{
        const isEndpoint=index===0||index===points.length-1;
        const isStop=p.hard||isEndpoint;
        const isSelected=selected===index;
        if(points.length>=80&&index%Math.ceil(points.length/60)!==0&&!isStop&&!isSelected)return null;
        return <g key={p.id+'-'+index} className={`map-point${isStop?' map-stop':''}${isSelected?' map-selected':''}`}
          transform={`translate(${p.east} ${-p.north}) scale(${unitsPerPixel})`}
          onPointerDown={e=>{if(tool!=='select')return;e.stopPropagation();dragging.current=index;onSelect(index);e.currentTarget.closest('svg')?.setPointerCapture(e.pointerId);}}>
          <circle r="11" fill="transparent"/>
          {isSelected&&<circle className="map-selection-ring" r="9" strokeWidth="1" fill="none"/>}
          {isStop?<path className="map-node" d="M 0 -5 L 5 0 L 0 5 L -5 0 Z"/>:<circle className="map-node" r="3.5"/>}
          {(isStop||isSelected)&&<text x="10" y="-7">{p.id}</text>}
        </g>;
      })}
    </svg>
    <div className="map-compass" aria-label="North is up"><span>N</span><Navigation size={22} aria-hidden="true"/></div>
    <div className="map-ruler" aria-label={`Map scale ${scaleDistance} metres`}>
      <span style={{width:`${100*scaleDistance/view}%`}}><i aria-hidden="true"/>{scaleDistance} m</span>
    </div>
    <div className="viewport-status"><span>Grid every {gridSpacing} m</span><span><i aria-hidden="true"/> {radius} m operating radius</span></div>
  </div>;
}

export default function App(){
  const [project,setProject]=useState<Project>(()=>({...sampleProject,environment:{...initialEnvironment},points:sampleProject.points.map(p=>({...p}))}));
  const [tool,setTool]=useState<Tool>('select');
  const [selected,setSelected]=useState(1);
  const [range,setRange]=useState(45);
  const [mapFitRequest,requestMapFit]=useState(0);
  const [active,setActive]=useState<WorkspaceStep>('geometry');
  const [field,setField]=useState<Field>('speed');
  const [resultPanel,setResultPanel]=useState<ResultPanel>('checks');
  const [windPanel,setWindPanel]=useState<'estimate'|'compare'>('estimate');
  const [showChartStats,setShowChartStats]=useState(false);
  const [propertyPanel,setPropertyPanel]=useState<PropertyPanel>('setup');
  const [utilityPane,setUtilityPane]=useState<'workspace'|'project'|'properties'>('workspace');
  const [helpTopic,setHelpTopic]=useState<HelpTopic>('start');
  const helpDialog=useRef<HTMLDialogElement>(null);
  const stepHeading=useRef<HTMLHeadingElement>(null);
  const openHelp=(topic:HelpTopic)=>{
    setHelpTopic(topic);
    if(!helpDialog.current?.open)helpDialog.current?.showModal();
  };
  const openStep=(step:WorkspaceStep)=>{
    setActive(step);
    setUtilityPane('workspace');
    requestAnimationFrame(()=>stepHeading.current?.focus({preventScroll:true}));
  };
  const [advanced,setAdvanced]=useState(false);
  const [paneWidths,setPaneWidths]=useState({left:235,right:288});
  const resizing=useRef<{edge:'left'|'right';pointerX:number;start:number}|null>(null);
  const [busy,setBusy]=useState(false);
  const [sweeping,setSweeping]=useState(false);
  const [profileBusy,setProfileBusy]=useState(false);
  const [error,setError]=useState('');
  const [notice,setNotice]=useState('');
  const [result,setResult]=useState<Preview|null>(null);
  const [samples,setSamples]=useState<Sample[]>([]);
  const [routeSamples,setRouteSamples]=useState<Sample[]>([]);
  const [fullMissionSamples,setFullMissionSamples]=useState<Sample[]>([]);
  const [viewScope,setViewScope]=useState<'route'|'full'>('full');
  const [cursor,setCursor]=useState(0);
  const [playing,setPlaying]=useState(false);
  const [registered,setRegistered]=useState(false);
  const [sitl,setSitl]=useState(false);
  const [job,setJob]=useState<Job|null>(null);
  const selectedJob=job&&result?.plan_id===job.plan_id?job:null;
  const [hasQueue,setHasQueue]=useState(false);
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
  const playbackStart=useRef(0);
  const lastDraw=useRef<Point|null>(null);
  const geometryRevision=useRef(0);
  const history=useRef<Project[]>([]);

  const onResizeStart=(edge:'left'|'right',event:React.PointerEvent<HTMLDivElement>)=>{
    resizing.current={edge,pointerX:event.clientX,start:paneWidths[edge]};
    event.currentTarget.setPointerCapture(event.pointerId);
  };
  const onResizeMove=(event:React.PointerEvent<HTMLDivElement>)=>{
    const r=resizing.current;if(!r)return;
    const delta=(event.clientX-r.pointerX)*(r.edge==='left'?1:-1);
    setPaneWidths(p=>({...p,[r.edge]:Math.max(r.edge==='left'?180:230,Math.min(r.edge==='left'?400:440,r.start+delta))}));
  };
  const onResizeEnd=()=>{resizing.current=null;};
  useEffect(()=>{fetch('/api/capabilities').then(r=>r.json()).then(d=>{setEngine(d.execution_location==='browser'?'browser':'local');setHasQueue(d.study_queue===true);setSitl(d.sitl_enabled===true);setRegistered(d.registered_aircraft===true);}).catch(()=>setEngine('unknown'));},[]);
  useEffect(()=>()=>{abort.current?.abort();cancelAnimationFrame(frame.current);},[]);
  const projectRef=useRef(project);
  const manualWeather=useRef(initialEnvironment);
  const replaceProject=(next:Project)=>{
    projectRef.current=next;
    setProject(next);
  };
  const invalidate=()=>{
    geometryRevision.current++;
    abort.current?.abort();
    setResult(null);setSamples([]);setRouteSamples([]);setFullMissionSamples([]);setCursor(0);setPlaying(false);setDirty(true);setError('');setNotice('');
  };
  const update=(f:(p:Project)=>Project)=>{
    const previous=projectRef.current,next=f(previous);
    if(next===previous)return;
    history.current=[...history.current.slice(-19),previous];
    if(next.environment!==previous.environment)manualWeather.current=next.environment.mode==='manual'?next.environment:initialEnvironment;
    replaceProject(next);
    invalidate();
  };
  const selectAircraft=async(preset:string)=>{
    if(preset==='baseline'){update(p=>({...p,aircraft:undefined}));return;}
    setProfileBusy(true);
    const revision=geometryRevision.current;
    try{
      const response=await fetch(preset==='registered'?'/api/aircraft/binding':'/api/aircraft/example/'+preset);
      if(!response.ok)throw Error('Aircraft preset unavailable');
      const data=await response.json();
      const profile=preset==='registered'?data.profile:data;
      const assessed=await fetch('/api/aircraft/assess',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(profile)});
      const setup=await assessed.json();
      if(!assessed.ok||!setup.ready)throw Error('Aircraft profile could not be assessed');
      if(revision===geometryRevision.current)update(p=>({...p,aircraft:{preset,profile,sha256:setup.profile_sha256}}));
    }catch(e){setError((e as Error).message);}finally{setProfileBusy(false);}
  };
  const changeValue=(key:keyof Project,value:number|string)=>{
    update(p=>({...p,[key]:value}));
  };
  const changeEnv=(patch:Partial<Environment>)=>{
    const current=projectRef.current;
    const environment=updateEnvironment(current.environment,patch);
    if(environment.mode==='manual')manualWeather.current=environment;
    replaceProject({...current,environment});
    setDirty(true);
    setWeatherError('');
  };
  const setWeatherMode=(mode:Environment['mode'])=>{
    const current=projectRef.current.environment;
    if(mode===current.mode)return;
    changeEnv(mode==='manual'?{...manualWeather.current,mode,latitude:current.latitude,longitude:current.longitude,
      timestamp:current.timestamp,fetchedAt:undefined,forecastValidAt:undefined,source:initialEnvironment.source}:{mode});
  };
  const editPoint=(index:number,point:Point)=>update(p=>({...p,points:p.points.map((v,i)=>i===index?point:v)}));
  const newPoint=(north:number,east:number)=>{
    const index=projectRef.current.points.length;
    if(index>=8192){setError('This route already has 8,192 points.');return;}
    update(p=>({...p,points:[...p.points,{id:nextWaypointId(p.points),north,east,altitude:p.altitude,hard:false,exact:false}]}));
    setSelected(index);
  };
  const drawPoint=(north:number,east:number,first:boolean)=>{
    const current=projectRef.current;
    if(current.points.length>=8192){setError('This route already has 8,192 points.');return;}
    const previous=lastDraw.current;
    if(!first&&previous&&Math.hypot(previous.north-north,previous.east-east)<.6)return;
    if(first)history.current=[...history.current.slice(-19),current];
    const point:Point={id:'D'+sid(),north,east,altitude:current.altitude,hard:false,exact:false};
    lastDraw.current=point;
    replaceProject({...current,points:[...current.points,point]});
    invalidate();
  };
  const finishDraw=()=>{
    if(lastDraw.current)setNotice('Sketch added. Run verification to review the planned route.');
    lastDraw.current=null;
  };
  const clearRoute=()=>{
    update(p=>({...p,title:'New mission',points:[]}));
    setTool('draw');setSelected(-1);requestMapFit(n=>n+1);
  };
  const undo=()=>{
    const previous=history.current.pop();
    if(!previous)return;
    if(previous.environment!==projectRef.current.environment)
      manualWeather.current=previous.environment.mode==='manual'?previous.environment:initialEnvironment;
    replaceProject(previous);
    setSelected(-1);
    invalidate();
  };
  const applyExample=(name:string)=>{
    update(p=>name==='five-stops'?projectFromJson(structuredClone(fiveStopStudy)):{...p,title:name+' study',points:example(name,p.altitude)});
    setSelected(-1);requestMapFit(n=>n+1);
  };
  const draftDimensions=useMemo(()=>{
    let length=0, minimum=Infinity, maximum=-Infinity;
    for(const point of project.points){
      minimum=Math.min(minimum,point.altitude);
      maximum=Math.max(maximum,point.altitude);
    }
    for(let i=1;i<project.points.length;i++){
      const a=project.points[i-1],b=project.points[i];
      length+=Math.hypot(b.north-a.north,b.east-a.east,b.altitude-a.altitude);
    }
    return {length,minimum,maximum};
  },[project.points]);
  const modelRadius=project.aircraft?Math.min(80,Number(project.aircraft.profile.max_radius_m)):40;
  const selectedPoint=project.points[selected];
  const stats=useMemo(()=>metrics(samples,project.environment),[samples,project.environment]);
  const current=useMemo(()=>interpolate(samples,cursor),[samples,cursor]);
  const duration=stats?.duration||0;
  const modelCap=project.aircraft?.profile.maximum_speed_m_s??3;
  const cap=Math.min(project.speedCap,modelCap);
  const weatherReady=project.environment.mode==='manual'||!!project.environment.fetchedAt;
  const visibleField=field==='airspeed'&&!weatherReady?'speed':field;

  useEffect(()=>{
    if(!playing||samples.length<2)return;
    const t0=performance.now(),start=playbackStart.current,end=samples[samples.length-1].time;
    const tick=(now:number)=>{
      const next=Math.min(end,start+Math.max(0,(now-t0)/1000));
      setCursor(next);
      if(next<end)frame.current=requestAnimationFrame(tick);
      else setPlaying(false);
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
      const routeDigest=await sha256(data.trajectory_csv);
      if(routeDigest!==data.summary?.controller_trajectory_sha256)
        throw Error('The verified route and its checksum differ.');
      if(data.mission?.trajectory_csv&&(data.mission.controller_route_sha256!==routeDigest||
        data.mission.trajectory_sha256!==await sha256(data.mission.trajectory_csv)))
        throw Error('The full mission checksum or its linked route differs.');
      const parsed=parseCsv(data.trajectory_csv);
      const full=data.mission?.trajectory_csv?parseCsv(data.mission.trajectory_csv):[];
      controller.signal.throwIfAborted();
      if(submittedRevision!==geometryRevision.current){setNotice('Edited during compilation; discarded the obsolete result. Run verification again.');return;}
      if(!parsed.length)throw Error('Verified plan returned no usable samples');
      setResult(data);setRouteSamples(parsed);setFullMissionSamples(full);setViewScope(full.length?'full':'route');setSamples(full.length?full:parsed);setCursor(0);setResultPanel('checks');openStep('analysis');setDirty(false);
      setNotice('Route verified. Review the results or preview the planned motion.');
    }catch(e){if((e as Error).name!=='AbortError')setError((e as Error).message);else setNotice('Planning cancelled or invalidated by an edit. Your mission was kept.');}
    finally{setBusy(false);abort.current=null;}
  };
  const fetchWeather=async()=>{
    setWeatherBusy(true);setWeatherError('');
    const env=projectRef.current.environment;
    try{
      if(!Number.isFinite(env.latitude)||!Number.isFinite(env.longitude)||Math.abs(env.latitude)>90||Math.abs(env.longitude)>180)
        throw Error('Enter valid latitude and longitude');
      const q=new URLSearchParams({latitude:String(env.latitude),longitude:String(env.longitude),timezone:'GMT',wind_speed_unit:'ms',
        hourly:'relative_humidity_2m,wind_speed_10m,wind_direction_10m,wind_gusts_10m,temperature_2m,surface_pressure',forecast_days:'7'});
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
      const vals=['relative_humidity_2m','wind_speed_10m','wind_direction_10m','wind_gusts_10m','temperature_2m','surface_pressure'];
      if(vals.some(k=>!Number.isFinite(h[k]?.[i])))throw Error('Forecast contains missing required weather values');
      if(env!==projectRef.current.environment)return;
      changeEnv({humidity:h.relative_humidity_2m[i], forecastValidAt:times[i]+'Z', windSpeed:h.wind_speed_10m[i],windDirection:h.wind_direction_10m[i],gust:h.wind_gusts_10m[i],
        temperature:h.temperature_2m[i],pressure:h.surface_pressure[i],
        fetchedAt:new Date().toISOString(),
        source:'Open-Meteo forecast · 10 m modeled wind · hourly UTC snapshot'});
    }catch(e){if(env===projectRef.current.environment)setWeatherError((e as Error).message);}
    finally{setWeatherBusy(false);}
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
        const safeId=read('point_id')||'P'+String(i+1).padStart(2,'0');
        return {id:safeId,north:number('north_m'),east:number('east_m'),altitude:number('altitude_m'),
          hard:read('hard')==='true'||read('hard')==='1'||i===0||i===rows.length-1,
          exact:(read('exact')==='true'||read('exact')==='1') && !(read('hard')==='true'||read('hard')==='1'||i===0||i===rows.length-1)} satisfies Point;
      });
      const next={...projectRef.current,title:file.name.replace(/\.csv$/i,''),points:imported};
      validateProject(next);
      update(()=>next);setSelected(-1);requestMapFit(n=>n+1);openStep('geometry');
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
      update(()=>imported);setSelected(-1);requestMapFit(n=>n+1);openStep('geometry');
    }catch(e){setError((e as Error).message);}
  };
  const saveProject=()=>downloadJson(project.title.replace(/[^a-z0-9_-]/gi,'_')+'.px4-project.json',project);
  const saveReference=()=>{if(result?.trajectory_csv)exportBlob('verified-route.csv',result.trajectory_csv,'text/csv');};
  const changeScope=(scope:'route'|'full')=>{
    const next=scope==='route'?routeSamples:fullMissionSamples;
    if(!next.length)return;
    setViewScope(scope);setSamples(next);setPlaying(false);setCursor(next[0].time);
  };
  const summary=getPreviewSummary(result||{accepted:false});
  return <div className="app-shell">
    <header className="topbar">
      <div className="brand"><span className="brand-mark"><Route size={17} aria-hidden="true"/></span><strong>PX4 <span>Trajectory Studio</span></strong></div>
      <span className="project-title" title={project.title}><FolderOpen size={14} aria-hidden="true"/><span>{project.title}{dirty?' *':''}</span></span>
      <div className="top-actions">
        <button aria-label="Open project" title="Open project" onClick={()=>fileRef.current?.click()}><FolderOpen size={16}/><span>Open</span></button>
        <button aria-label="Save editable project" title="Save editable project" onClick={saveProject}><Save size={16}/><span>Save</span></button>
        <button className="help-button" aria-haspopup="dialog" onClick={()=>openHelp('start')}><HelpCircle size={16}/> Help</button>
        <input ref={fileRef} hidden type="file" accept=".json" onChange={importProject}/>
        <input ref={pointsCsvRef} hidden type="file" accept=".csv,text/csv" onChange={importWaypointCsv}/>
      </div>
    </header>
    <div className="command-bar">
      <nav className="workflow" aria-label="Mission workflow">
        {workspaceSteps.map(step=><button key={step.id} aria-current={active===step.id?'step':undefined} onClick={()=>openStep(step.id)}>
          <step.icon size={15} aria-hidden="true"/>{step.label}
        </button>)}
      </nav>
      <div className="command-actions">
        <button className="compact-pane project-pane-button" aria-pressed={utilityPane==='project'}
          onClick={()=>setUtilityPane(p=>p==='project'?'workspace':'project')}><FolderOpen size={15}/> Project</button>
        <button className="compact-pane properties-pane-button" aria-pressed={utilityPane==='properties'}
          onClick={()=>setUtilityPane(p=>p==='properties'?'workspace':'properties')}><Settings2 size={15}/> Settings</button>
        <span className="command-spacer"/>
        {job&&<button className="run-shortcut" onClick={()=>{setResultPanel('simulation');openStep('analysis');}}>Simulator run</button>}
        <button className="undo-button" onClick={undo} disabled={history.current.length===0}><RotateCcw size={15}/> Undo</button>
        <button className="primary" onClick={run} disabled={busy||sweeping||profileBusy||project.points.length<2}>
          {busy?<span className="spinner"/>:<ShieldCheck size={16}/>} {busy?'Verifying…':'Plan & verify'}
        </button>
        {busy&&<button onClick={()=>abort.current?.abort()}>Cancel planning</button>}
      </div>
    </div>
    <div className={`workbench show-${utilityPane}`} style={{'--left-pane':paneWidths.left+'px','--right-pane':paneWidths.right+'px'} as React.CSSProperties}>
      <aside className="left-rail" aria-label="Route library">
        <div className="pane-title"><FolderOpen size={15}/> Project</div>
        <div className="library-body">
        <div className="operations">
          <button className="new-route" onClick={()=>{clearRoute();openStep('geometry');}}><Plus size={17}/> New blank mission</button>
          <button onClick={()=>pointsCsvRef.current?.click()}><FolderOpen size={16}/> Import waypoint CSV</button>
          <button onClick={exportWaypointCsv}><ArrowDownToLine size={16}/> Export waypoint CSV</button>
        </div>
        <div className="pane-title second-title">Example routes</div>
        <div className="operations example-list">
          {examples.map(([id,label,Icon])=><button key={id} onClick={()=>{applyExample(id);openStep('geometry');}}><Icon size={15}/>{label}<ArrowRight size={14}/></button>)}
        </div>
        </div>
        <div className="pane-resizer left-resizer" role="separator" aria-label="Resize model explorer" aria-orientation="vertical" tabIndex={0}
          onKeyDown={e=>{if(e.key==='ArrowLeft')setPaneWidths(w=>({...w,left:Math.max(180,w.left-15)}));if(e.key==='ArrowRight')setPaneWidths(w=>({...w,left:Math.min(400,w.left+15)}));}}
          onPointerDown={e=>onResizeStart('left',e)} onPointerMove={onResizeMove} onPointerUp={onResizeEnd} onPointerCancel={onResizeEnd}/>
      </aside>
      <main className="main-pane" aria-labelledby="workspace-heading">
        <div className="dock-bar workspace-heading">
          <h1 id="workspace-heading" ref={stepHeading} tabIndex={-1}>{workspaceSteps.find(step=>step.id===active)?.label}</h1>
          {active==='geometry'&&<div className="route-summary" aria-label="Draft route dimensions">
            <span><span>Length</span><strong>{display(draftDimensions.length,'m',1)}</strong></span>
            <span><span>Height</span><strong>{draftDimensions.minimum!==draftDimensions.maximum&&project.points.length
              ?`${draftDimensions.minimum.toFixed(1)}–${draftDimensions.maximum.toFixed(1)} m`
              :display(draftDimensions.maximum,'m',1)}</strong></span>
            <span><span>Points</span><strong>{project.points.length.toLocaleString()}</strong></span>
          </div>}
          {active==='analysis'&&<button aria-label="Results help" aria-haspopup="dialog" onClick={()=>openHelp('results')}><HelpCircle size={16}/></button>}
        </div>
        {error&&<ErrorPanel message={error} onClose={()=>setError('')}/>}
        {active==='geometry'&&<div className="editor-dock">
          <section className="route-workspace" aria-label="Route map">
            <div className="route-canvas">
              <div className="toolstrip">
                <div className="tool-group">
                  {([{id:'select',icon:MousePointer2,label:'Select / move'},{id:'waypoint',icon:Plus,label:'Add point'},
                    {id:'draw',icon:Grip,label:'Freehand'},{id:'pan',icon:Crosshair,label:'Pan'}] as const).map(t=>
                    <button key={t.id} onClick={()=>setTool(t.id)} aria-pressed={tool===t.id} title={t.label}>
                      <t.icon size={15}/><span>{t.label}</span>
                    </button>)}
                </div>
                <button className="contextual-help" aria-haspopup="dialog" aria-label="Map help" onClick={()=>openHelp('map')}>
                  <HelpCircle size={16}/><span>Map help</span>
                </button>
              </div>
              <div className="viewport-panel">
                <TopMap points={project.points} selected={selected} onSelect={setSelected} onChange={editPoint} tool={tool}
                  onAdd={newPoint} onDraw={drawPoint} onFinishDraw={finishDraw} radius={Number.isFinite(modelRadius)?modelRadius:40}
                  range={range} setRange={setRange} fitRequest={mapFitRequest}/>
              </div>
            </div>
          </section>
          <WaypointDock points={project.points} selected={selected} select={setSelected} change={editPoint}
            add={()=>newPoint(0,0)} remove={()=>{update(p=>({...p,points:p.points.filter((_,i)=>i!==selected)}));setSelected(-1);}}/>
        </div>}
        {active==='trajectory'&&<div className="preview-dock">
        <div className="viewport-panel">
          <div className="field-select"><label htmlFor="preview-colour">Colour route by</label>
            <select id="preview-colour" value={visibleField} onChange={e=>setField(e.target.value as Field)}>
              <option value="speed">3D speed</option><option disabled={!weatherReady} value="airspeed">Wind-relative airspeed</option>
              <option value="acceleration">Acceleration</option><option value="jerk">Jerk</option><option value="altitude">Altitude</option>
              <option value="yaw">Heading</option>
            </select>
          </div>
          <Viewport points={project.points} samples={samples} cursor={cursor} field={visibleField} environment={project.environment}/>
        </div>
        <section className="playback-dock">
          <div className="section-heading"><strong>Playback</strong>
          <div className="timeline-buttons"><div className="scope-tabs"><button className={viewScope==='route'?'scope-current':''} disabled={!routeSamples.length} onClick={()=>changeScope('route')}>Route</button><button className={viewScope==='full'?'scope-current':''} disabled={!fullMissionSamples.length} onClick={()=>changeScope('full')}>Full mission</button></div><button aria-label="Play or pause" onClick={()=>{if(playing){setPlaying(false);return;}
              const start=cursor>=samples[samples.length-1].time?samples[0].time:cursor;
              playbackStart.current=start;setCursor(start);setPlaying(true);}} disabled={!samples.length}>{playing?<Pause size={16}/>:<Play size={16}/>} {playing?'Pause':'Play'}</button>
            <button onClick={()=>{setPlaying(false);setCursor(samples[0]?.time||0);}} disabled={!samples.length} title="Reset time"><RotateCcw size={15}/></button></div></div>
          <div className="scrubber"><input aria-label="Reference time" type="range" min={samples[0]?.time||0} max={samples[samples.length-1]?.time||1} step=".01" value={cursor} disabled={!samples.length} onChange={e=>{setPlaying(false);setCursor(Number(e.target.value));}}/>
            <span>{display(cursor,'s')} / {display(duration,'s')}</span></div>
          {current&&<div className="sample-readout">
            <div><span>TIME</span><strong>{display(current.time,'s')}</strong>{current.phase&&<span>{current.phase.replaceAll('_',' ')}</span>}</div>
            <div><span>N / E</span><strong>{round(current.north)} / {round(current.east)} m</strong></div>
            <div><span>HEIGHT</span><strong>{display(current.altitude,'m')}</strong></div>
            <div><span>3D SPEED</span><strong>{display(current.speed,'m/s')}</strong></div>
            <div><span>YAW FROM NORTH</span><strong>{display((current.yaw*180/Math.PI%360+360)%360,'°')}</strong></div>
            <div><span>WIND-RELATIVE</span><strong>{display(airspeed(current,project.environment),'m/s')}</strong></div>
          </div>}
        </section>
        </div>}
        {active==='analysis'&&<nav className="result-tabs" aria-label="Result views">
          {resultPanels.map(([id,label])=><button key={id} aria-pressed={resultPanel===id} onClick={()=>setResultPanel(id)}>{label}</button>)}
        </nav>}
        {active==='analysis'&&resultPanel==='checks'&&<div className="analysis-panel" aria-label="Verification checks">
          {result?<section className="data-section verification-card">
          <div className="section-heading"><strong>Route verification</strong>
            <div className="section-actions">{sitl&&<button onClick={()=>setResultPanel('simulation')}>Simulator <ArrowRight size={14}/></button>}<button className="preview-link" onClick={()=>openStep('trajectory')}><Play size={14}/> Play in 3D</button><button onClick={saveReference}><ArrowDownToLine size={14}/> Route CSV</button>
              {result.mission?.trajectory_csv&&<button title="Display-only lifecycle reference; not controller execution input" onClick={()=>exportBlob('planned-full-mission-reference.csv',result.mission?.trajectory_csv||'','text/csv')}><ArrowDownToLine size={14}/> Full mission CSV</button>}
              <button onClick={()=>downloadJson('planner-evidence.json',{schema:'trajectory-studio.scenario.v1',executionBackend:engine,solverSourceSha256:import.meta.env.VITE_SOLVER_DIGEST,project,request:buildRequest(project),result:{...result,trajectory_csv:undefined,mission:{...result.mission,trajectory_csv:undefined}},analysisWeather:project.environment,liveExecution:selectedJob||{status:'not_run',firmware:null,simulationStep:null,seed:null},scope:'verified reference and analytical weather sensitivity'})}><ArrowDownToLine size={14}/> Report JSON</button></div></div>
          <div className="verification-grid">
            <div><span>Planner result</span><strong className="pass-label"><ShieldCheck size={16} aria-hidden="true"/> Verified</strong></div>
            <div><span>Planned route length</span><strong>{display(Number(summary.speed.route_distance_sampled_m),'m',1)}</strong></div>
            <div><span>REQUESTED CRUISE</span><strong>{display(project.cruise,'m/s')}</strong></div>
            <div><span>PLANNED ROUTE PEAK</span><strong>{display(Number(summary.speed.planned_route_peak_sampled_m_s),'m/s')}</strong></div>
            <div><span>MODEL CAP</span><strong>{display(modelCap,'m/s')}</strong></div>
            <div><span>REQUESTED NEVER-EXCEED</span><strong>{display(project.speedCap,'m/s')}</strong></div>
            <div><span>PLANNED CAP</span><strong>{display(Number(summary.speed.planning_cap_m_s),'m/s')}</strong></div>

          </div>
          {result.simulation_compatible===false&&<p className="compatibility-note">{result.preview_settings?.planner_backend==='stop_turn'
            ?'For this rehearsal, keep every point at the same height, start at north 0 and east 0, and use a fixed heading and matching aircraft.'
            :'Choose the aircraft that matches your simulator in Flight settings, then check the route again.'}</p>}

          {result.planning_adjustments?.adjusted&&<div className="adjustments"><strong>Changes to review</strong>
            <p>{result.planning_adjustments.original_rounding_met?'The accepted route fits your path tolerance.':'The accepted route goes outside your path tolerance.'}</p>
            <ul>{result.planning_adjustments.adjustments?.map((a,i)=><li key={i}>{a.message}</li>)}</ul>
          </div>}
          <details><summary>Raw verification evidence and adjustment report</summary><pre>{JSON.stringify({verification:summary.certificate,adjustments:result.planning_adjustments,interpretation:result.interpretation},null,2)}</pre></details>
        </section>:<div className="empty-panel">
            <strong>Check a route to see its planned motion and verification results.</strong>
            <button onClick={()=>openStep('geometry')}><ArrowLeft size={15}/> Build a route</button>
          </div>}
        </div>}
        <div className="analysis-panel" hidden={active!=='analysis'||resultPanel!=='simulation'} aria-label="Simulator results">
          {sitl?<ConnectedRehearsal plan={result} onJob={setJob}/>:<p className="empty-panel">Connect Studio to a local PX4 / Gazebo installation to run a rehearsal.</p>}
        </div>
        {active==='analysis'&&<div className="analysis-panel charts-panel" hidden={resultPanel!=='charts'} aria-label="Reference charts">
          <div className="dock-bar"><label htmlFor="chart-field">Chart</label>
            <select id="chart-field" value={visibleField} onChange={e=>setField(e.target.value as Field)}>
              <option value="speed">3D speed</option><option disabled={!weatherReady} value="airspeed">Wind-relative airspeed</option>
              <option value="acceleration">Acceleration</option><option value="jerk">Jerk</option>
              <option value="altitude">Altitude</option><option value="yaw">Heading</option>
            </select>
            {stats&&<button aria-pressed={showChartStats} onClick={()=>setShowChartStats(v=>!v)}>Statistics</button>}
          </div>
          {showChartStats&&stats?<dl className="metric-table">
            {[
              ['Duration',display(stats.duration,'s',1)],['Distance',display(stats.distance,'m',1)],['Peak 3D speed',display(stats.peak,'m/s')],
              ['Peak airspeed',display(stats.air,'m/s')],['Peak acceleration',display(stats.accel,'m/s²')],['Peak jerk',display(stats.jerk,'m/s³')],
              ['Estimated air density',display(stats.density,'kg/m³',3)],['Peak dynamic pressure',display(stats.peakDynamicPressure,'Pa',1)],
            ].map(([label,value])=><div key={label}><dt>{label}</dt><dd>{value}</dd></div>)}
          </dl>:<Plot samples={samples} field={visibleField} environment={project.environment} cursor={cursor}
            onCursor={t=>{setPlaying(false);setCursor(t);}}
            ceiling={visibleField==='speed'?(viewScope==='route'?Number(summary.speed.planning_cap_m_s)||cap:cap):
              visibleField==='acceleration'?result?.vehicle_model?.maximum_acceleration_m_s2:
              visibleField==='jerk'?result?.vehicle_model?.maximum_jerk_m_s3:undefined}/>}
        </div>}
        {active==='analysis'&&<>
          <div className="analysis-panel" hidden={resultPanel!=='weather'} aria-label="Wind analysis">
            <nav className="dock-bar" aria-label="Wind views">
              <button aria-pressed={windPanel==='estimate'} onClick={()=>setWindPanel('estimate')}>Tracking estimate</button>
              <button aria-pressed={windPanel==='compare'} onClick={()=>setWindPanel('compare')}>Compare winds</button>
            </nav>
            <div hidden={windPanel!=='estimate'}>
              {result?.vehicle_model&&routeSamples.length>0?<Prediction key={geometryRevision.current} samples={routeSamples} environment={project.environment}
                vehicle={result.vehicle_model} digest={result.summary?.controller_trajectory_sha256}/>:<p className="empty-panel">Verify a route to estimate how wind affects tracking.</p>}
            </div>
            <div hidden={windPanel!=='compare'}><WindStudy samples={routeSamples} environment={project.environment}
              digest={result?.summary?.controller_trajectory_sha256}/></div>
          </div>
          <div className="analysis-panel" hidden={resultPanel!=='studies'} aria-label="Saved studies">
            <StudyBatches enabled={hasQueue} project={project} restore={p=>{update(()=>p);setSelected(-1);}}/>
          </div>
          <div className="analysis-panel" hidden={resultPanel!=='comparison'} aria-label="Flight comparison">
            <MissionComparison key={String(result?.summary?.controller_trajectory_sha256??geometryRevision.current)} points={project.points} result={result}/>
          </div>
          <div className="analysis-panel" hidden={resultPanel!=='recordings'} aria-label="Recorded flights"><FlightReplay local={engine==='local'}/></div>
          <div className="analysis-panel" hidden={resultPanel!=='speeds'} aria-label="Speed sweep">
            <Sweep key={geometryRevision.current} project={project} disabled={busy||profileBusy} onBusy={setSweeping}/>
          </div>
        </>}
      </main>
      <aside className="inspector" aria-label="Flight settings">
        <div className="pane-title settings-title"><Settings2 size={15}/> Settings
          <button aria-label="Flight settings help" aria-haspopup="dialog" onClick={()=>openHelp('settings')}><HelpCircle size={16}/></button>
        </div>
        <nav className="property-tabs" aria-label="Settings views">
          {propertyPanels.map(([id,label])=><button key={id} aria-label={id==='weather'?'Weather for analysis':label}
            aria-pressed={propertyPanel===id} onClick={()=>setPropertyPanel(id)}>{label}</button>)}
        </nav>
        <div className="inspector-content" hidden={propertyPanel!=='setup'} aria-label="Aircraft settings">
          <div className="inspector-section">
            <div className="group-title">Aircraft and height</div>
            <label htmlFor="aircraft-model">Aircraft model</label><select id="aircraft-model" value={project.aircraft?.preset??'baseline'} disabled={busy||profileBusy||sweeping} onChange={e=>selectAircraft(e.target.value)}>
              <option value="baseline">X500 · reference model</option>{registered&&<option value="registered">Registered Gazebo aircraft</option>}<option value="x500">X500 · Gazebo SDF profile</option>
              <option value="quad">Illustrative quadrotor</option><option value="hex">Illustrative hexacopter</option><option value="octo">Illustrative octocopter</option></select>
            <div className="hint">{project.aircraft?.profile.name??'X500 reference model'} · modeled speed limit {display(modelCap,'m/s')}.</div>
            {registered&&project.aircraft?.preset!=='registered'&&<div className="hint">Choose Registered Gazebo aircraft to use your connected simulator.</div>}
            <div><label>Height above launch (m)<input type="number" min=".1" max="30" step=".5" value={project.altitude}
              onChange={e=>{const value=asNum(e.target.value,project.altitude);update(p=>({...p,altitude:value,points:p.points.map(q=>({...q,altitude:value}))}));}}/></label></div>
          </div>
          <div className="inspector-section">
            <div className="group-title">Speed</div>
            <div className="form-row"><label>Preferred cruise (m/s)<input type="number" min="0" step="any" value={project.cruise} onChange={e=>changeValue('cruise',asNum(e.target.value,project.cruise))}/></label>
              <label>Never exceed (m/s)<input type="number" min="0" step="any" value={project.speedCap} onChange={e=>changeValue('speedCap',asNum(e.target.value,project.speedCap))}/></label></div>
            {project.cruise>cap&&<div className="hint warn">Requested {project.cruise} m/s. The selected cap is {cap} m/s before the planner margin, turns and stops.</div>}
            <button className="advanced-toggle" aria-expanded={advanced} aria-controls="advanced-settings" onClick={()=>setAdvanced(!advanced)}>
              <ChevronDown size={16}/> Advanced route settings
            </button>
            <div id="advanced-settings" hidden={!advanced}>
              <label>Route tolerance (m)<input type="number" step=".05" min=".01" max="10" value={project.tolerance} onChange={e=>changeValue('tolerance',asNum(e.target.value,project.tolerance))}/></label>
              <label>Fixed heading (° from north)<input type="number" step="5" value={project.yaw} onChange={e=>changeValue('yaw',asNum(e.target.value,project.yaw))}/></label>
              <div className="hint">0° faces north. 90° faces east.</div>
            </div>
          </div>
        </div>
        <div className="inspector-content" hidden={propertyPanel!=='weather'} aria-label="Weather settings">
            <div className="mode-switch">
              <button className={project.environment.mode==='manual'?'mode-active':''} onClick={()=>setWeatherMode('manual')} aria-pressed={project.environment.mode==='manual'}>Manual</button>
              <button className={project.environment.mode==='live'?'mode-active':''} onClick={()=>setWeatherMode('live')} aria-pressed={project.environment.mode==='live'}>Live forecast</button>
            </div>
            <label htmlFor="place-search">Search city or place name</label>
            <div className="place-search"><input id="place-search" aria-label="Search location" value={placeQuery} placeholder="e.g. College Station, Texas" onChange={e=>setPlaceQuery(e.target.value)} onKeyDown={e=>{if(e.key==='Enter')searchPlace();}}/><button disabled={searchingPlace} onClick={searchPlace}>{searchingPlace?'...':'Find'}</button></div>
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
            <div className="form-row"><label>Wind from (°)<input type="number" min="0" max="360" step="5" value={weatherReady?project.environment.windDirection:''}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({windDirection:asNum(e.target.value,project.environment.windDirection)})}/></label>
              <label>Wind speed (m/s)<input type="number" min="0" step=".1" value={weatherReady?project.environment.windSpeed:''}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({windSpeed:asNum(e.target.value,project.environment.windSpeed)})}/></label></div>
            <div className="form-row"><label>Gust (m/s)<input type="number" min="0" step=".1" value={weatherReady?project.environment.gust:''}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({gust:asNum(e.target.value,project.environment.gust)})}/></label>
              <label>Temperature (°C)<input type="number" step=".5" value={weatherReady?project.environment.temperature:''}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({temperature:asNum(e.target.value,project.environment.temperature)})}/></label></div>
            <label>Relative humidity (%)<input type="number" min="0" max="100" step="any" value={weatherReady?(project.environment.humidity??''):''} placeholder="Unknown" disabled={project.environment.mode==='live'} onChange={e=>changeEnv({humidity:e.target.value===''?undefined:Number(e.target.value)})}/></label>
            <label>Surface pressure (hPa)<input type="number" step=".5" min="100" max="1100" value={weatherReady?project.environment.pressure:''}
              disabled={project.environment.mode==='live'} onChange={e=>changeEnv({pressure:asNum(e.target.value,project.environment.pressure)})}/></label>
            <div className="hint weather-source"><Wind size={14}/><span>{project.environment.source}<br/>{project.environment.forecastValidAt?'Valid '+project.environment.forecastValidAt+' · ':''}{project.environment.fetchedAt?'Fetched: '+project.environment.fetchedAt:''}</span></div>
            <div className="hint warn">Steady wind drives airspeed analysis and the offline tracking estimate. The planner checks its selected vehicle envelope. Density assumes dry air. Gust and humidity are retained with the scenario.</div>
        </div>
        <div className="inspector-content inspector-section" hidden={propertyPanel!=='point'} aria-label="Selected waypoint">
            <div className="group-title">SELECTED POINT</div>
            {selectedPoint?<><div className="selected-id">{selectedPoint.id}</div>
              <div className="hint">North {round(selectedPoint.north)} m · East {round(selectedPoint.east)} m · Alt {round(selectedPoint.altitude)} m</div>
              <label className="checkbox-row"><input type="checkbox" checked={selectedPoint.hard||selected===0||selected===project.points.length-1} disabled={selected===0||selected===project.points.length-1} onChange={e=>editPoint(selected,{...selectedPoint,hard:e.target.checked,exact:e.target.checked?false:selectedPoint.exact})}/> Required full stop</label>
              <label className="checkbox-row"><input type="checkbox" checked={selected!==0&&selected!==project.points.length-1&&!selectedPoint.hard&&selectedPoint.exact} disabled={selected===0||selected===project.points.length-1} onChange={e=>editPoint(selected,{...selectedPoint,exact:e.target.checked,hard:e.target.checked?false:selectedPoint.hard})}/> Exact point</label>
            </>:<div className="hint">Select a point on the map or in the table.</div>}
        </div>
        <div className="pane-resizer right-resizer" role="separator" aria-label="Resize properties inspector" aria-orientation="vertical" tabIndex={0}
          onKeyDown={e=>{if(e.key==='ArrowLeft')setPaneWidths(w=>({...w,right:Math.min(440,w.right+15)}));if(e.key==='ArrowRight')setPaneWidths(w=>({...w,right:Math.max(230,w.right-15)}));}}
          onPointerDown={e=>onResizeStart('right',e)} onPointerMove={onResizeMove} onPointerUp={onResizeEnd} onPointerCancel={onResizeEnd}/>

      </aside>
    </div>
    <footer className="statusbar" role="status">
      <span><span className={`status-indicator ${result?.accepted?'verified':''}`}/>{result?.accepted?'Reference verified':'No active verified result'}</span>
      <span className="activity-note">{notice||(active==='geometry'?(project.points.length<2?'Add at least two points to check the route.':drawingHints[tool]):'')}</span>
      <span className={`connection-status${sitl?' connected':''}`}><i aria-hidden="true"/>{sitl?'Simulator connected':engine==='unknown'?'Connecting to planner':'Planning mode'}</span>
    </footer>
    <WorkspaceHelp dialog={helpDialog} topic={helpTopic} onTopic={setHelpTopic}/>
  </div>;
}
