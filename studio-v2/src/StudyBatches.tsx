import {useEffect, useRef, useState} from 'react';
import {buildRequest, exportBlob, projectFromJson} from './engineering';
import type {Project} from './types';

type Entry = {id:string; title:string; created:number; total:number; finished:number};
type Case = {number:number; speed:number; state:string; attempts:number; error:string|null;
  metrics:{peak_m_s:number; route_s:number; peak_airspeed_m_s:number|null; limits:string[]}|null};
type Study = {id:string; title:string; cases:Case[]; snapshot?:{project:Project; speeds:number[]}};
async function api(path:string, body?:unknown) {
  const res=await fetch('/api/studies'+path,body===undefined?{cache:'no-store'}:{
    method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const value=await res.json();
  if(!res.ok)throw Error(value.error||'Study request failed');
  return value;
}

export default function StudyBatches({enabled,project,restore}:{enabled:boolean;project:Project;restore:(p:Project)=>void}){
  const [speeds,setSpeeds]=useState('1, 5, 25, 50');
  const [entries,setEntries]=useState<Entry[]>([]),[selected,setSelected]=useState('');
  const [study,setStudy]=useState<Study|null>(null),[error,setError]=useState(''),[busy,setBusy]=useState(false);
  const [revision,refresh]=useState(0);
  const submission=useRef({content:'',id:''});
  useEffect(()=>{
    if(!enabled)return;
    let stopped=false,timer:ReturnType<typeof setTimeout>;
    async function poll(){
      let interval=6000;
      try{
        const list:Entry[]=(await api('')).studies;
        if(list.some(v=>v.finished<v.total))interval=1000;
        const id=selected||list[0]?.id;
        const detail=id?await api('/'+id):null;
        if(!stopped){setEntries(list);setStudy(detail);if(!selected&&id)setSelected(id);}
      }catch(e){if(!stopped)setError((e as Error).message);}
      if(!stopped)timer=setTimeout(poll,interval);
    }
    poll();
    return ()=>{stopped=true;clearTimeout(timer);};
  },[enabled,selected,revision]);
  async function act(fn:()=>Promise<void>){
    setBusy(true);setError('');
    try{await fn();refresh(n=>n+1);}catch(e){setError((e as Error).message);}finally{setBusy(false);}
  }
  function submit(){
    return act(async()=>{
      const values=speeds.split(',').map(Number);
      if(values.some(v=>!Number.isFinite(v)||v<=0))throw Error('Enter comma-separated positive speeds.');
      const payload={project,request:buildRequest(project),speeds:values},content=JSON.stringify(payload);
      if(submission.current.content!==content)submission.current={content,id:crypto.randomUUID()};
      const next:Study=await api('',{...payload,id:submission.current.id});
      setStudy(next);setSelected(next.id);submission.current.content='';
    });
  }
  const save=(name:string,value:unknown)=>exportBlob(name,JSON.stringify(value,null,2),'application/json');
  const current=study?.id===selected?study:null;
  return <section className="data-section"><h3>Unattended studies</h3>
    {!enabled?<p>For background studies, start the downloaded studio with <code>python start_studio.py</code>.</p>:<>
      <p>Queue the current geometry, aircraft and weather together. The local server continues after this tab closes and resumes unfinished cases after restart.</p>
      <label>Batch speeds (m/s)<input value={speeds} onChange={e=>setSpeeds(e.target.value)}/></label>
      <button disabled={busy||project.points.length<2} onClick={submit}>Queue study</button>
      {entries.length>0&&<label>Saved studies <select value={selected} onChange={e=>setSelected(e.target.value)}>
        {entries.map(v=><option key={v.id} value={v.id}>{v.title} · {new Date(v.created*1000).toLocaleString()} · {v.finished}/{v.total}</option>)}
      </select></label>}
      {current&&<><table className="point-table"><thead><tr>{['Requested (m/s)','State','Peak (m/s)','Route (s)','Case result'].map(s=><th key={s}>{s}</th>)}</tr></thead>
        <tbody>{current.cases.map(c=><tr key={c.number}><td>{c.speed}</td><td>{c.state}</td><td>{c.metrics?.peak_m_s.toFixed(3)??'—'}</td>
          <td>{c.metrics?.route_s.toFixed(2)??'—'}</td><td>{c.metrics?<button onClick={()=>act(async()=>{
            save('study-'+selected+'-case-'+(c.number+1)+'.json',await api('/'+selected+'/cases/'+c.number));})}>Download case {c.number+1}</button>:c.error}</td></tr>)}</tbody></table>
        <button disabled={busy||!current.cases.some(c=>['queued','running'].includes(c.state))} onClick={()=>act(async()=>{await api('/'+selected+'/cancel',{});})}>Cancel unfinished</button>{' '}
        <button disabled={busy||!current.cases.some(c=>['failed','timed_out','cancelled'].includes(c.state))} onClick={()=>act(async()=>{await api('/'+selected+'/retry',{});})}>Retry unfinished</button>{' '}
        <button disabled={busy} onClick={()=>act(async()=>{save('study-'+selected+'.json',await api('/'+selected+'/export'));})}>Export study</button>{' '}
        <button disabled={busy} onClick={()=>act(async()=>{const v=await api('/'+selected+'/export');restore(projectFromJson(v.snapshot.project));})}>Restore scenario</button>
      </>}
      {error&&<p role="alert">{error}</p>}
    </>}
  </section>;
}
