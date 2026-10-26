/* Compact results and device-local draft recovery; planner acceptance is unchanged. */
(function(root){
 'use strict';
 const KEY='px4-studio-draft-v1';
 const FIELDS=['altitude','deviation','targetSpeed','maxSpeed','yaw','fixedYaw','deadline','pattern','radius','laps','length','width','spacing','waypoints'];
 function duration(value){
  if(!Number.isFinite(value)||value<0)return '—';
  if(value<60)return (value<10?value.toFixed(1):Math.round(value))+' sec';
  const seconds=Math.round(value),minutes=Math.floor(seconds/60),rest=seconds%60;
  return minutes+' min'+(rest?' '+rest+' sec':'');
 }
 function decode(raw,draft){
  if(typeof raw!=='string'||raw.length>2*1024*1024)throw Error('Draft is too large');
  const value=JSON.parse(raw),route=value?.route;
  if(![1,2].includes(value?.version)||!['drawing','interview'].includes(value.mode)||!['stroke','points'].includes(value.tool)||
   !route||!Array.isArray(route.points)||!Array.isArray(route.kinds)||route.points.length!==route.kinds.length||
   !['open_stop','periodic'].includes(route.closure)||route.kinds.some(k=>!['ordinary','stop','exact'].includes(k))||
   route.points.some(p=>!Array.isArray(p)||p.length!==2||p.some(n=>typeof n!=='number'||!Number.isFinite(n)))||
   !value.fields||typeof value.fields!=='object'||Array.isArray(value.fields))throw Error('Invalid draft');
  const fields={};
  for(const id of FIELDS){const field=value.fields[id];if(typeof field!=='string'||field.length>(id==='waypoints'?10000:80))throw Error('Invalid saved setting');fields[id]=field;}
  for(const id of ['altitude','targetSpeed','maxSpeed','radius','laps','length','width','spacing'])if(!fields[id].trim()||!Number.isFinite(Number(fields[id]))||Number(fields[id])<=0)throw Error('Invalid saved '+id);
  for(const id of ['deviation','fixedYaw'])if(!fields[id].trim()||!Number.isFinite(Number(fields[id]))||(id==='deviation'&&Number(fields[id])<0))throw Error('Invalid saved '+id);
  if(fields.deadline.trim()&&(!Number.isFinite(Number(fields.deadline))||Number(fields.deadline)<=0))throw Error('Invalid saved deadline');
  if(!Number.isInteger(Number(fields.laps)))throw Error('Saved laps must be an integer');
  draft.validate(route.points,route.kinds,route.closure);
  if(value.version===2&&!['aircraft','custom'].includes(value.speed_cap_mode))throw Error('Invalid speed policy');
  return {version:value.version,...(value.version===2?{speed_cap_mode:value.speed_cap_mode}:{}),mode:value.mode,tool:value.tool,route:{points:route.points,kinds:route.kinds,closure:route.closure},fields};
 }
 function errorMessage(result){
  const message=String(result?.error||'');
  if(result?.limiting_constraint==='duration')return 'The route time limit is too short. '+(Number.isFinite(result.minimum_feasible_duration_s)?'This candidate needs at least '+duration(result.minimum_feasible_duration_s)+'. ':'')+'Open Flight settings and allow more time, or leave the time limit blank.';
  if(/exceeded 120|timed? ?out/i.test(message))return 'This route took too long to check. Use Points & stops to reduce the number of points, then check it again.';
  if(/radius|outside.*envelope|too far|within .*m of START/i.test(message))return 'Part of the route is outside the allowed area. Move those points closer to START, then check it again.';
  if(/too many points/i.test(message))return 'The route has too many points. Try Place points to make a simpler route.';
  if(/soft.*target|fidelity|rounding|corner|curvature/i.test(message))return 'The planner could not follow this shape within the chosen limits. Mark a sharp corner as Stop here in Points & stops, or revise the route, then check it again.';
  if(/fetch|network|load failed|offline/i.test(message))return 'The planner could not be reached. Check your connection, then try again. Your drawing is still here.';
  if(message&&!/traceback|numpy|spline|retim|NaN|assert|exception|solver|infeasible|ValueError|TypeError/i.test(message))return message;
  return 'This route could not be checked with the current settings. Review Points & stops and Flight settings, then try again. The technical details are available below.';
 }
 const api={duration,decode,errorMessage,fields:FIELDS};
 if(typeof module!=='undefined'&&module.exports)module.exports=api;
 if(!root.document)return;
 root.PX4Simple=api;
 const $=id=>document.getElementById(id);
 let initialized=false,domReady=false,settingsReady=false,timer=null,stamp=null,actions=null;
 const make=(tag,cls,text)=>{const el=document.createElement(tag);if(cls)el.className=cls;if(text)el.textContent=text;return el;};
 function attachStorage(){
  if(initialized||!domReady||!settingsReady||!root.PX4Editor)return;
  try{
   const raw=localStorage.getItem(KEY);
   if(raw&&root.PX4Editor.canRestore()){const saved=decode(raw,root.PX4Editor.draft);for(const id of FIELDS){const el=$(id);if(!el||el.tagName==='SELECT'&&![...el.options].some(o=>o.value===saved.fields[id]))throw Error('Unsupported setting');}
    root.PX4Editor.restore(saved);stamp.textContent='Draft restored · check it again';}
   else stamp.textContent=raw?'Current edits kept · draft will save here':'Draft saves on this device';
  }catch(e){stamp.textContent='Draft recovery unavailable · keep this page open';}
  initialized=true;
  if(!root.PX4Editor.canRestore())api.saveSoon();
 }
 api.settingsReady=()=>{settingsReady=true;attachStorage();};
 api.saveSoon=()=>{
  if(!initialized)return;
  clearTimeout(timer);timer=setTimeout(save,350);
 };
 function project(){const state=root.PX4Editor.snapshot();state.version=2;state.speed_cap_mode=$('speedCapMode').value;state.fields=Object.fromEntries(FIELDS.map(id=>[id,$(id).value]));decode(JSON.stringify(state),root.PX4Editor.draft);return state;}
 function save(){
  if(!initialized||!root.PX4Editor||root.PX4Editor.draft.gesture||root.PX4Editor.draft.pending)return;
  try{
   const state=project();
   const raw=JSON.stringify(state);decode(raw,root.PX4Editor.draft);
   localStorage.setItem(KEY,raw);stamp.textContent='Draft saved on this device';
  }catch(e){stamp.textContent='Draft not saved · keep this page open';}
 }
 function wrap(title,child){
  const details=make('details','review-details');details.append(make('summary','',title),child);return details;
 }
 api.clear=()=>{if(actions)actions.hidden=true;};
 api.showError=message=>{if(actions)actions.hidden=false;};
 api.show=result=>{
  if(!domReady)return;
  const mission=result.mission,rt=result.summary.retiming,pieces=result.summary.fidelity_policy?.pieces||[];
  const adaptation=result.planning_adjustments;
  const met=adaptation&&!adaptation.original_rounding_met?false:pieces.length?pieces.every(p=>p.soft_target_met):null;
  $('duration').textContent=duration(mission?.total_duration_s??rt.selected_duration_s);
  $('simpleGoal').textContent=met===true?'Closer-fit goal met':met===false?'Within the limit · closer fit needs work':'Route checks passed';
  if(adaptation?.adjusted)$('simpleGoal').textContent=adaptation.original_rounding_met?'Feasible route · within requested fit':'Feasible route · drawing adjusted';
  $('simpleGoal').dataset.met=String(met);
  $('reviewSummary').textContent=mission?'Ground → '+(-result.intent.path.points[0].down_m).toFixed(1)+' m → your route → home → ground.':'Follow the checked route shown in blue.';
  const label={speed:'speed limit',acceleration:'acceleration limit',jerk:'limit on sudden changes',tilt:'tilt limit',thrust:'motor thrust limit',duration:'route time limit',stop_and_turn:'stop-and-turn motion'}[rt.limiting_constraint]||'motion limits';
  $('paceExplanation').textContent='The '+label+' sets the pace for this plan. Slowing at turns helps keep the route close to your drawing. '+(mission?'The route itself takes '+duration(mission.route_duration_s)+'. Takeoff, return and landing make up the rest.':'');
  const pace=result.summary.speed_preferences;
  if(pace){$('speedSummary').textContent='Requested '+(pace.requested_cruise_m_s??'automatic')+' m/s · planned peak '+pace.planned_route_peak_sampled_m_s.toFixed(2)+' m/s (sampled). Planning cap '+pace.planning_cap_m_s.toFixed(2)+' m/s, including the 10% margin. Turns and stops can reduce speed.';
   $('speedBound').textContent=pace.planned_route_peak_sampled_m_s.toFixed(2)+' m/s';
   $('speedBound').nextElementSibling.textContent='Planned route peak · sampled';
   const requested=pace.requested_cruise_m_s===null?'automatic':pace.requested_cruise_m_s.toFixed(2)+' m/s';
   $('paceExplanation').textContent='Requested cruise '+requested+' · effective cap '+pace.planning_cap_m_s.toFixed(2)+' m/s · planned mean '+pace.planned_route_mean_sampled_m_s.toFixed(2)+' m/s. '+pace.semantics+' A 10% planning margin applies. This is reference motion; no measured flight is loaded.';
   if(Number.isFinite(pace.stationary_time_sampled_s))$('speedSummary').textContent+=' Route '+pace.route_distance_sampled_m.toFixed(1)+' m in '+duration(pace.route_duration_s)+'. Stationary '+duration(pace.stationary_time_sampled_s)+' (including '+duration(pace.stationary_turn_time_sampled_s)+' turning), estimated from full-resolution reference samples.';
   if(pace.reasons.includes('requested_speed_cap'))$('paceExplanation').textContent+=' Your Never exceed setting limits the requested cruise. Change it in Flight settings if intended.';
   if(pace.reasons.includes('aircraft_speed_cap'))$('paceExplanation').textContent+=' The selected aircraft ceiling is '+pace.aircraft_maximum_m_s.toFixed(2)+' m/s.';
  }
  $('status').textContent='Plan checks passed. Gazebo has not run.'+(met===false?' For a closer fit, mark a sharp corner as a stop or revise the route.':'');
  if(mission){[...$('missionPhases').children].forEach((item,i)=>{const phase=mission.phases[i];item.textContent=item.textContent.split(' · ')[0]+' · '+duration(phase.duration_s);});}
  actions.hidden=true;
 };
 document.addEventListener('DOMContentLoaded',()=>{
  const results=$('results');if(!results||!document.querySelector('.review-rail'))return;
  const metrics=results.querySelector('.metrics');metrics.classList.add('compact-metrics');
  const motion=make('div','motion-details');for(const id of ['binding','speedBound','tiltBound'])motion.append($(id).closest('.metric'));
  const pace=make('p','muted');pace.id='paceExplanation';motion.append(pace);
  const goal=make('p','compact-goal');goal.id='simpleGoal';metrics.append(goal);
  const speedSummary=make('p','speed-summary');speedSummary.id='speedSummary';metrics.after(speedSummary);
  const flight=wrap('Flight stages and heading',$('fullMissionView')),comparison=wrap('Compare drawing and planned route',$('routeComparison'));
  const more=wrap('What sets the pace?',motion);
  const downloadBox=make('details','download-menu'),list=make('div','download-options');
  downloadBox.append(make('summary','','Download'),list);
  const oldDownloads=$('downloadCsv').parentElement;
  for(const [id,label] of [['downloadCsv','Flight data (.csv)'],['saveRoutePlot','Route image (.png)'],['downloadEvidence','Plan and checks (.json)']]){
   const button=$(id);button.textContent=label;button.classList.remove('save-plot');button.addEventListener('click',()=>downloadBox.open=false);list.append(button);
  }
  oldDownloads.remove();
  results.insertBefore(downloadBox,results.querySelector('details'));
  results.append(flight,comparison,more);
  const advanced=make('details','review-details engineering-details');advanced.append(make('summary','','Engineering details'));
  for(const child of [...results.children]){if(child.tagName==='DETAILS'&&![flight,comparison,more,downloadBox,advanced].includes(child))advanced.append(child);}
  results.append(advanced);
  for(const card of [$('fullMissionView'),$('routeComparison')])card.classList.add('nested-result');
  actions=make('div','error-actions');actions.hidden=true;
  for(const [label,target] of [['Edit points & stops','openPoints'],['Edit flight settings','openSettings']]){const button=make('button','text-button',label);button.onclick=()=>$(target).click();actions.append(button);}
  $('status').after(actions);
  stamp=make('p','draft-save-status');stamp.id='draftSaveStatus';stamp.setAttribute('role','status');document.querySelector('.map-card').append(stamp);
  document.addEventListener('input',e=>{if(FIELDS.includes(e.target.id)||e.target.id==='speedCapMode')api.saveSoon();});
  document.addEventListener('change',e=>{if(FIELDS.includes(e.target.id)||e.target.id==='speedCapMode')api.saveSoon();});
  root.addEventListener('pagehide',save);document.addEventListener('visibilitychange',()=>{if(document.visibilityState==='hidden')save();});
  const projects=make('section','project-controls');projects.append(make('h3','','Route projects'),make('p','muted','Export editable geometry and settings, or import a project from another device. Imported projects must be checked again; certificates and simulation authority are never restored.'));
  const exportButton=make('button','','Export editable project');exportButton.id='exportProject';
  const importLabel=make('label','','Import route project (.json)');importLabel.htmlFor='importProject';const importFile=make('input');importFile.type='file';importFile.accept='.json,application/json';importFile.id='importProject';
  const projectStatus=make('p','muted');projectStatus.id='projectStatus';projectStatus.setAttribute('role','status');
  exportButton.onclick=()=>{try{const raw=JSON.stringify(project(),null,2),url=URL.createObjectURL(new Blob([raw],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='trajectory-project.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);projectStatus.textContent='Editable project exported. Verify again after import.';}catch(error){projectStatus.textContent=error.message;}};
  importFile.onchange=async()=>{try{const file=importFile.files[0];if(!file)return;if(file.size>2*1024*1024)throw Error('Use a project under 2 MB.');const signature=()=>JSON.stringify({route:root.PX4Editor.snapshot(),fields:FIELDS.map(id=>$(id).value),policy:$('speedCapMode').value}),before=signature(),saved=decode(await file.text(),root.PX4Editor.draft);if(before!==signature())throw Error('Drawing changed during import. Select the project again to apply it.');for(const id of FIELDS){const el=$(id);if(el.tagName==='SELECT'&&![...el.options].some(o=>o.value===saved.fields[id]))throw Error('Unsupported saved '+id);}root.PX4Editor.restore(saved);projectStatus.textContent='Project imported. Review settings and plan again.';}catch(error){projectStatus.textContent='Import failed: '+error.message;}finally{importFile.value='';}};
  projects.append(exportButton,importLabel,importFile,projectStatus);($('panel-advanced')||$('flightSettings')).append(projects);
  domReady=true;attachStorage();
 });
})(typeof window!=='undefined'?window:globalThis);
