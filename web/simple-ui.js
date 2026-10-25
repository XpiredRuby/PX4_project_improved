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
  if(value?.version!==1||!['drawing','interview'].includes(value.mode)||!['stroke','points'].includes(value.tool)||
   !route||!Array.isArray(route.points)||!Array.isArray(route.kinds)||route.points.length!==route.kinds.length||
   !['open_stop','periodic'].includes(route.closure)||route.kinds.some(k=>!['ordinary','stop','exact'].includes(k))||
   route.points.some(p=>!Array.isArray(p)||p.length!==2||p.some(n=>typeof n!=='number'||!Number.isFinite(n)))||
   !value.fields||typeof value.fields!=='object'||Array.isArray(value.fields))throw Error('Invalid draft');
  const fields={};
  for(const id of FIELDS){const field=value.fields[id];if(typeof field!=='string'||field.length>(id==='waypoints'?10000:80))throw Error('Invalid saved setting');fields[id]=field;}
  draft.validate(route.points,route.kinds,route.closure);
  return {version:1,mode:value.mode,tool:value.tool,route:{points:route.points,kinds:route.kinds,closure:route.closure},fields};
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
 function save(){
  if(!initialized||!root.PX4Editor||root.PX4Editor.draft.gesture||root.PX4Editor.draft.pending)return;
  try{
   const state=root.PX4Editor.snapshot();state.version=1;
   state.fields=Object.fromEntries(FIELDS.map(id=>[id,$(id).value]));
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
  if(pace){$('speedBound').textContent=pace.planned_route_peak_sampled_m_s.toFixed(2)+' m/s';
   $('speedBound').nextElementSibling.textContent='Planned route peak · sampled';
   const requested=pace.requested_cruise_m_s===null?'automatic':pace.requested_cruise_m_s.toFixed(2)+' m/s';
   $('paceExplanation').textContent='Requested cruise '+requested+' · effective cap '+pace.planning_cap_m_s.toFixed(2)+' m/s · planned mean '+pace.planned_route_mean_sampled_m_s.toFixed(2)+' m/s. '+pace.semantics+' A 10% planning margin applies. This is reference motion; no measured flight is loaded.';
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
  document.addEventListener('input',e=>{if(FIELDS.includes(e.target.id))api.saveSoon();});
  document.addEventListener('change',e=>{if(FIELDS.includes(e.target.id))api.saveSoon();});
  root.addEventListener('pagehide',save);document.addEventListener('visibilitychange',()=>{if(document.visibilityState==='hidden')save();});
  domReady=true;attachStorage();
 });
})(typeof window!=='undefined'?window:globalThis);
