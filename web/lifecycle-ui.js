/* Full mission is a reference plan. Route CSV remains the local runner artifact. */
(()=>{
 let panel,data;
 const $=id=>document.getElementById(id);
 window.PX4MissionView={
  clear(){data=null;if(panel)panel.hidden=true;},
  show(result){
   data=result;const mission=result.mission;if(!panel||!mission)return;
   panel.hidden=false;
   $('missionGroundSequence').textContent=`Ground 0 m → Takeoff ${(-result.intent.path.points[0].down_m).toFixed(1)} m → Ground 0 m`;
   const names={TAKEOFF:'Takeoff',ROUTE:'Route',TURN_HOME:'Turn home',RETURN_HOME:'Return',LAND:'Land'};
   $('missionPhases').replaceChildren();
   for(const phase of mission.phases){const item=document.createElement('span');item.textContent=`${names[phase.name]} · ${phase.duration_s.toFixed(1)} s`;$('missionPhases').append(item);}
   chart('missionHeight',mission.samples.map(p=>[p.time,p.height_m]));
   $('duration').textContent=mission.total_duration_s.toFixed(1)+' s';
   $('duration').nextElementSibling.textContent='Full mission duration';
   $('reviewSummary').textContent=`Starts stationary at the ground origin, takes off to ${(-result.intent.path.points[0].down_m).toFixed(1)} m, follows the route (${mission.route_duration_s.toFixed(1)} s), returns to launch and descends to 0 m. Full planned mission: ${mission.total_duration_s.toFixed(1)} s. This is a reference plan; Gazebo has not run.`;
   const budget=mission.energy_budget;
   $('aircraftSummary').textContent=budget?`${result.aircraft_setup.profile.name} · Full mission budget ${budget.total_budgeted_s.toFixed(1)} s including settling and contingency, ${budget.upper_bound_energy_wh.toFixed(1)} Wh upper bound, ${(budget.reserve_fraction*100).toFixed(0)}% reserve. Declared aircraft assumptions apply.`:'Using the standard virtual drone. Route and added takeoff/return/descent phases have model checks. Actual touchdown and motor shutdown require a Gazebo run.';
   $('headingPolicy').textContent=result.intent.motion.yaw_mode==='fixed'?`Fixed heading: ${result.intent.motion.fixed_yaw_deg.toFixed(1)}°. Zero means holding the launch reference heading; it does not mean heading is missing.`:'Heading follows the route; takeoff holds the first heading, then the mission turns toward home before returning and landing.';
  }
 };
 document.addEventListener('DOMContentLoaded',()=>{
  panel=document.createElement('section');panel.className='mission-card';panel.id='fullMissionView';panel.hidden=true;
  panel.innerHTML='<h2>Full flight · ground to ground</h2><p id="missionGroundSequence" class="ground-sequence"></p><canvas id="missionHeight" width="560" height="240" role="img" aria-label="Full mission height profile starting at ground zero, rising to route height and descending back to ground zero"></canvas><div id="missionPhases" class="mission-phases"></div><p id="headingPolicy"></p><details><summary>Coordinates and landing assumptions</summary><p>Height is positive above launch. CSV z uses NED: ground 0, 20 m high is z = −20, landed 0. The export also includes positive height_m and phase columns.</p><p>Takeoff speed ≤0.8 m/s, descent ≤0.3 m/s, return ≤1 m/s, with additional planner margin and any tighter aircraft limits. The reference assumes a flat launch/landing surface. Reaching planned height 0 does not prove touchdown, contact, disarm or motor shutdown. Local Gazebo uses its existing controlled takeoff/landing flow.</p></details>';
  const comparison=$('routeComparison');(comparison||$('planReview')).before(panel);
  $('downloadCsv').textContent='Full mission CSV';$('downloadCsv').onclick=()=>{if(data?.mission)download('full_mission_ned.csv',data.mission.trajectory_csv,'text/csv');};
  const routeDownload=document.createElement('button');routeDownload.id='downloadRouteCsv';routeDownload.textContent='Route-only CSV (local SITL)';routeDownload.onclick=()=>{if(data)download('route_only_sitl.csv',data.trajectory_csv,'text/csv');};
  const details=document.createElement('details');details.className='route-export';const summary=document.createElement('summary');summary.textContent='Developer route export';details.append(summary,routeDownload);$('results').append(details);
  $('downloadEvidence').onclick=()=>{if(!data)return;const {trajectory_csv,...report}=data;const {trajectory_csv:missionCsv,...mission}=data.mission;download('mission_plan_and_checks.json',JSON.stringify({...report,mission},null,2),'application/json');};
 });
})();
