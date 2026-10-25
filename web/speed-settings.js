/* Explicit user speed policy; it never raises an aircraft's declared limits. */
(function(root,factory){const api=factory(root);if(typeof module==='object'&&module.exports)module.exports=api;else root.PX4SpeedSettings=api;})(typeof window==='undefined'?globalThis:window,(root)=>{
 'use strict';
 function policy(target,custom,aircraft,mode='aircraft'){
  if(!['aircraft','custom'].includes(mode))throw Error('Choose an aircraft or custom speed cap.');
  for(const [value,label] of [[target,'Requested cruise'],[aircraft,'Aircraft speed ceiling'],...(mode==='custom'?[[custom,'Never exceed']]:[])])if(!Number.isFinite(value)||value<=0)throw Error(label+' must be a finite number greater than zero.');
  const cap=mode==='aircraft'?aircraft:custom,planning=.9*Math.min(target,cap,aircraft);
  return {cap,planning,conflict:mode==='custom'&&target>cap,
   message:`Requested cruise ${target.toFixed(2)} m/s · ${mode==='aircraft'?'aircraft':'custom'} cap ${cap.toFixed(2)} m/s · planning cap ${planning.toFixed(2)} m/s after the 10% margin. Turns and stops may reduce the achieved speed.`};
 }
 const api={policy};
 if(typeof document==='undefined')return api;
 const $=id=>document.getElementById(id);let legacy=false;
 function state(){return policy(Number($('targetSpeed').value),Number($('maxSpeed').value),root.PX4Editor?.aircraftSpeedLimit()||3,$('speedCapMode').value);}
 api.refresh=()=>{
  const mode=$('speedCapMode').value,automatic=mode==='aircraft';
  $('maxSpeed').readOnly=automatic;
  if(automatic)$('maxSpeed').value=String(root.PX4Editor?.aircraftSpeedLimit()||3);
  try{const s=state();$('speedInputHint').textContent=s.message+(s.conflict?' Resolve the custom cap conflict in Advanced settings before planning.':'');$('speedInputHint').classList.toggle('speed-warning',s.conflict);}
  catch(error){$('speedInputHint').textContent=error.message;$('speedInputHint').classList.add('speed-warning');}
  $('legacySpeedNotice').hidden=!legacy;
 };
 api.validate=()=>{api.refresh();const s=state();if(s.conflict)throw Error(`Requested cruise exceeds your custom Never exceed setting (${s.cap} m/s). Open Advanced settings: select Aircraft model cap, raise your custom cap, or lower requested cruise.`);return s;};
 api.restore=saved=>{legacy=saved.version===1;$('speedCapMode').value=legacy?'custom':saved.speed_cap_mode;api.refresh();};
 document.addEventListener('DOMContentLoaded',()=>{
  $('speedCapMode').addEventListener('input',()=>{legacy=false;api.refresh();});
  $('useAircraftSpeed').onclick=()=>{legacy=false;$('speedCapMode').value='aircraft';$('speedCapMode').dispatchEvent(new Event('input',{bubbles:true}));};
  document.addEventListener('input',e=>{if(['targetSpeed','maxSpeed','aircraftMode'].includes(e.target.id)||e.target.id.startsWith('aircraft-'))api.refresh();});
  const build=document.querySelector('meta[name="px4-build"]')?.content;if(build){const label=document.createElement('span');label.id='buildIdentity';label.textContent=' · Build '+build;document.querySelector('footer').append(label);}
  api.refresh();
 });
 return api;
});
