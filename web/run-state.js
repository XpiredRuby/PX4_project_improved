/* Explicit planner states. Progress is stage-based; elapsed time is not a percent. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PX4RunState=api;})(typeof window==='undefined'?globalThis:window,()=>{
 'use strict';
 const active=['starting','loading','checking','stale'];
 const labels={draft:'Draft',starting:'Starting',loading:'Preparing engine',checking:'Checking',stale:'Drawing changed',verified:'Model checked',adjusted:'Adjusted reference',cancelled:'Cancelled',error:'Needs attention'};
 function next(state,event){
  if(!Object.hasOwn(labels,event))throw Error('Unknown planner state');
  if(['loading','checking'].includes(event)&&!['starting','loading','checking'].includes(state))return state;
  if(['verified','adjusted','error','cancelled'].includes(event)&&!active.includes(state))return state;
  return event;
 }
 return {next,labels,active};
});
