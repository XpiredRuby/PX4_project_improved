/* Drawing/plan comparison. Visual sampling never replaces the Python certificate. */
(function(root,factory){const api=factory();if(typeof module==='object'&&module.exports)module.exports=api;else root.PX4Compare=api;})(typeof window==='undefined'?globalThis:window,()=>{
 const colours={input:'#edc482',plan:'#79d5e7'};
 function nearest(point,polyline){
  let best={distance:Infinity,point:null};
  for(let i=1;i<polyline.length;i++){
   const a=polyline[i-1],b=polyline[i],dx=b[0]-a[0],dy=b[1]-a[1],den=dx*dx+dy*dy;
   const t=den?Math.max(0,Math.min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/den)):0;
   const q=[a[0]+t*dx,a[1]+t*dy],distance=Math.hypot(point[0]-q[0],point[1]-q[1]);
   if(distance<best.distance)best={distance,point:q};
  }
  return best;
 }
 function analyze(result){
  const input=(result.requested_intent||result.intent).path.points.map(p=>[p.east_m,p.north_m]);
  const planned=result.samples.map(p=>[p.y,p.x]);
  let worst={distance:0,point:planned[0],nearest:input[0]};
  for(const p of planned){const q=nearest(p,input);if(q.distance>worst.distance)worst={distance:q.distance,point:p,nearest:q.point};}
  const tolerance=(result.requested_intent||result.intent).path.maximum_deviation_m;
  const effectiveTolerance=result.intent.path.maximum_deviation_m;
  const check=result.verification?.checks?.find(c=>c.requirement_id==='INT-007');
  const effectiveBound=result.verification?.accepted&&check?.status==='PASS'&&check.units==='m'&&Number.isFinite(check.margin)&&check.margin>=-1e-9&&check.margin<=effectiveTolerance+1e-9?Math.max(0,effectiveTolerance-check.margin):null;
  const adjustment=result.planning_adjustments;
  // An effective-intent certificate cannot stand in for an original-request
  // certificate after a repair. Missing evidence must stay unavailable.
  const requestedBound=effectiveBound!==null&&adjustment&&Number.isFinite(adjustment.curve_error_bound_from_requested_m)&&adjustment.curve_error_bound_from_requested_m>=0?adjustment.curve_error_bound_from_requested_m:null;
  const bound=result.requested_intent||adjustment?requestedBound:effectiveBound;
  return {input,planned,worst,tolerance,bound,adjustment,fit:result.summary?.fidelity_policy,decimated:!!result.display_samples_decimated};
 }
 function line(ctx,data,map,colour,width,dashes=[]){if(data.length<2)return;ctx.strokeStyle=colour;ctx.lineWidth=width;ctx.setLineDash(dashes);ctx.beginPath();data.forEach((p,i)=>{const q=map(p);i?ctx.lineTo(...q):ctx.moveTo(...q);});ctx.stroke();}
 function drawOriginal(ctx,input,map,ui){
  if(api.view==='plan')return;
  ctx.save();ctx.lineCap='round';ctx.lineJoin='round';
  line(ctx,input,map,'#edc48235',12*ui);
  line(ctx,input,map,colours.input,1.8*ui,[5*ui,5*ui]);ctx.restore();
 }
 function drawPlanned(ctx,planned,map,ui){
  if(api.view==='input')return;
  ctx.save();ctx.lineCap='round';ctx.lineJoin='round';line(ctx,planned,map,'#0b1923',5*ui);line(ctx,planned,map,colours.plan,2.6*ui);ctx.restore();
 }
 function drawInset(canvas,comparison){
  const ctx=canvas.getContext('2d'),w=canvas.width,h=canvas.height,{input,planned,worst}=comparison;
  ctx.clearRect(0,0,w,h);ctx.fillStyle='#101c29';ctx.fillRect(0,0,w,h);
  const cx=(worst.point[0]+worst.nearest[0])/2,cy=(worst.point[1]+worst.nearest[1])/2;
  const verticalSpan=Math.max(1.2,worst.distance*8),scale=(h-54)/verticalSpan;
  const map=p=>[w/2+(p[0]-cx)*scale,h/2-(p[1]-cy)*scale];
  ctx.save();ctx.beginPath();ctx.rect(0,0,w,h);ctx.clip();drawOriginal(ctx,input,map,1);drawPlanned(ctx,planned,map,1);
  if(api.view==='overlay'&&worst.distance>1e-7){line(ctx,[worst.point,worst.nearest],map,'#e8eef3',1,[3,4]);for(const p of [worst.point,worst.nearest]){ctx.fillStyle='#e8eef3';ctx.beginPath();ctx.arc(...map(p),3,0,Math.PI*2);ctx.fill();}}
  ctx.restore();
  const metres=verticalSpan>=4?1:.25,x=18,y=h-18;
  ctx.fillStyle='#101c29dd';ctx.fillRect(10,h-48,metres*scale+25,42);ctx.strokeStyle='#b3c4d1';ctx.lineWidth=1.5;ctx.setLineDash([]);ctx.beginPath();ctx.moveTo(x,y);ctx.lineTo(x+metres*scale,y);ctx.stroke();ctx.font='11px system-ui';ctx.fillStyle='#b3c4d1';ctx.fillText(metres+' m',x,h-29);
 }
 let card,bar,inset,note,boundText,gapText,sourceLabel,data=null;
 function emit(){if(typeof window!=='undefined')window.dispatchEvent(new Event('route-comparison-change'));}
 function update(){
  if(!card)return;card.hidden=bar.hidden=!data;
  if(!data)return;
  gapText.textContent=data.worst.distance<.005?'Under 0.5 cm':(data.worst.distance*100).toFixed(1)+' cm';
  boundText.textContent=data.bound===null?'See plan report':'≤ '+(data.bound*100).toFixed(1)+' cm';
  note.textContent=data.worst.distance<1e-7?'The displayed routes overlap. A straight route may need timing changes without changing its shape.':'Dashed amber is your input. Blue is the planned route. The dotted link marks the largest gap among displayed plan samples.';
  document.getElementById('comparisonAllowance').textContent='Allowed rounding: '+(data.tolerance*100).toFixed(1)+' cm';
  const goal=document.getElementById('comparisonGoal'),pieces=data.fit?.pieces||[];
  if(pieces.length){const met=pieces.every(p=>p.soft_target_met),target=Math.min(...pieces.map(p=>p.soft_target_m)),achieved=Math.max(...pieces.map(p=>p.curve_bound_m));goal.textContent=(met?'Curve goal met. ':'Closer-fit goal not reached. ')+`Curve bound ≤ ${(achieved*100).toFixed(1)} cm; target ${(target*100).toFixed(1)} cm. The route bound above also includes interpolation. `+(met?'Tighter corners can mean slower flight.':'Within the hard limit. Mark a corner stop or revise the route for a closer fit.');goal.dataset.met=String(met);goal.hidden=false;}else goal.hidden=true;
  if(data.adjustment?.adjusted){goal.hidden=false;goal.dataset.met=String(data.adjustment.original_rounding_met);goal.textContent=data.adjustment.original_rounding_met?'The requested drawing stays within its rounding allowance. Stops and timing may change so it can be followed.':'The closest route found needs changes beyond the requested rounding allowance. Compare the original amber drawing with the blue route and review the adjustment summary.';}
  document.getElementById('comparisonSampleNote').textContent='The close-up uses displayed samples'+(data.decimated?' (reduced for display)':'')+'. Its gap is an estimate, not flight error. The proximity bound covers the curve and controller interpolation. A separate order-aware check rejects reordered paths, and a second spline evaluator checks emitted positions. Aircraft checks still share model primitives. The closer-fit goal applies to the continuous curve, including traversal order.';
  for(const button of bar.querySelectorAll('[data-view]'))button.setAttribute('aria-pressed',String(button.dataset.view===api.view));
  drawInset(inset,data);
 }
 function install(){
  const map=document.querySelector('.map-surface');if(!map)return;
  bar=document.createElement('div');bar.id='comparisonControls';bar.className='comparison-controls';bar.hidden=true;
  bar.innerHTML='<span class="comparison-title">Compare routes</span><div role="group" aria-label="Route comparison layers"><button data-view="overlay" aria-pressed="true">Overlay</button><button data-view="input" aria-pressed="false">Drawing only</button><button data-view="plan" aria-pressed="false">Plan only</button></div><button id="saveRoutePlot" class="save-plot">Save plot</button>';
  document.getElementById('zoomFit').textContent='Fit route';document.getElementById('zoomFit').setAttribute('aria-label','Fit original and planned routes');
  map.before(bar);bar.querySelectorAll('[data-view]').forEach(button=>button.onclick=()=>{api.view=button.dataset.view;update();emit();});
  card=document.createElement('section');card.id='routeComparison';card.className='comparison-card';card.hidden=true;
  card.innerHTML='<div class="section-heading"><h2>What changed?</h2><span class="comparison-zoom">Close-up</span></div><canvas id="comparisonInset" width="560" height="280" role="img" aria-label="Magnified input and planned route around the largest displayed gap"></canvas><div class="comparison-metrics"><div><strong id="comparisonGap">—</strong><span>Largest displayed gap</span></div><div><strong id="comparisonBound">—</strong><span>Verified deviation bound</span></div></div><p id="comparisonAllowance"></p><p id="comparisonGoal" class="fidelity-goal" role="status" hidden></p><p id="comparisonNote"></p><details><summary>How these numbers are measured</summary><p id="comparisonSampleNote"></p></details>';
  document.getElementById('planReview').before(card);inset=card.querySelector('canvas');gapText=document.getElementById('comparisonGap');boundText=document.getElementById('comparisonBound');note=document.getElementById('comparisonNote');
  const legend=document.querySelector('.legend');legend.innerHTML='<span class="input-shadow">Your drawing</span><span class="planned">Planned route</span>';
  sourceLabel=legend.querySelector('.input-shadow');
  document.getElementById('saveRoutePlot').onclick=()=>{
   if(!data)return;const canvas=document.createElement('canvas');canvas.width=900;canvas.height=704;const ctx=canvas.getContext('2d');ctx.fillStyle='#101c29';ctx.fillRect(0,0,900,704);ctx.drawImage(document.getElementById('route'),0,0);ctx.font='14px system-ui';ctx.fillStyle=colours.input;ctx.fillText('Amber shadow: original input',24,656);ctx.fillStyle=colours.plan;ctx.fillText('Blue: planned route',350,656);ctx.fillStyle='#afc1ce';ctx.font='12px system-ui';ctx.fillText('View: '+api.view+' · Planning comparison only. Gazebo has not run.',24,683);
   canvas.toBlob(blob=>{if(!blob)return;const url=URL.createObjectURL(blob),link=document.createElement('a');link.href=url;link.download='trajectory-comparison.png';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);},'image/png');
  };
  update();
 }
 const api={view:'overlay',analyze,nearest,drawOriginal,drawPlanned,drawInset,
  setResult(result,source){data=analyze(result);api.view='overlay';if(sourceLabel)sourceLabel.textContent=source==='drawing'?'Your drawing':'Input shape';update();},
  clear(){data=null;api.view='overlay';update();}
 };
 if(typeof document!=='undefined')document.addEventListener('DOMContentLoaded',install);
 return api;
});
