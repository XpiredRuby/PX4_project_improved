/* Presentation layer: retained controls keep the existing draft/compiler behavior. */
(()=>{
 const byId=id=>document.getElementById(id);
 const node=(tag,cls,text)=>{const el=document.createElement(tag);if(cls)el.className=cls;if(text)el.textContent=text;return el;};
 const icon='<svg viewBox="0 0 32 32" aria-hidden="true"><path d="M6 24V10c0-3 4-5 7-2l7 14c2 4 6 2 6-1V7" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round"/><circle cx="6" cy="24" r="3" fill="currentColor"/><circle cx="26" cy="7" r="3" fill="currentColor"/></svg>';
 const originalMain=document.querySelector('main'),aside=originalMain.querySelector('aside');
 const model=aside.children[0],flight=byId('flightSettings'),compilePanel=byId('compile').closest('section');
 const workspace=document.querySelector('.workspace'),review=byId('planReview');
 const simulation=byId('rehearse').closest('section'),steps=workspace.querySelector('.steps');
 const header=document.querySelector('header');header.className='app-header';header.replaceChildren();
 const brand=node('a','brand');brand.href='/';brand.setAttribute('aria-label','Trajectory Studio home');brand.innerHTML=icon+'<span>Trajectory<span class="brand-secondary"> Studio</span></span>';
 const headerRight=node('div','header-actions');
 const scope=node('span','scope-chip','Browser workspace');
 const settingsButton=node('button','header-button','Settings');settingsButton.id='openSettings';
 headerRight.append(scope,settingsButton);header.append(brand,headerRight);
 const main=node('main','studio-layout');const editor=node('section','map-card');editor.setAttribute('aria-label','Route workspace');
 const heading=node('div','workspace-heading');
 const headingText=node('div');headingText.append(node('p','overline','ROUTE BUILDER'),node('h1','','Plan your route'),node('p','workspace-description','Draw anything, even your name. We’ll find a feasible route and show what changed.'));
 const exampleMenu=node('details','example-menu');exampleMenu.append(node('summary','','Examples'));
 const exampleContent=node('div','example-list');const examples=byId('quickExamples');for(const b of [...examples.querySelectorAll('button')]){exampleContent.append(b);b.addEventListener('click',()=>exampleMenu.open=false);}exampleMenu.append(exampleContent);heading.append(headingText,exampleMenu);
 const toolbar=byId('routeToolbar');toolbar.className='editor-toolbar';
 byId('strokeTool').textContent='Draw';byId('pointTool').textContent='Place points';byId('clear').textContent='Clear';byId('clear').title='Clear the route (can be undone)';
 const pointButton=node('button','subtle-button','Points & stops');pointButton.id='openPoints';
 toolbar.append(pointButton);
 const canvasWrap=node('div','map-surface');const guide=byId('drawingGuide');guide.className='map-hint';guide.querySelector('.launch-icon')?.remove();guide.querySelector('strong').textContent='Draw anywhere';
 const canvasFrame=byId('route').parentElement;canvasFrame.className='canvas-frame';
 const mapControls=workspace.querySelector('.map-view');mapControls.className='map-controls';mapControls.querySelector('span')?.remove();byId('zoomIn').textContent='+';byId('zoomOut').textContent='−';byId('zoomFit').textContent='Fit';
 canvasWrap.append(canvasFrame,guide,mapControls);
 const caption=node('div','map-caption');const legend=workspace.querySelector('.legend');legend.querySelector('.flown')?.remove();caption.append(legend,byId('pointCount'));
 editor.append(heading,toolbar,byId('drawingError'),byId('pendingStroke'),canvasWrap,caption);
 const rail=node('aside','review-rail');
 const config=node('section','route-summary');const configHeading=node('div','section-heading');configHeading.append(node('h2','','Route settings'));const editSettings=node('button','text-button','Edit');configHeading.append(editSettings);
 const configMetrics=node('div','config-metrics');
 const heightButton=node('button','config-value'),speedButton=node('button','config-value');heightButton.id='heightShortcut';speedButton.id='speedShortcut';configMetrics.append(heightButton,speedButton);config.append(configHeading,configMetrics);
 const stateLabel=node('span','state-chip','Draft');stateLabel.id='planState';const reviewHeading=review.querySelector('h2');reviewHeading.textContent='Your plan';const reviewHeadingWrap=node('div','section-heading');reviewHeading.replaceWith(reviewHeadingWrap);reviewHeadingWrap.append(reviewHeading,stateLabel);
 const empty=node('div','empty-review');empty.id='emptyReview';empty.innerHTML=icon+'<h3>A line is all you need</h3><p>Draw on the map or pick an example. Your checked plan will appear here.</p>';
 review.insertBefore(empty,byId('status'));
 byId('status').textContent='Add a route to get started.';
 const results=byId('results'),metrics=results.querySelector('.metrics');results.insertBefore(metrics,byId('reviewSummary'));
 const modelDetails=node('details','model-details');modelDetails.append(node('summary','','Model and assumptions'),byId('aircraftSummary'));
 results.append(modelDetails);byId('downloadCsv').textContent='Download CSV';byId('downloadEvidence').textContent='Plan report';
 const downloads=byId('downloadCsv').parentElement;downloads.classList.add('downloads');results.insertBefore(downloads,results.querySelector('details'));
 const action=node('div','plan-action');action.append(byId('compile'),byId('busy'));
 const smallPrint=node('p','local-note','Runs on this device. No aircraft is connected.');action.append(smallPrint);review.append(action);
 const simDetails=node('details','simulation-details');const simSummary=node('summary');simSummary.append(node('span','','Cloud simulation'),node('span','coming-soon','Not connected'));simDetails.append(simSummary,simulation);
 simulation.querySelector('h2').remove();
 const help=node('details','workspace-help');help.append(node('summary','','How to use the workspace'));const helpContent=node('div');helpContent.innerHTML='<p><b>1.</b> Draw from the gold START marker, or choose an example.</p><p><b>2.</b> Set the height and speed if you need to. Defaults are ready.</p><p><b>3.</b> Plan and verify, then review the blue route and download.</p><p>Use Points & stops to enter coordinates or add a stop. Undo also works after Clear.</p>';
 helpContent.append(compilePanel.querySelector('details'));help.append(helpContent);
 rail.append(config,review,simDetails,help);main.append(editor,rail);
 const dialog=node('dialog','settings-dialog');dialog.id='settingsDialog';dialog.setAttribute('aria-labelledby','settingsTitle');
 const dialogHeader=node('div','dialog-header');const dialogTitle=node('h2','','Workspace settings');dialogTitle.id='settingsTitle';const close=node('button','close-button','×');close.setAttribute('aria-label','Close settings');dialogHeader.append(dialogTitle,close);
 const tabbar=node('div','settings-tabs');tabbar.setAttribute('role','tablist');tabbar.setAttribute('aria-label','Settings categories');
 const panels=node('div','settings-content');const pointEditor=byId('pointEditor');pointEditor.open=true;pointEditor.querySelector('summary').hidden=true;
 const advanced=node('div');advanced.append(byId('speedCapControls'),byId('motionOptions'));
 const entries=[['flight','Flight',flight],['advanced','Advanced',advanced],['points','Points & stops',pointEditor],['model','Model',model]];
 let lastTrigger=settingsButton;
 function selectTab(key){for(const [id,,panel] of entries){byId('panel-'+id).hidden=id!==key;const tab=byId('settings-'+id);tab.setAttribute('aria-selected',String(id===key));tab.tabIndex=id===key?0:-1;}}
 for(const [id,title,panel] of entries){const b=node('button','',title);b.id='settings-'+id;b.setAttribute('role','tab');b.setAttribute('aria-controls','panel-'+id);panel.setAttribute('role','tabpanel');panel.setAttribute('aria-labelledby',b.id);panel.id==='flightSettings'||panel.id==='pointEditor'?panel.setAttribute('data-panel',id):null;const wrapper=node('div','settings-panel');wrapper.id='panel-'+id;wrapper.append(panel);panels.append(wrapper);b.onclick=()=>{selectTab(id);};b.onkeydown=e=>{if(!['ArrowLeft','ArrowRight','Home','End'].includes(e.key))return;e.preventDefault();const index=entries.findIndex(x=>x[0]===id),next=e.key==='Home'?0:e.key==='End'?entries.length-1:(index+(e.key==='ArrowRight'?1:-1)+entries.length)%entries.length;selectTab(entries[next][0]);byId('settings-'+entries[next][0]).focus();};tabbar.append(b);}
 dialog.append(dialogHeader,tabbar,panels);
 function openSettings(key,trigger){lastTrigger=trigger||settingsButton;selectTab(key);if(!dialog.open)dialog.showModal();byId('settings-'+key).focus();}
 settingsButton.onclick=()=>openSettings('flight',settingsButton);editSettings.onclick=()=>openSettings('flight',editSettings);pointButton.onclick=()=>openSettings('points',pointButton);
 heightButton.onclick=()=>{openSettings('flight',heightButton);byId('altitude').focus();};speedButton.onclick=()=>{openSettings('flight',speedButton);byId('targetSpeed').focus();};
 close.onclick=()=>dialog.close();dialog.addEventListener('close',()=>lastTrigger?.focus());dialog.addEventListener('click',e=>{if(e.target===dialog){const box=dialog.getBoundingClientRect();if(e.clientX<box.left||e.clientX>box.right||e.clientY<box.top||e.clientY>box.bottom)dialog.close();}});
 // Keep elements referenced by the core, but remove duplicate actions/instructions.
 const retained=node('div');retained.hidden=true;retained.append(steps,toolbar.querySelector('#planQuick'),byId('planRoute'),byId('routeNextTitle'),byId('routeNextHint'),byId('lifecycle'),examples);
 originalMain.replaceWith(main);document.body.append(dialog,retained);
 const yawLabel=document.querySelector('label[for="yaw"]'),fixedLabel=document.querySelector('label[for="fixedYaw"]');
 const headingGroup=node('div','heading-settings');headingGroup.append(yawLabel,byId('yaw'),fixedLabel,byId('fixedYaw'));
 flight.append(headingGroup);byId('yaw').value='tangent';
 byId('yaw').querySelector('[value="tangent"]').textContent='Face along the route';
 const headingExplanation=node('p','muted');headingExplanation.id='headingExplanation';headingGroup.append(headingExplanation);
 function headingSummary(){const fixed=byId('yaw').value==='fixed';fixedLabel.hidden=byId('fixedYaw').hidden=!fixed;headingExplanation.textContent=fixed?'Hold the chosen heading while moving. 0° keeps the launch reference direction.':'Yaw changes with the route direction. Vertical takeoff and landing hold their phase heading.';}
 byId('yaw').addEventListener('change',headingSummary);headingSummary();
 const fidelityHint=node('p','muted');fidelityHint.textContent='Allowed rounding is the hard limit. We aim for one quarter of it, slow down for tighter turns, and show when that closer-fit goal cannot be reached. Mark a corner as a stop when you need it exact.';byId('deviation').after(fidelityHint);
 const closureHint=node('p','muted');closureHint.textContent='Choosing Loop connects END back to START. Open routes stop at END before returning home.';byId('closed').after(closureHint);

 document.querySelector('footer').textContent='Trajectory Studio · Planning checks a mathematical model. Gazebo has not run.';
 function syncSummary(){heightButton.innerHTML='<span>Height</span><strong>'+byId('altitude').value+' <small>m</small></strong>';speedButton.innerHTML='<span>Cruising speed</span><strong>'+byId('targetSpeed').value+' <small>m/s</small></strong>';}
 byId('altitude').addEventListener('input',syncSummary);byId('targetSpeed').addEventListener('input',syncSummary);syncSummary();
 let runState='draft',startedAt=null,elapsedTimer=null;
 const runInfo=node('div','run-info');runInfo.hidden=true;runInfo.setAttribute('role','status');
 const pulse=node('span','run-pulse'),runText=node('span'),elapsed=node('span','run-elapsed');elapsed.id='runElapsed';
 const cancel=node('button','text-button','Cancel check');cancel.id='cancelPlanning';cancel.onclick=()=>window.PX4Editor?.cancelPlanning();
 runInfo.append(pulse,runText,elapsed,cancel);byId('busy').before(runInfo);
 const messages={starting:'Starting the planning engine',loading:'Preparing the engine',checking:'Checking geometry and motion',stale:'Edits kept · previous check will be discarded'};
 function syncState(){const busy=byId('busy').style.display==='block',ready=!results.hidden,error=byId('status').classList.contains('error');
  if(!busy&&error)runState='error';
  empty.hidden=busy||ready||error;const label=window.PX4RunState.labels[runState];
  stateLabel.textContent=label;stateLabel.dataset.state=label;review.dataset.state=label;
  byId('compile').setAttribute('aria-busy',String(busy));byId('compile').textContent=busy?'Checking your route…':ready?'Verify again':'Plan and verify';
  runInfo.hidden=!busy;cancel.disabled=!busy;runText.textContent=messages[runState]||'Finishing checks';
  if(busy&&startedAt===null){startedAt=performance.now();elapsedTimer=setInterval(()=>{elapsed.textContent=Math.floor((performance.now()-startedAt)/1000)+' s';},1000);}
  if(!busy){clearInterval(elapsedTimer);startedAt=null;elapsed.textContent='';}
 }
 window.addEventListener('planner-state',e=>{runState=window.PX4RunState.next(runState,e.detail.state);syncState();});
 window.addEventListener('planner-progress',e=>{if(byId('busy').style.display!=='block')return;const stage=/Loading|Preparing/.test(e.detail)?'loading':'checking';runState=window.PX4RunState.next(runState,stage);syncState();});
 new MutationObserver(syncState).observe(byId('status'),{childList:true,attributes:true});new MutationObserver(syncState).observe(results,{attributes:true,attributeFilter:['hidden']});new MutationObserver(syncState).observe(byId('busy'),{attributes:true,attributeFilter:['style']});syncState();
 // The core hides this details element in shape mode; only show its tab contents when drawing.
 window.addEventListener('DOMContentLoaded',()=>{const modeHint=node('p','muted','Point editing is available in Draw mode. Choose Draw in Flight settings.');pointEditor.parentElement.append(modeHint);new MutationObserver(()=>modeHint.hidden=!pointEditor.hidden).observe(pointEditor,{attributes:true,attributeFilter:['hidden']});modeHint.hidden=!pointEditor.hidden;});
})();
