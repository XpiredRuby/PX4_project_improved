/* Local-only planner adapter. It never connects to a laptop or a flight runner. */
(()=>{
  const originalFetch=window.fetch.bind(window);
  let worker,sequence=0,pending;
  function reset(){worker?.terminate();worker=undefined;}
  function run(operation,payload,signal){
    if(signal?.aborted)return Promise.reject(Error('Planning cancelled. Your drawing is kept.'));
    if(pending)return Promise.reject(Error('A planning check is already running. Please wait.'));
    if(!worker){
      worker=new Worker('/planner-worker.js',{type:'module'});
      const activeWorker=worker;
      worker.onmessage=({data})=>{
        if(worker!==activeWorker)return;
        if(data.progress){window.dispatchEvent(new CustomEvent('planner-progress',{detail:data.progress}));return;}
        if(!pending||data.id!==pending.id)return;
        const item=pending;pending=undefined;clearTimeout(item.timer);item.cleanup?.();
        if(data.error){reset();item.reject(Error(data.error));}else item.resolve(data.result);
      };
      worker.onerror=()=>{if(worker!==activeWorker)return;if(pending){const item=pending;pending=undefined;clearTimeout(item.timer);item.cleanup?.();item.reject(Error('The planning engine could not load. Check your internet connection, then try again.'));}reset();};
    }
    return new Promise((resolve,reject)=>{
      const id=++sequence;
      const timer=setTimeout(()=>{pending=undefined;signal?.removeEventListener('abort',abort);reset();reject(Error('This route took too long on this device. Try fewer points or a simpler route, then plan again.'));},120000);
      const abort=()=>{if(pending?.id!==id)return;pending=undefined;clearTimeout(timer);signal?.removeEventListener('abort',abort);reset();reject(Error('Planning cancelled. Your drawing is kept.'));};
      const cleanup=()=>signal?.removeEventListener('abort',abort);
      pending={id,timer,resolve,reject,cleanup};signal?.addEventListener('abort',abort,{once:true});worker.postMessage({id,operation,payload});
    });
  }
  function json(value,status=200){return new Response(JSON.stringify(value),{status,headers:{'Content-Type':'application/json'}});}
  window.fetch=async(input,options={})=>{
    const url=new URL(typeof input==='string'?input:input.url,location.href);
    if(url.origin!==location.origin||!url.pathname.startsWith('/api/'))return originalFetch(input,options);
    const path=url.pathname;
    if(path==='/api/capabilities')return json({sitl_enabled:false,hardware_enabled:false,execution_location:'browser'});
    if(path==='/api/editor-settings')return json({max_input_points:8192,max_radius_m:40,max_height_m:30,connected_heading_modes:[]});
    if(path==='/api/aircraft/binding')return json({profile:null});
    if(path==='/api/aircraft/catalog')return originalFetch('/aircraft-catalog.json');
    if(/^\/api\/aircraft\/example\/(quad|hex|octo|x500)$/.test(path))return originalFetch('/python/config/planner/aircraft/'+path.split('/').pop()+'.json');
    if(path.startsWith('/api/rehearsal'))return json({error:'Cloud Gazebo is not connected. No simulation has started.'},404);
    const operation={'/api/preview':'preview','/api/aircraft/assess':'assess','/api/aircraft/parameters':'parameters'}[path];
    if(!operation)return json({error:'Unknown request'},404);
    if(typeof options.body!=='string'||new TextEncoder().encode(options.body).length>2*1024*1024)return json({error:'The request could not be read. Your drawing is still saved.'},400);
    try{const result=await run(operation,JSON.parse(options.body),options.signal);return json(result,result.error?400:200);}
    catch(error){return json({accepted:false,error:error.message},503);}
  };
  window.addEventListener('planner-progress',event=>{
    const busy=document.getElementById('busy');
    if(busy&&busy.style.display==='block')busy.textContent=event.detail;
  });
})();
