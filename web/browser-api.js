/* Local-only planner adapter. It never connects to a laptop or a flight runner. */
(()=>{
  const originalFetch=window.fetch.bind(window);
  let worker,sequence=0,pending;
  function reset(){worker?.terminate();worker=undefined;}
  function run(operation,payload){
    if(pending)return Promise.reject(Error('A planning check is already running. Please wait.'));
    if(!worker){
      worker=new Worker('/planner-worker.js',{type:'module'});
      worker.onmessage=({data})=>{
        if(data.progress){window.dispatchEvent(new CustomEvent('planner-progress',{detail:data.progress}));return;}
        if(!pending||data.id!==pending.id)return;
        const item=pending;pending=undefined;clearTimeout(item.timer);
        if(data.error){reset();item.reject(Error(data.error));}else item.resolve(data.result);
      };
      worker.onerror=()=>{if(pending){const item=pending;pending=undefined;clearTimeout(item.timer);item.reject(Error('The planning engine could not load. Check your internet connection, then try again.'));}reset();};
    }
    return new Promise((resolve,reject)=>{
      const id=++sequence;
      const timer=setTimeout(()=>{pending=undefined;reset();reject(Error('This route took too long on this device. Try fewer points or a simpler route, then plan again.'));},120000);
      pending={id,timer,resolve,reject};worker.postMessage({id,operation,payload});
    });
  }
  function json(value,status=200){return new Response(JSON.stringify(value),{status,headers:{'Content-Type':'application/json'}});}
  window.fetch=async(input,options={})=>{
    const url=new URL(typeof input==='string'?input:input.url,location.href);
    if(url.origin!==location.origin||!url.pathname.startsWith('/api/'))return originalFetch(input,options);
    const path=url.pathname;
    if(path==='/api/capabilities')return json({sitl_enabled:false,hardware_enabled:false,execution_location:'browser'});
    if(path==='/api/editor-settings')return json({max_input_points:256,max_radius_m:40,max_height_m:30,connected_heading_modes:[]});
    if(path==='/api/aircraft/binding')return json({profile:null});
    if(path==='/api/aircraft/catalog')return originalFetch('/aircraft-catalog.json');
    if(/^\/api\/aircraft\/example\/(quad|hex|octo|x500)$/.test(path))return originalFetch('/python/config/planner/aircraft/'+path.split('/').pop()+'.json');
    if(path.startsWith('/api/rehearsal'))return json({error:'Cloud Gazebo is not connected. No simulation has started.'},404);
    const operation={'/api/preview':'preview','/api/aircraft/assess':'assess','/api/aircraft/parameters':'parameters'}[path];
    if(!operation)return json({error:'Unknown request'},404);
    if(typeof options.body!=='string'||new TextEncoder().encode(options.body).length>65536)return json({error:'The drawing is too large. Use fewer points.'},400);
    try{const result=await run(operation,JSON.parse(options.body));return json(result,result.error?400:200);}
    catch(error){return json({accepted:false,error:error.message},503);}
  };
  window.addEventListener('planner-progress',event=>{
    const busy=document.getElementById('busy');
    if(busy&&busy.style.display==='block')busy.textContent=event.detail;
  });
})();
