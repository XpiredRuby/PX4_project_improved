/* The same Python/NumPy compiler as the local studio, isolated off the UI thread. */
let runtime,verifiedBuild;
const announce=message=>postMessage({progress:message});
async function boot(){
  announce('Loading the planning engine. The first visit needs a download; keep this page open…');
  const {loadPyodide}=await import('https://cdn.jsdelivr.net/pyodide/v0.27.7/full/pyodide.mjs');
  const py=await loadPyodide({indexURL:'https://cdn.jsdelivr.net/pyodide/v0.27.7/full/'});
  announce('Preparing the motion checks…');
  await py.loadPackage('numpy');
  const expected=new URL(location.href).searchParams.get('build');
  const response=await fetch('/python-manifest.json'+(expected?'?v='+encodeURIComponent(expected):''),{cache:'no-store'});
  if(!response.ok)throw Error('Planner files could not be loaded. Refresh and try again.');
  const manifest=await response.json();
  verifiedBuild=manifest.build_id;
  if(expected&&manifest.build_id!==expected)throw Error('The app updated while this page was open. Refresh to use one consistent version.');
  for(const file of manifest.files){
    const r=await fetch('/python/'+file.path+'?v='+file.sha256);
    if(!r.ok)throw Error('A planner file is missing. Refresh and try again.');
    const bytes=new Uint8Array(await r.arrayBuffer());
    const path='/project/'+file.path;
    py.FS.mkdirTree(path.slice(0,path.lastIndexOf('/')));
    py.FS.writeFile(path,bytes);
  }
  py.globals.set('_manifest_json',JSON.stringify(manifest));
  await py.runPythonAsync(`
import sys, json, hashlib
from pathlib import Path
for item in json.loads(_manifest_json)['files']:
    if hashlib.sha256(Path('/project', item['path']).read_bytes()).hexdigest() != item['sha256']:
        raise ValueError('Planner file integrity check failed. Refresh and try again.')
sys.path.insert(0, '/project')
from mission_planner.preview import preview
from mission_planner.aircraft import assess_profile, import_px4_parameters
from mission_planner.explain import PlanningError
`);
  return py;
}
const operations={preview:'preview',assess:'assess_profile',parameters:'import_px4_parameters'};
let busy=false;
onmessage=async({data})=>{
  const {id,operation,payload}=data;
  if(busy){postMessage({id,error:'A plan is already being checked. Please wait.'});return;}
  if(!Object.hasOwn(operations,operation)){postMessage({id,error:'Unknown planning request.'});return;}
  busy=true;
  try{
    const py=await(runtime??=boot());
    announce('Smoothing your route and independently checking its motion limits…');
    py.globals.set('_request_json',JSON.stringify(payload));
    // Only fixed functions may run; user values are data, never executable code.
    const argument=operation==='parameters'?"json.loads(_request_json)['text']":'json.loads(_request_json)';
    const result=JSON.parse(await py.runPythonAsync(`
try:
    _result = ${operations[operation]}(${argument})
except PlanningError as exc:
    _result = {**exc.to_dict(), 'accepted': False, 'error': str(exc)}
except (ValueError, TypeError, KeyError) as exc:
    _result = {'accepted': False, 'error': str(exc)}
json.dumps(_result, allow_nan=False)
`));
    result.execution_location='browser';
    result.app_build_id=verifiedBuild;
    result.simulation_status='NOT_RUN';
    postMessage({id,result});
  }catch(error){runtime=undefined;postMessage({id,error:'The planning engine could not finish. '+error.message});}
  finally{busy=false;}
};
