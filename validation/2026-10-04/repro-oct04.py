import traceback
from pathlib import Path
try:
 import os,sys,time,json,signal,subprocess,hashlib
 from pathlib import Path
 from datetime import datetime,timezone
 import xml.etree.ElementTree as ET
 from pymavlink import mavutil
 ROOT=Path('/home/xpire/PX4-research-oct04')
 AUTO=Path('/home/xpire/PX4-Autopilot')
 sys.path.insert(0,str(ROOT/'scripts'))
 from sitl_fault_injector import read_parameter,set_parameter
 from sitl_monitor import allow_next_trial
 import resource
 old_limits=resource.getrlimit(resource.RLIMIT_CORE)
 resource.setrlimit(resource.RLIMIT_CORE,(0,old_limits[1]))
 BASE=Path('/home/xpire/PX4-runs')/('repro-oct04-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
 BASE.mkdir()
 Path('/mnt/f/PX4/repro-oct04.run-dir').write_text(str(BASE))
 summary=[]
 def progress(stage,case=None):
  Path('/mnt/f/PX4/repro-oct04-state.json').write_text(json.dumps({'stage':stage,'case':case,'time':datetime.now(timezone.utc).isoformat(),'root':str(BASE)},indent=2)+'\n')
  print(stage,case,flush=True)
 def connection():
  m=mavutil.mavlink_connection('udpin:127.0.0.1:14601',source_system=253,source_component=190)
  if m.wait_heartbeat(timeout=15) is None:raise RuntimeError('No fresh SITL heartbeat')
  m.target_component=1
  return m
 def ground(m):
  for _ in range(1000):
   if m.recv_match(blocking=False) is None:break
  for mid in (245,375):
   m.mav.command_long_send(m.target_system,1,mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,0,mid,100000,0,0,0,0,0)
  seen={};at={};end=time.monotonic()+4;safe_since=None
  while time.monotonic()<end:
   p=m.recv_match(blocking=True,timeout=.2)
   now=time.monotonic()
   if p is not None and p.get_srcSystem()==m.target_system and p.get_srcComponent()==1:
    kind=p.get_type()
    if kind=='HEARTBEAT':seen['armed']=bool(p.base_mode&128);at['armed']=now
    elif kind=='EXTENDED_SYS_STATE':seen['landed_state']=p.landed_state;at['landed_state']=now
    elif kind=='ACTUATOR_OUTPUT_STATUS':seen['actuators']=list(p.actuator[:4]);at['actuators']=now
   fresh=len(at)==3 and all(now-v<=1.5 for v in at.values())
   safe=fresh and seen.get('armed') is False and seen.get('landed_state')==1 and len(seen.get('actuators',[]))==4 and max(abs(x) for x in seen['actuators'])<=.01
   if safe:
    if safe_since is None:safe_since=now
    if now-safe_since>=1.:return seen
   else:safe_since=None
  raise RuntimeError('Cannot confirm fresh ground/disarm/zero propulsion: '+str(seen))
 def stop_group(pid):
  cmd=Path('/proc')/str(pid)/'cmdline'
  if not cmd.exists():return
  text=cmd.read_bytes()
  if b'px4_sitl' not in text or b'gz_x500' not in text:raise RuntimeError('Simulator process identity changed')
  if os.getpgid(pid)!=pid:raise RuntimeError('Simulator process group identity changed')
  os.killpg(pid,signal.SIGTERM)
  time.sleep(3)
 def configure(m,folder):
  before={k:read_parameter(m,k) for k in ['MPC_LAND_SPEED','MPC_LAND_CRWL','EKF2_NOAID_TOUT']}
  assert before['EKF2_NOAID_TOUT'][0]==5000000
  set_parameter(m,'MPC_LAND_SPEED',.4,before['MPC_LAND_SPEED'][1])
  assert abs(read_parameter(m,'MPC_LAND_SPEED')[0]-.4)<1e-5
  assert abs(before['MPC_LAND_CRWL'][0]-.3)<1e-5
  (folder/'parameter_configuration.json').write_text(json.dumps({'before':before,'applied':{'MPC_LAND_SPEED':.4},'ground_verified':ground(m)},indent=2)+'\n')

 for p in Path('/proc').glob('[0-9]*/cmdline'):
  try:
   if b'/PX4-Autopilot/build/px4_sitl_default/bin/px4' in p.read_bytes():raise RuntimeError('An existing PX4 simulation is running')
  except (FileNotFoundError,ProcessLookupError,PermissionError):pass

 plans=[('gust-gps-12s-repro','afvl_validation','0,0,0,0,0,0',{'gust':True,'gps_outage_s':12.,'trigger_phase':'TRAJECTORY','trigger_clock_s':15.,'expected':'navigation_landing'})]
 for name,world_name,pose,extra in plans:
  folder=BASE/name;folder.mkdir();progress('START_SIMULATOR',name)
  case=dict(name=name,**extra)
  cases=folder/'cases.json';cases.write_text(json.dumps([case],indent=2)+'\n')
  env=os.environ.copy();env['HEADLESS']='1';env['PX4_GZ_WORLD']=world_name;env['PX4_GZ_MODEL_POSE']=pose
  world_file=AUTO/'Tools/simulation/gz/worlds'/(world_name+'.sdf')
  (folder/'simulator_configuration.json').write_text(json.dumps({'world':world_name,'model_pose':pose,'world_sha256':hashlib.sha256(world_file.read_bytes()).hexdigest(),'controller_root':str(ROOT)},indent=2)+'\n')
  simlog=(folder/'simulator.log').open('w')
  sim=subprocess.Popen(['bash','-c','tail -f /dev/null | make px4_sitl gz_x500'],cwd=AUTO,env=env,stdout=simlog,stderr=subprocess.STDOUT,start_new_session=True)
  deadline=time.monotonic()+35
  while True:
   if sim.poll() is not None:raise RuntimeError('Simulator startup exited: '+name)
   r=subprocess.run([str(AUTO/'build/px4_sitl_default/bin/px4-mavlink'),'start','-u','14600','-o','14601','-t','127.0.0.1','-m','onboard','-r','400000'],capture_output=True)
   if r.returncode==0:break
   if time.monotonic()>deadline:raise RuntimeError('Simulator startup deadline: '+name)
   time.sleep(.5)
  m=connection();ground(m);configure(m,folder);m.close()
  progress('RUN_MISSION',name)
  run=subprocess.run([sys.executable,'-u',str(ROOT/'scripts/sitl_scenario_batch.py'),'--confirm-sitl','--world',world_name,'--cases',str(cases),'--output',str(folder/'batch')],cwd=ROOT)
  result=json.loads((folder/'batch/summary.json').read_text())[0]
  progress('VERIFY_FINAL_GROUND',name)
  m=connection();evidence=ground(m);m.close()
  (folder/'final_ground.json').write_text(json.dumps(evidence,indent=2)+'\n')
  stop_group(sim.pid)
  try:sim.wait(timeout=10)
  except subprocess.TimeoutExpired:raise RuntimeError('Simulator shutdown timeout after verified ground')
  simlog.close()
  record=dict(case=name,batch_exit=run.returncode,result=result,final_ground=evidence)
  summary.append(record);(BASE/'driver_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
  if not allow_next_trial(case,result):
   progress('BLOCKED_BY_VALIDATION_FAILURE',name)
   raise RuntimeError('Strict validation failed; simulator stopped only after safe ground confirmation: '+name)
 progress('COMPLETED')

 Path('/mnt/f/PX4/repro-oct04.exit').write_text('0')
except Exception:
 traceback.print_exc()
 Path('/mnt/f/PX4/repro-oct04.exit').write_text('1')
 raise
