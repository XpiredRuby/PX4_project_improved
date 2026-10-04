import json,sys,time,subprocess,re
from pathlib import Path
from datetime import datetime,timezone
import pandas as pd,numpy as np
root=Path('/home/xpire/PX4-runs/repro-oct04-20261004T024700Z')
ulogs=Path('/home/xpire/PX4-Autopilot/build/px4_sitl_default/rootfs/log')
records=[]
while True:
 for outer in sorted(root.iterdir()):
  if not outer.is_dir():continue
  folder=outer/'batch'/outer.name
  if not (folder/'audit.json').exists() or any(r['case']==outer.name for r in records):continue
  try:
   manifest=json.loads(next(folder.glob('run_manifest_*.json')).read_text())
   a=json.loads((folder/'audit.json').read_text())
  except (json.JSONDecodeError,StopIteration):continue
  started=datetime.fromisoformat(manifest['started_at_utc'].replace('Z','+00:00'))
  matches=[]
  for p in ulogs.glob('*/*.ulg'):
   try:logged=datetime.strptime(p.parent.name+' '+p.stem,'%Y-%m-%d %H_%M_%S').replace(tzinfo=timezone.utc)
   except ValueError:continue
   if 0 <= (logged-started).total_seconds() < 60:matches.append(p)
  assert len(matches)==1,(folder,matches)
  subprocess.run([sys.executable,'analysis/sitl_ground_truth.py',str(folder),'--ulog',str(matches[0])],check=True)
  d=pd.read_csv(next(folder.glob('research_log_*.csv')),low_memory=False)
  t=pd.read_csv(folder/'aligned_ground_truth.csv.gz')
  touchdown=d.index[d.phase.isin(['PX4_LAND','PX4_FAILSAFE']) & d.landed_state.eq(1)][0]
  ref=d.loc[:touchdown,['desired_x','desired_y']].dropna().iloc[-1].to_numpy()
  assert t.loc[touchdown,['truth_x','truth_y','truth_z']].notna().all()
  result={'case':outer.name,'audit_passed':a['overall_passed'],
   'physical_touchdown_xy_error_m':float(np.linalg.norm(t.loc[touchdown,['truth_x','truth_y']].to_numpy()-ref)),
   'physical_contact_z_ned':float(t.loc[touchdown,'truth_z']),
   'contact_combined_tilt_deg':float(np.degrees(np.arccos(np.clip(np.cos(d.loc[touchdown,'roll'])*np.cos(d.loc[touchdown,'pitch']),-1,1)))),
   'total_logged_elapsed_s':float(d.elapsed_s.max()),
   'first_native_land_elapsed_s':float(d.loc[d.phase.isin(['PX4_LAND','PX4_FAILSAFE']),'elapsed_s'].iloc[0]),
   'outcome':manifest['outcome'],'final_state':manifest['final_state'],'px4_parameters':manifest['px4_parameters'],
   'source_sha256':manifest['source_sha256'],
   'ulog_sha256':json.loads((folder/'ground_truth_audit.json').read_text())['ulog']['sha256'],
   'failed_checks':[c for c in a['checks'] if not c['passed']]}
  h=d.navigation_state.eq('HOLD') & ~d.phase.isin(['PX4_LAND','PX4_FAILSAFE'])
  if h.any():
   hd=d.loc[h];ht=t.loc[h]
   result['physical_hold_xy_drift_m']=float(np.hypot(ht.truth_x-ht.truth_x.iloc[0],ht.truth_y-ht.truth_y.iloc[0]).max())
   result['physical_hold_reference_max_error_m']=float(np.hypot(ht.truth_x-hd.desired_x,ht.truth_y-hd.desired_y).max())
  (folder/'independent_verification.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
  records.append(result)
  (root/'independent_verification.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
  print(json.dumps({k:v for k,v in result.items() if k not in ['source_sha256','px4_parameters','final_state']}),flush=True)
 break
