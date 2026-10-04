set -eu
cd /home/xpire/PX4-final-check
python3 - <<'PY'
import json,numpy as np,pandas as pd
from pathlib import Path
from pyulog import ULog
root=Path(Path('/mnt/f/PX4/terrain-final-oct03.run-dir').read_text().strip())
records=[]
for outer in sorted(root.iterdir()):
 folder=outer/'batch'/outer.name
 if not (folder/'independent_verification.json').exists():continue
 r=json.loads((folder/'independent_verification.json').read_text())
 ga=json.loads((folder/'ground_truth_audit.json').read_text())
 paths=list(Path('/home/xpire/PX4-Autopilot/build/px4_sitl_default/rootfs/log').glob('*/'+ga['ulog']['name']))
 assert len(paths)==1
 u=ULog(str(paths[0]),message_name_filter_list=['vehicle_global_position_groundtruth'])
 tr=next(x.data for x in u.data_list if x.name=='vehicle_global_position_groundtruth')
 ts=tr['timestamp'];alt=tr['alt'];initial=ts<=ts[0]+1000000
 assert float(np.ptp(alt[initial])) < .02,'Initial ground was not stable'
 d=pd.read_csv(next(folder.glob('research_log_*.csv')),low_memory=False)
 idx=d.index[d.phase.isin(['PX4_LAND','PX4_FAILSAFE']) & d.landed_state.eq(1)][0]
 contact=float(np.interp(d.loc[idx,'position_time_boot_ms']*1000.,ts,alt))
 r['physical_surface_height_change_m']=contact-float(np.median(alt[initial]))
 r['height_change_method']='ULog absolute simulator truth altitude: first stable ground second to contact; excludes bootstrap ascent and estimator-origin shifts'
 r['initial_ground_truth_altitude_m']=float(np.median(alt[initial]))
 r['contact_ground_truth_altitude_m']=contact
 r['first_controller_sample_truth_z_ned']=r.pop('physical_start_z_ned',r.get('first_controller_sample_truth_z_ned'))
 r['first_controller_sample_truth_xy_ned']=r.pop('physical_start_xy_ned',r.get('first_controller_sample_truth_xy_ned'))
 r['maximum_ground_combined_tilt_deg']=float(np.degrees(np.arccos(np.clip(np.cos(d.loc[idx:,'roll'])*np.cos(d.loc[idx:,'pitch']),-1,1))).max())
 r['ulog_absolute_path']=str(paths[0])
 (folder/'independent_verification.json').write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
 records.append(r)
 print(json.dumps({k:r[k] for k in ['case','audit_passed','physical_touchdown_xy_error_m','physical_surface_height_change_m','contact_combined_tilt_deg','maximum_ground_combined_tilt_deg','outcome','failed_checks']}))
(root/'independent_verification.json').write_text(json.dumps(records,indent=2,allow_nan=False)+'\n')
assert len(records)==len([p for p in root.iterdir() if (p/'batch'/p.name/'audit.json').exists()])
PY
