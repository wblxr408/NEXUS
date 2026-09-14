"""Offline height-prior projection; never publishes a live localization result."""
from pathlib import Path
import bisect,hashlib,json
import cv2
import numpy as np
from scipy.spatial.transform import Rotation,Slerp
import yaml

ROOT=Path(__file__).resolve().parents[3]
RUN=Path(__file__).resolve().parent
source=ROOT/'data/raw/2026-09-08_E047_recovery_and_pose_stream/e047_centered_09_events.json'
data=json.loads(source.read_text())
cal=ROOT/'code/ros2_ws/src/nexus_pi_readonly_ingress/config/imx219_intrinsics_measured_v01.json'
intr=json.loads(cal.read_text());K=np.array(intr['camera_matrix'],float)
origin=np.floor(np.array(intr['valid_roi_normalized'][:2])*[intr['width'],intr['height']]);K[:2,2]-=origin
extr=ROOT/'code/ros2_ws/src/nexus_bringup/config/vision_live_approximate.yaml'
T=np.array(yaml.safe_load(extr.read_text())['transform_body_camera'])
poses={int(x['platform']['sample_timestamp_ns']):x['platform'] for x in data['/nexus/viz/localization_state'] if x['platform'].get('position') is not None and x['platform'].get('orientation') is not None and x['platform']['display_state'] in ['observed','predicted']}
stamps=sorted(poses)
reference_pixel=np.array([555.,794.])  # selected tall pole top in centered_v09 reference
rows=[]
for packet in data['/nexus/vision/target_tracks']:
 for track in packet['tracks']:
  if track['track_id']!='centered_object_001' or not track['observed'] or track['identity_ambiguous']:continue
  stamp=packet['sample_timestamp_ns'];idx=bisect.bisect_right(stamps,stamp)
  if not 0<idx<len(stamps):continue
  a,b=stamps[idx-1:idx+1]
  if b-a>250_000_000:continue
  obs=track['feature_observations']
  first=np.array([o['reference_pixel_px'] for o in obs],float);second=np.array([o['pixel_px'] for o in obs],float)
  if len(obs)<8:continue
  H,mask=cv2.findHomography(first,second,cv2.RANSAC,3.)
  if H is None or mask is None or mask.sum()<8:continue
  pixel=cv2.perspectiveTransform(reference_pixel.reshape(1,1,2),H)[0,0]
  fraction=(stamp-a)/(b-a)
  position=(1-fraction)*np.array(poses[a]['position'])+fraction*np.array(poses[b]['position'])
  rotation=Slerp([0.,1.],Rotation.from_quat([poses[a]['orientation'],poses[b]['orientation']]))([fraction]).as_matrix()[0]
  center=position+rotation@T[:3,3];R=rotation@T[:3,:3];direction=R@np.linalg.solve(K,np.r_[pixel,1.])
  results={}
  for height in [.22,.12]:
   if abs(direction[2])<1e-8:continue
   scale=(height-center[2])/direction[2]
   if scale<=0:continue
   point=center+scale*direction;local=R.T@(point-center);reproj=K@local;reproj=reproj[:2]/reproj[2]
   error=float(np.linalg.norm(reproj-pixel))
   assert error<1e-7 and abs(point[2]-height)<1e-9
   results[str(height)]=dict(xyz_m=point.tolist(),reprojection_error_px=error,ray_scale_m=float(scale))
  if results:rows.append(dict(sample_timestamp_ns=stamp,pixel_px=pixel.tolist(),homography_inliers=int(mask.sum()),reference_top_pixel_px=reference_pixel.tolist(),pose_bracket_ns=[a,b],pose_gap_ms=(b-a)/1e6,platform_xyz_m=position.tolist(),results=results))
if not rows:raise RuntimeError('No paired forward-facing height-prior projection')
# Choose maximum visual support, then tighter time bracket; never choose by closeness to catalog.
best=max(rows,key=lambda x:(x['homography_inliers'],-x['pose_gap_ms']))
result=dict(status='COMPUTED_APPROXIMATE_NOT_LIVE_VALIDATED',semantic_label='junction_3_user_identified_signal',coordinate_frame='E047_live_map',method='image_feature_homography_then_camera_ray_intersection_with_height_prior',assumptions=['Tall signal top z=0.22 m; alternate low height is sensitivity only.','Reference top transferred through local homography; the original anchor feature was not directly observed.','E047 approximate camera mounting, lever arm and platform pose retained.','E030 physical_sandbox_map_v01 is not proven aligned with E047_live_map; junction identity is not used to fit the result.'],best=best,accepted_frames=len(rows),per_frame=rows,source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),calibration_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [cal,extr]})
(RUN/'result.json').write_text(json.dumps(result,indent=2))
print(json.dumps(dict(best=best,accepted_frames=len(rows)),indent=2))
