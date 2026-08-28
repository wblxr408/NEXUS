#!/usr/bin/env python3
"""Query fixed sandbox target poses from the authored map model.

This is the final map-query stage, not a detector: ORB-SLAM2 does not identify
objects.  The explicit label prevents static map lookup from being reported as
visual 6-D recognition.
"""
from __future__ import annotations
import argparse, csv, json
from pathlib import Path
import numpy as np, yaml

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--scene',default='simulation/sandbox_scene.yaml'); ap.add_argument('--targets',default='simulation/target_spec.yaml'); ap.add_argument('--out',required=True); args=ap.parse_args()
    scene=yaml.safe_load(Path(args.scene).read_text(encoding='utf-8')); spec=yaml.safe_load(Path(args.targets).read_text(encoding='utf-8'))
    by_id={x['id']:x for x in scene['objects']}; scale=0.001 if scene.get('unit')=='mm' else 1.0
    poses={}
    for idx,obj_id in enumerate(spec['source_object_ids'],1):
        o=by_id[obj_id]; size=(np.asarray(o['size'],float)*scale); p=np.asarray(o['position'],float)*scale
        if scene.get('object_z_mode')=='base': p[2] += float(scene['dimensions']['deck_height'])*scale + size[2]/2
        poses[str(idx)]={'obj_id':idx,'source_object_id':obj_id,'R_map_target':np.eye(3).tolist(),'t_m':p.tolist(),'size_m':size.tolist(),'symmetry':'continuous_z','localization_mode':'known_map_query'}
    Path(args.out).write_text(json.dumps({'algorithm':'known_map_target_query','coordinate_frame':'map','targets':poses,'visual_recognition':False},indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'targets':len(poses),'visual_recognition':False},indent=2))
if __name__=='__main__': main()
