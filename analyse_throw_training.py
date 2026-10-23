"""Inspect recorded gestures without changing recognition or live controls."""
import json
from pathlib import Path
import sys
import numpy as np

root = Path(sys.argv[1] if len(sys.argv)>1 else Path.home()/'mkw-headtracker/throw-training')
for path in sorted(root.glob('*.jsonl')):
    if '195431-' in path.name:  # setup validation, not the user's labelled takes
        continue
    rows=[]
    for line in path.open():
        try:
            item=json.loads(line)
            if item.get('type')=='sample':rows.append(item)
        except ValueError:
            pass
    if not rows:
        continue
    t=np.array([r['t'] for r in rows]);base=t[0]
    print('\nFILE',path.name,'samples',len(rows),'duration',round(t[-1]-base,1),
          'detected_pct',round(100*sum(r['detected'] for r in rows)/len(rows),1))
    print('sample interval ms p50/p95/max',np.round(np.percentile(np.diff(t)*1000,[50,95,100]),1).tolist())
    for side,s,e,w in [('left',11,13,15),('right',12,14,16)]:
        valid=[]
        for r in rows:
            p=np.asarray(r['landmarks'])
            if len(p)<33:continue
            width=np.linalg.norm(p[11,:2]-p[12,:2])
            if width<.03:continue
            vis=min(p[s,3],p[e,3],p[w,3])
            world=np.asarray(r['world_landmarks'])
            q=world if len(world)>=33 else p
            upper=q[s,:3]-q[e,:3];lower=q[w,:3]-q[e,:3]
            angle=np.degrees(np.arccos(np.clip(np.dot(upper,lower)/max(np.linalg.norm(upper)*np.linalg.norm(lower),1e-9),-1,1)))
            valid.append([r['t'],vis,*((p[w,:3]-p[s,:3])/width),angle,*((q[w,:3]-q[s,:3]))])
        a=np.asarray(valid)
        if len(a)<5:continue
        print(side,'usable_pct',round(100*np.mean(a[:,1]>=.4),1),
              'visibility p10/50',np.round(np.percentile(a[:,1],[10,50]),2).tolist())
        sm=a.copy()
        for i in range(2,len(a)):
            sm[i,2:]=np.median(a[i-2:i+1,2:],axis=0)
        dt=np.diff(sm[:,0]);v=np.diff(sm[:,2:5],axis=0)/dt[:,None]
        good=(a[1:,1]>=.4)&(a[:-1,1]>=.4)&(dt<.2)
        speed=np.linalg.norm(v,axis=1)
        if not good.any():continue
        print('  wrist relative xyz p05/p50/p95',np.round(np.percentile(a[a[:,1]>=.4,2:5],[5,50,95],axis=0),2).tolist())
        print('  velocity xyz p05/p50/p95',np.round(np.percentile(v[good],[5,50,95],axis=0),2).tolist(),
              'speed p50/90/99',np.round(np.percentile(speed[good],[50,90,99]),2).tolist())
        # Group fast strokes, showing the lift/throw distinction rather than
        # treating each moving frame as a separate labelled example.
        indices=np.flatnonzero(good&(speed>3.0))+1
        groups=[]
        for i in indices:
            if not groups or sm[i,0]-sm[groups[-1][-1],0]>.35:groups.append([i])
            else:groups[-1].append(i)
        for group in groups:
            peak=max(group,key=lambda i:speed[i-1])
            if speed[peak-1]<4:continue
            lo=max(0,peak-4);hi=min(len(a)-1,peak+3)
            print('  stroke t',round(sm[peak,0]-base,2),'speed',round(speed[peak-1],1),
                  'peak_velocity',np.round(v[peak-1],1).tolist(),
                  'wrist_before/after',np.round(sm[[lo,hi],2:5],2).tolist(),
                  'elbow_before/after',np.round(sm[[lo,hi],5],1).tolist())
