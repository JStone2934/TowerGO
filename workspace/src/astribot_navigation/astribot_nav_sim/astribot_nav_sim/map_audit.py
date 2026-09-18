#!/usr/bin/env python3
"""Independent map coverage audit, including walls incorrectly marked free."""
import argparse,json,math
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial import cKDTree
from astribot_nav_sim.world import geometry

def pgm(path):
    with open(path,'rb') as f:
        def token():
            out=b''
            while True:
                c=f.read(1)
                if c==b'#':f.readline();continue
                if not c:break
                if c.isspace():
                    if out:return out
                else:out+=c
            return out
        mode=token();w=int(token());h=int(token());maximum=int(token())
        if maximum!=255 or mode!=b'P5':raise RuntimeError('Expected 8 bit P5 PGM')
        return np.frombuffer(f.read(w*h),dtype=np.uint8).reshape(h,w)

def audit(map_yaml,scenario):
    config=yaml.safe_load(map_yaml.read_text());grid=np.flipud(pgm(map_yaml.parent/config['image']))
    res=float(config['resolution']);ox,oy,a=config['origin'];rot=np.array([[math.cos(a),-math.sin(a)],[math.sin(a),math.cos(a)]])
    occ=(255-grid)/255.>config['occupied_thresh'];free=(255-grid)/255.<config['free_thresh']
    iy,ix=np.where(occ);points=np.stack(((ix+.5)*res,(iy+.5)*res),axis=1)@rot.T+np.array([ox,oy])
    if not len(points):raise RuntimeError('No occupied cells')
    walls=geometry(scenario);samples=[]
    for start,end in walls:
        n=max(2,int(np.linalg.norm(end-start)/.025)+1);samples.extend(np.linspace(start,end,n))
    samples=np.array(samples);local=(samples-np.array([ox,oy]))@rot
    indices=np.floor(local/res).astype(int);inside=(indices[:,0]>=0)&(indices[:,0]<grid.shape[1])&(indices[:,1]>=0)&(indices[:,1]<grid.shape[0])
    distances,_=cKDTree(points).query(samples)
    in_free=np.zeros(len(samples),dtype=bool)
    in_free[inside]=free[indices[inside,1],indices[inside,0]]
    unexplained=in_free&(distances>.15)
    return {'passed':bool(np.mean(distances<=.15)>=.99 and not np.any(unexplained)), 'wall_samples_outside_grid':int(np.sum(~inside)), 'true_wall_samples':len(samples),'true_wall_coverage_within_15cm':float(np.mean(distances<=.15)),
            'wall_to_map_p95_m':float(np.percentile(distances,95)),
            'wall_samples_in_free_space_without_nearby_obstacle':int(unexplained.sum()),
            'unexplained_free_fraction':float(np.mean(unexplained))}
def main():
    p=argparse.ArgumentParser();p.add_argument('reports');p.add_argument('--map');p.add_argument('--scenario');a=p.parse_args();rows=[]
    if a.map:
        row=audit(Path(a.map),a.scenario);Path(a.reports).write_text(json.dumps(row,indent=2));print(json.dumps(row));return
    for file in sorted(Path(a.reports).glob('acceptance-*/mapping-*/map.yaml')):
        report=json.loads((file.parent/'report.json').read_text());row=audit(file,report['scenario']);row.update({'run':file.parent.name,'scan_source':report['scan_source'],'scenario':report['scenario']});rows.append(row)
    (Path(a.reports)/'map-coverage-audit.json').write_text(json.dumps(rows,indent=2));print(json.dumps(rows,indent=2))
if __name__=='__main__':main()
