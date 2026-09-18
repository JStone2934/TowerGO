import sys
from pathlib import Path
exec(Path('/towergo/src/astribot_navigation/scripts/audit_maps.py').read_text().split('p=argparse.ArgumentParser()')[0])
p=Path('/data/reports/mapping-obstacles-dual-investigation/map.yaml')
c=yaml.safe_load(p.read_text());g=np.flipud(pgm(p.parent/c['image']));res=c['resolution'];ox,oy,_=c['origin'];iy,ix=np.where((255-g)/255.>c['occupied_thresh']);tree=cKDTree(np.stack(((ix+.5)*res+ox,(iy+.5)*res+oy),axis=1))
for start,end in geometry('obstacles'):
 samples=np.linspace(start,end,100);dist,_=tree.query(samples);print(start,end,'coverage',np.mean(dist<=.15),'p95',np.percentile(dist,95))
