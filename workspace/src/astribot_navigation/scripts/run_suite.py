#!/usr/bin/env python3
"""Run inside the offline container; report all failures without hiding failed runs."""
import argparse,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--output',default='/data/reports/suite');p.add_argument('--rate',type=float,default=2.);p.add_argument('--smoke',action='store_true');p.add_argument('--scenario',choices=['room','corridor','obstacles']);p.add_argument('--navigation-only',action='store_true');a=p.parse_args()
root=Path(a.output);root.mkdir(parents=True,exist_ok=True);runs=[]
def execute(mode,scenario,seed=42,source='front',noise=False,map_file=None):
    tag=f'{mode}-{scenario}-{source}-{seed}-'+('noise' if noise else 'clean');out=root/tag
    cmd=['ros2','run','astribot_nav_sim','experiment',mode,'--scenario',scenario,'--seed',str(seed),'--scan-source',source,'--rate',str(a.rate),'--output',str(out)]
    if noise:cmd+=['--noise']
    if map_file:cmd+=['--map',str(map_file)]
    result=subprocess.run(cmd,timeout=600)
    report=out/'report.json'
    data=json.loads(report.read_text()) if report.exists() else {'passed':False,'error':'No report'}
    runs.append({'id':tag,'returncode':result.returncode,'report':data})
    (root/'summary.json').write_text(json.dumps(runs,indent=2))
    return out/'map.yaml',data.get('passed',False)
scenarios=[a.scenario] if a.scenario else (['room'] if a.smoke else ['room','corridor','obstacles'])
for scenario in scenarios:
    maps={}
    if a.navigation_only:
        file=root/f'mapping-{scenario}-front-42-clean/map.yaml'
        report=file.parent/'report.json'
        if not report.exists() or not json.loads(report.read_text()).get('passed'):raise RuntimeError('Existing accepted mapping report required')
        maps['front']=(file,True)
    for source in ([] if a.navigation_only else (['front'] if a.smoke else ['front','dual'])):
        file,passed=execute('mapping',scenario,source=source);maps[source]=(file,passed)
        if not a.smoke:execute('mapping',scenario,source=source,noise=True)
    map_file,passed=maps['front']
    if not passed:
        runs.append({'id':'navigation-'+scenario,'report':{'passed':False,'error':'Mapping acceptance failed; navigation skipped'}});continue
    for seed in ([42] if a.smoke else [42,43,44]):execute('navigation',scenario,seed,source='front',noise=True,map_file=map_file)
    if scenario=='room':execute('faults',scenario,map_file=map_file)
(root/'summary.json').write_text(json.dumps(runs,indent=2))
raise SystemExit(0 if all(x['report'].get('passed',False) for x in runs) else 1)
