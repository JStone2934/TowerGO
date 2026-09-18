#!/usr/bin/env python3
"""Exercise the installed navigate CLI against the complete offline stack."""
import argparse,math,subprocess,json
from types import SimpleNamespace
import rclpy
from astribot_nav_sim.experiment import Run
p=argparse.ArgumentParser();p.add_argument('--map',required=True);p.add_argument('--output',required=True);a=p.parse_args()
rclpy.init(args=[])
run=Run(SimpleNamespace(mode='navigation',scenario='room',seed=42,scan_source='front',noise=False,rate=2.,map=a.map,output=a.output,record=False))
try:
    run.start();run.localize()
    result=subprocess.run(['ros2','run','astribot_nav_api','navigate','1.0','0.0','0.0'],stdout=run.log,stderr=subprocess.STDOUT,timeout=90)
    truth=run.node.truth[-1][1:];error=math.hypot(truth[0]-1.,truth[1])
    run.report.update({'mode':'public_api','cli_exit_code':result.returncode,'position_error_m':error,'passed':result.returncode==0 and error<=.2 and run.node.collisions==0})
except Exception as e:run.report.update({'mode':'public_api','passed':False,'error':str(e)})
finally:run.close()
print(json.dumps(run.report,indent=2));raise SystemExit(0 if run.report['passed'] else 1)
