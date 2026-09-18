import os,signal,subprocess,json,time
from types import SimpleNamespace
import rclpy
from astribot_nav_sim.experiment import Run,wait_until
rclpy.init(args=[])
run=Run(SimpleNamespace(mode='navigation',scenario='room',seed=42,scan_source='front',noise=False,rate=2.,map='/data/reports/acceptance-room/mapping-room-front-42-clean/map.yaml',output='/data/reports/public-api-cancel-check',record=False))
child=None
try:
 run.start();run.localize()
 child=subprocess.Popen(['/towergo/install/astribot_nav_api/lib/astribot_nav_api/navigate','3','0','0'],stdout=run.log,stderr=subprocess.STDOUT,start_new_session=True)
 if not wait_until(lambda:sum(abs(v) for v in run.node.actual_velocity)>.01,30):raise RuntimeError('No movement')
 begin=run.node.last_truth_stamp;os.killpg(child.pid,signal.SIGINT)
 stopped=wait_until(lambda:sum(abs(v) for v in run.node.actual_velocity)<.001,3)
 elapsed=run.node.last_truth_stamp-begin
 time.sleep(1)
 sustained=sum(abs(v) for v in run.node.actual_velocity)<.001
 run.report.update(mode='public_api_cancel',stop_seconds=elapsed,sustained_stop=sustained,passed=stopped and elapsed<=.5 and sustained)
except Exception as e:run.report.update(passed=False,error=str(e))
finally:
 if child and child.poll() is None:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
 run.close()
print(json.dumps(run.report,indent=2))
