import json,os,signal,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from nav_msgs.srv import GetMap
assert os.environ.get('ROS_DOMAIN_ID')=='125'
root=Path('/data/maps/static_lidar_20260918_213221');expected=json.loads((root/'map_report.json').read_text());result={'passed':False,'image_id':os.environ.get('TOWERGO_IMAGE_ID'),'maps':{}}
rclpy.init();node=Node('towergo_map_file_check');thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);thread.start()
def call(kind,name,req):
 c=node.create_client(kind,name)
 if not c.wait_for_service(timeout_sec=15):raise RuntimeError(name+' unavailable')
 f=c.call_async(req);end=time.monotonic()+15
 while not f.done() and time.monotonic()<end:time.sleep(.02)
 if not f.done():raise TimeoutError(name)
 value=f.result();node.destroy_client(c);return value
try:
 for name in ['map','map_front']:
  log=(root/(name+'-load.log')).open('w');process=subprocess.Popen(['ros2','run','nav2_map_server','map_server','--ros-args','-r','__node:=towergo_static_map_server','-p','yaml_filename:='+str(root/(name+'.yaml')),'-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
  try:
   for transition in [1,3]:
    req=ChangeState.Request();req.transition.id=transition;response=call(ChangeState,'/towergo_static_map_server/change_state',req)
    if not response.success:raise RuntimeError('Lifecycle transition failed')
   services=[name for name,types in node.get_service_names_and_types() if 'nav_msgs/srv/GetMap' in types]
   if len(services)!=1:raise RuntimeError('Expected one GetMap service: '+str(services))
   m=call(GetMap,services[0],GetMap.Request()).map
   counts={'occupied_cells':sum(x==100 for x in m.data),'free_cells':sum(x==0 for x in m.data),'unknown_cells':sum(x==-1 for x in m.data)}
   passed=all(counts[k]==expected['maps'][name][k] for k in counts) and [m.info.width,m.info.height]==expected['map_dimensions_cells'] and abs(m.info.resolution-.05)<1e-6
   result['maps'][name]={'passed':passed,'width':m.info.width,'height':m.info.height,**counts}
  finally:
   os.killpg(process.pid,signal.SIGINT);process.wait(timeout=10);log.close();time.sleep(1)
 result['passed']=all(x['passed'] for x in result['maps'].values()) and len(result['maps'])==2
except Exception as e:result['error']=str(e)
finally:
 rclpy.shutdown();thread.join(timeout=3);node.destroy_node()
(root/'load_validation.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
raise SystemExit(0 if result['passed'] else 1)
