#!/usr/bin/env python3
"""Host-only LiDAR capture. No Astribot SDK or chassis command interfaces."""
import os,time,json,signal,subprocess,threading,shutil,datetime
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,Imu
from std_srvs.srv import SetBool
if os.environ.get('ROS_DOMAIN_ID')!='25':raise RuntimeError('Expected physical sensor domain 25')
root=Path('/home/astribot/TowerGO/workspace/data/bags')/('static_lidar_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
root.mkdir(parents=True,exist_ok=False)
topics={'/livox/lidar_front':PointCloud2,'/livox/lidar_back':PointCloud2,'/livox/imu_front':Imu,'/livox/imu_back':Imu}
report={'path':str(root),'passed':False,'sdk_used':False,'chassis_commands_sent':False,'stationary_requested':True,'stationarity_independently_verified':False,'requested_recording_seconds':45,'topics':{}}
record=None;owned=False;log=None
rclpy.init(args=[]);node=Node('towergo_static_lidar_capture');thread=threading.Thread(target=rclpy.spin,args=(node,),daemon=True)
def receive(topic,msg):
 row=report['topics'].setdefault(topic,{'received':0,'frames':[]})
 row['received']+=1;stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
 row.setdefault('first_stamp',stamp);row['last_stamp']=stamp
 if msg.header.frame_id not in row['frames']:row['frames'].append(msg.header.frame_id)
 if isinstance(msg,PointCloud2):row['last_points']=msg.width*msg.height;row['fields']=[f.name for f in msg.fields]
for topic,kind in topics.items():node.create_subscription(kind,topic,lambda msg,t=topic:receive(t,msg),qos_profile_sensor_data)
client=node.create_client(SetBool,'/astribot_lidar_control');thread.start()
def control(value):
 if not client.wait_for_service(timeout_sec=15):raise RuntimeError('LiDAR service unavailable')
 req=SetBool.Request();req.data=value;future=client.call_async(req);deadline=time.monotonic()+20
 while not future.done() and time.monotonic()<deadline:time.sleep(.05)
 if not future.done():raise TimeoutError('LiDAR switch timed out')
 result=future.result()
 if not result.success:raise RuntimeError(result.message)
 return result.message
def driver_running():
 import psutil
 for proc in psutil.process_iter(['cmdline']):
  try:
   if any(Path(x).name=='livox_ros_driver2_node' for x in proc.info['cmdline'] or []):return True
  except (psutil.NoSuchProcess,psutil.AccessDenied):pass
 return False
try:
 report['driver_was_running']=driver_running()
 for name,src in {'MID360_config.json':'/opt/astribot_ros/software/livox_ros_driver2/share/config/MID360_config.json','driver_launch.py':'/opt/astribot_ros/software/livox_ros_driver2/share/livox_ros_driver2/msg_MID360_launch_rviz_msg_type.py'}.items():shutil.copy2(src,root/name)
 if not report['driver_was_running']:
  owned=True;report['activation']=control(True)
 deadline=time.monotonic()+40
 while not all(report['topics'].get(t,{}).get('received',0)>=3 for t in topics):
  if time.monotonic()>deadline:raise TimeoutError('All four LiDAR/IMU streams did not arrive')
  time.sleep(.1)
 qos=root/'qos.yaml';qos.write_text(''.join(t+':\n  reliability: best_effort\n  durability: volatile\n  history: keep_last\n  depth: 100\n' for t in topics))
 log=(root/'record.log').open('w');record=subprocess.Popen(['ros2','bag','record','-s','mcap','-o',str(root/'bag'),'--qos-profile-overrides-path',str(qos),*topics],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 begin=time.monotonic();print('CAPTURING',root,flush=True)
 while time.monotonic()-begin<47:
  if record.poll() is not None:raise RuntimeError('Recorder exited early')
  time.sleep(.2)
 report['recorder_wall_seconds']=time.monotonic()-begin
except Exception as e:report['error']=str(e)
finally:
 if record and record.poll() is None:
  os.killpg(record.pid,signal.SIGINT)
  try:record.wait(timeout=20)
  except subprocess.TimeoutExpired:
   os.killpg(record.pid,signal.SIGTERM);record.wait(timeout=5);report['error']='Recorder required forced termination'
 if log:log.close()
 if owned:
  try:report['deactivation']=control(False);time.sleep(3);report['driver_stopped']=not driver_running()
  except Exception as e:report['shutdown_error']=str(e)
 else:report['left_existing_driver_running']=True
 rclpy.shutdown();thread.join(timeout=3);node.destroy_node()
 (root/'capture_report.json').write_text(json.dumps(report,indent=2))
try:
 import rosbag2_py
 reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
 counts={t:0 for t in topics};first={};last={}
 while reader.has_next():
  topic,data,stamp=reader.read_next();counts[topic]=counts.get(topic,0)+1;first.setdefault(topic,stamp);last[topic]=stamp
 report['bag_counts']=counts;report['bag_durations_seconds']={t:(last[t]-first[t])/1e9 for t in first};report['bag_bytes']=sum(p.stat().st_size for p in (root/'bag').rglob('*') if p.is_file())
 report['passed']=not report.get('error') and not report.get('shutdown_error') and all(counts[t]>0 and report['bag_durations_seconds'].get(t,0)>=44 for t in topics) and (not owned or report.get('driver_stopped',False))
except Exception as e:report['validation_error']=str(e)
(root/'capture_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
raise SystemExit(0 if report['passed'] else 1)
