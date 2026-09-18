#!/usr/bin/env python3
"""Replay pause/resume/restart check for a bag sanitized with prepare_bag."""
import argparse,json,os,signal,subprocess,threading,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosbag2_interfaces.srv import Pause,Resume
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy

p=argparse.ArgumentParser();p.add_argument('bag');p.add_argument('--output',required=True);a=p.parse_args()
if os.environ.get('ROS_DOMAIN_ID')!='125':raise RuntimeError('Offline domain required')
rclpy.init();n=Node('replay_check',parameter_overrides=[Parameter('use_sim_time',value=True)])
maps=[];n.create_subscription(OccupancyGrid,'/map',maps.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
t=threading.Thread(target=rclpy.spin,args=(n,),daemon=True);t.start();report={'passed':False,'runs':[]}
out=Path(a.output);out.mkdir(parents=True,exist_ok=True)
def wait(test,seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:
        if test():return True
        time.sleep(.05)
    return False
def call(kind,name):
    c=n.create_client(kind,name)
    if not c.wait_for_service(timeout_sec=10):raise RuntimeError(name+' missing')
    f=c.call_async(kind.Request())
    if not wait(f.done,5):raise RuntimeError(name+' timeout')
    f.result();n.destroy_client(c)
try:
    for repeat in range(2):
        maps.clear()
        with (out/f'replay-{repeat}.log').open('w') as log:
            process=subprocess.Popen(['ros2','launch','astribot_nav_bringup','replay_mapping.launch.py','bag:='+a.bag],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                if not wait(lambda:len(maps)>0,30):raise RuntimeError('No map from replay')
                call(Pause,'/rosbag2_player/pause');time.sleep(.3);before=n.get_clock().now().nanoseconds;time.sleep(.5)
                paused=before==n.get_clock().now().nanoseconds
                call(Resume,'/rosbag2_player/resume')
                resumed=wait(lambda:n.get_clock().now().nanoseconds>before,5)
                if not paused or not resumed:raise RuntimeError('Pause/resume failed')
                report['runs'].append({'restart_index':repeat,'pause_ok':paused,'resume_ok':resumed,'observed_cells':sum(x>=0 for x in maps[-1].data)})
            finally:
                os.killpg(process.pid,signal.SIGINT)
                try:process.wait(timeout=20)
                except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
                time.sleep(1.)
    report['passed']=len(report['runs'])==2
except Exception as e:report['error']=str(e)
finally:
    rclpy.shutdown();t.join(timeout=3);n.destroy_node();(out/'report.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report));raise SystemExit(0 if report['passed'] else 1)
