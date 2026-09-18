import json,bisect
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Imu
root=Path('/home/astribot/TowerGO/workspace/data/bags/static_lidar_20260918_213221')
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));result={};times={}
while r.has_next():
 topic,data,_=r.read_next();cloud='/lidar_' in topic;m=deserialize_message(data,PointCloud2 if cloud else Imu)
 t=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;times.setdefault(topic,[]).append(t)
 row=result.setdefault(topic,{'messages':0,'invalid_xyz_points':0,'total_points':0,'empty_messages':0});row['messages']+=1
 if cloud:
  offset={f.name:f.offset for f in m.fields};endian='>' if m.is_bigendian else '<'
  arrays=[np.ndarray((m.height,m.width),dtype=endian+'f4',buffer=m.data,offset=offset[k],strides=(m.row_step,m.point_step)) for k in ['x','y','z']]
  finite=np.isfinite(arrays[0])&np.isfinite(arrays[1])&np.isfinite(arrays[2]);row['total_points']+=int(finite.size);row['invalid_xyz_points']+=int(finite.size-finite.sum());row['empty_messages']+=int(finite.size==0)
for topic,stamps in times.items():
 delta=np.diff(stamps);result[topic].update(header_duration_seconds=stamps[-1]-stamps[0],nonincreasing_timestamps=int(np.sum(delta<=0)),maximum_interval_seconds=float(delta.max()),mean_hz=float(len(delta)/(stamps[-1]-stamps[0])))
f=times['/livox/lidar_front'];b=times['/livox/lidar_back'];nearest=[]
for t in f:
 i=bisect.bisect_left(b,t);nearest.append(min(abs(t-b[j]) for j in [i-1,i] if 0<=j<len(b)))
quality={'topics':result,'front_to_nearest_back_time_delta_ms_p95':float(np.percentile(nearest,95)*1000),'front_frames_without_back_within_20ms':sum(x>.02 for x in nearest),'note':'Timestamp alignment only; not proof of extrinsic calibration or chassis stationarity.'}
(root/'quality_report.json').write_text(json.dumps(quality,indent=2));print(json.dumps(quality,indent=2))
