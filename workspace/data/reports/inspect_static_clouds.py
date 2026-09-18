from pathlib import Path
import json,numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2,Imu
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path('/home/astribot/TowerGO/workspace/data/bags/static_lidar_20260918_213221');out=Path('/home/astribot/TowerGO/workspace/data/maps/static_lidar_20260918_213221');out.mkdir(exist_ok=True)
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri=str(root/'bag'),storage_id='mcap'),rosbag2_py.ConverterOptions('',''));counts={};clouds={'front':[],'back':[]};frames={'front':[],'back':[]};imu={'front':[],'back':[]}
while r.has_next():
 topic,data,_=r.read_next();side='front' if topic.endswith('front') else 'back';counts[topic]=counts.get(topic,0)+1
 if '/lidar_' in topic:
  if counts[topic]%5:continue
  m=deserialize_message(data,PointCloud2);offset={f.name:f.offset for f in m.fields};endian='>' if m.is_bigendian else '<'
  xyz=np.stack([np.ndarray((m.height,m.width),dtype=endian+'f4',buffer=m.data,offset=offset[k],strides=(m.row_step,m.point_step)).ravel() for k in ['x','y','z']],axis=1)
  keep=np.isfinite(xyz).all(axis=1)&(np.linalg.norm(xyz,axis=1)>.5)&(np.linalg.norm(xyz,axis=1)<20)
  clouds[side].append(xyz[keep]);frames[side].append(np.full(keep.sum(),counts[topic],dtype=np.int16))
 else:
  m=deserialize_message(data,Imu);imu[side].append([m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z,m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z])
clouds={k:np.concatenate(v) for k,v in clouds.items()};frames={k:np.concatenate(v) for k,v in frames.items()};imu={k:np.array(v) for k,v in imu.items()};np.savez_compressed(out/'samples.npz',**clouds,front_frames=frames['front'],back_frames=frames['back'],imu_front=imu['front'],imu_back=imu['back'])
cfg=json.loads((root/'MID360_config.json').read_text())['lidar_configs'][1]['extrinsic_parameter'];rot=Rotation.from_euler('xyz',[cfg[x] for x in ['roll','pitch','yaw']],degrees=True).as_matrix();trans=np.array([cfg[x] for x in ['x','y','z']])/1000
front=clouds['front'][::10];back=clouds['back'][::10];tree=cKDTree(front);metrics={}
for name,b in [('as_recorded',back),('apply_config_again',back@rot.T+trans),('undo_config',(back-trans)@rot)]:
 dist,_=tree.query(b,workers=2);metrics[name]={'nn_distance_quantiles':np.percentile(dist,[10,25,50,75,90,95]).tolist(),'fraction_within_5cm':float(np.mean(dist<.05))}
report={'cloud_bounds':{k:np.percentile(v,[0,1,50,99,100],axis=0).tolist() for k,v in clouds.items()},'imu_mean':{k:v.mean(axis=0).tolist() for k,v in imu.items()},'imu_std':{k:v.std(axis=0).tolist() for k,v in imu.items()},'extrinsic_comparison':metrics};(out/'inspection.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
fig,axes=plt.subplots(1,3,figsize=(15,5))
for side,color in [('front','tab:blue'),('back','tab:orange')]:
 v=clouds[side][::25]
 for ax,(i,j) in zip(axes,[(0,1),(0,2),(1,2)]):ax.scatter(v[:,i],v[:,j],s=.15,alpha=.4,c=color,label=side);ax.set_aspect('equal');ax.grid(alpha=.2);ax.set_xlabel('xyz'[i]+' [m]');ax.set_ylabel('xyz'[j]+' [m]')
axes[0].legend();fig.tight_layout();fig.savefig(out/'inspection.png',dpi=140)
