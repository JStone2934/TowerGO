#!/usr/bin/env python3
"""Offline stationary point-cloud mapping; no ROS node, SDK, TF or motion output."""
import argparse, hashlib, json, math
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def extract(capture, stride):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(capture/'bag'), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    clouds = {'front': [], 'back': []}; frames = {'front': [], 'back': []}; counts = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in ('/livox/lidar_front', '/livox/lidar_back'): continue
        counts[topic] = counts.get(topic, 0) + 1
        if counts[topic] % stride: continue
        side = topic.rsplit('_', 1)[1]; m = deserialize_message(data, PointCloud2)
        if m.header.frame_id != 'livox_frame': raise ValueError('Unexpected sensor frame')
        offsets = {f.name: f.offset for f in m.fields}
        xyz = np.stack([np.ndarray((m.height, m.width), dtype=('>' if m.is_bigendian else '<')+'f4', buffer=m.data, offset=offsets[k], strides=(m.row_step, m.point_step)).ravel() for k in ('x','y','z')], axis=1)
        clouds[side].append(xyz); frames[side].append(np.full(len(xyz), counts[topic], np.int16))
    return {s: (np.concatenate(clouds[s]), np.concatenate(frames[s])) for s in clouds}, counts


def voxelize(points, frames, size=.03, min_bins=3):
    grid = np.floor(points/size).astype(np.int64) + 2048
    if np.any((grid < 0) | (grid >= 4096)): raise ValueError('Voxel index exceeds supported range')
    keys = (grid[:,0]*4096+grid[:,1])*4096+grid[:,2]
    unique, inv, counts = np.unique(keys, return_inverse=True, return_counts=True)
    temporal = np.unique(keys*16 + np.minimum(frames//50, 15))
    stable_keys, support = np.unique(temporal//16, return_counts=True)
    assert np.array_equal(unique, stable_keys)
    means = np.stack([np.bincount(inv, weights=points[:,i])/counts for i in range(3)], axis=1)
    keep = (support >= min_bins) & (counts >= 3)
    return means[keep], {'input_points':len(points),'all_voxels':len(unique),'stable_voxels':int(keep.sum())}


def floor_plane(points):
    # Prefer a large near-horizontal plane below the LiDAR, never a tabletop.
    p = points[(points[:,2] > -.8) & (points[:,2] < .12) & (np.linalg.norm(points[:,:2],axis=1) < 9)]
    rng = np.random.default_rng(42)
    if len(p) > 20000: p = p[rng.choice(len(p), 20000, replace=False)]
    if len(p) < 100: raise ValueError('Insufficient floor evidence')
    best = None; score = 0
    for _ in range(350):
        a,b,c = p[rng.choice(len(p),3,replace=False)]; n = np.cross(b-a,c-a)
        if np.linalg.norm(n) < .1: continue
        n /= np.linalg.norm(n)
        if n[2] < 0: n = -n
        if n[2] < math.cos(math.radians(10)): continue
        d = -n@a
        if not .08 < d < .8: continue
        hits = np.abs(p@n+d) < .025; count = int(hits.sum())
        if count > score: best = (n,d,hits); score = count
    if best is None: raise ValueError('No plausible floor plane')
    n,d,hits = best
    for _ in range(3):
        q = p[hits]; center = q.mean(axis=0); _,_,vh = np.linalg.svd(q-center, full_matrices=False)
        n = vh[-1]
        if n[2] < 0: n = -n
        d = -n@center; hits = np.abs(p@n+d) < .025
    z = np.array([0.,0.,1.]); v = np.cross(n,z); c = n@z
    skew = np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    rot = np.eye(3)+skew+skew@skew/(1+c)
    return rot, float(d), {'normal':n.tolist(),'offset_m':float(d),'inlier_points':int(hits.sum()),'tested_points':len(p),'inlier_abs_residual_p95_m':float(np.percentile(np.abs(p[hits]@n+d),95)),'leveling_angle_deg':float(np.degrees(np.arccos(n[2])))}


def write_pcd(path, xyz):
    xyz = np.asarray(xyz,dtype='<f4')
    header = f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n'
    path.write_bytes(header.encode()+xyz.tobytes())


def grid_map(clouds, origins, sides, resolution, bounds):
    low, high = bounds; shape = np.ceil((high-low)/resolution).astype(int)+1
    occupied = np.zeros((shape[1],shape[0]),bool); free = occupied.copy(); endpoints = []
    # 0.15-1.60m above measured floor: a local obstacle projection, not clearance certification.
    selected_clouds = {}
    for side in sides:
        p = clouds[side]; p = p[(p[:,2]>=.15)&(p[:,2]<=1.60)]
        selected_clouds[side] = p
        cells = np.floor((p[:,:2]-low)/resolution).astype(int)
        occupied[cells[:,1],cells[:,0]] = True
    for side in sides:
        p = selected_clouds[side]
        vec = p[:,:2]-origins[side][:2]; ranges = np.linalg.norm(vec,axis=1)
        bins = np.floor((np.arctan2(vec[:,1],vec[:,0])+np.pi)/(2*np.pi)*1440).astype(int)%1440
        order = np.lexsort((ranges,bins)); _, first = np.unique(bins[order],return_index=True)
        selected = order[first]
        for index in selected:
            end = p[index,:2]; start = origins[side][:2]; distance = ranges[index]
            # Keep a near-sensor blind zone unknown and stop before the measured obstacle.
            sample = np.arange(.60,max(.60,distance-resolution),resolution/2)
            xy = start + sample[:,None]*(end-start)/distance
            ij = np.floor((xy-low)/resolution).astype(int)
            # Never clear beyond any obstacle column, even when that column
            # was seen at another height/angle or by the other sensor.
            blocked = np.flatnonzero(occupied[ij[:,1],ij[:,0]])
            if len(blocked): ij = ij[:blocked[0]]
            free[ij[:,1],ij[:,0]] = True
        endpoints.append(len(selected))
    # Endpoints from either sensor always override free-space rays.
    image = np.full(occupied.shape,205,np.uint8); image[free]=254; image[occupied]=0
    return image, {'occupied_cells':int(occupied.sum()),'free_cells':int((free&~occupied).sum()),'unknown_cells':int((~free&~occupied).sum()),'rays_per_sensor':endpoints}


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--capture',required=True);ap.add_argument('--output',required=True);ap.add_argument('--resolution',type=float,default=.05);ap.add_argument('--max-range',type=float,default=15);ap.add_argument('--frame-stride',type=int,default=5);args=ap.parse_args()
    capture=Path(args.capture);out=Path(args.output);out.mkdir(parents=True,exist_ok=True)
    if (out/'map.yaml').exists(): raise ValueError('Refusing to overwrite an existing map')
    config=json.loads((capture/'MID360_config.json').read_text()); e=config['lidar_configs'][1]['extrinsic_parameter']; origins={'front':np.zeros(3),'back':np.array([e[k] for k in ('x','y','z')])/1000.}
    raw,counts=extract(capture,args.frame_stride);stable={};statistics={};alignment={}
    for side,(points,frames) in raw.items():
        distance=np.linalg.norm(points-origins[side],axis=1)
        keep=np.isfinite(points).all(axis=1)&(distance>=.6)&(distance<=args.max_range)
        stable[side],statistics[side]=voxelize(points[keep],frames[keep]);statistics[side]['sampled_raw_points']=len(points);statistics[side]['range_or_invalid_removed']=int((~keep).sum())
        # Compare early/late readings without fitting any trajectory or registration.
        early=points[keep&(frames<=100)][::5];late=points[keep&(frames>=350)][::5]
        dist,_=cKDTree(late).query(early,workers=2)
        statistics[side]['early_to_late_nn_quantiles_m']=np.percentile(dist,[50,90,95]).tolist()
    tree=cKDTree(stable['front']);back=stable['back'];rr=Rotation.from_euler('xyz',[e[k] for k in ('roll','pitch','yaw')],degrees=True).as_matrix()
    for name,p in [('as_recorded',back),('duplicate_transform',back@rr.T+origins['back']),('inverse_transform',(back-origins['back'])@rr)]:
        d,_=tree.query(p,workers=2);alignment[name]={'fraction_within_5cm':float(np.mean(d<.05)),'nn_quantiles_m':np.percentile(d,[10,25,50,75,90]).tolist()}
    rot,height,floor=floor_plane(np.concatenate(list(stable.values())))
    leveled={k:v@rot.T+np.array([0,0,height]) for k,v in stable.items()};sensor_origins={k:v@rot.T+np.array([0,0,height]) for k,v in origins.items()}
    all_points=np.concatenate(list(leveled.values()));write_pcd(out/'map_3d.pcd',all_points)
    write_pcd(out/'front_3d.pcd',leveled['front']);write_pcd(out/'back_3d.pcd',leveled['back'])
    obstacles=all_points[(all_points[:,2]>=.15)&(all_points[:,2]<=1.60)]
    low=np.floor((obstacles[:,:2].min(axis=0)-.5)/args.resolution)*args.resolution;high=np.ceil((obstacles[:,:2].max(axis=0)+.5)/args.resolution)*args.resolution
    maps={};grids={}
    for name,sides in [('map',['front','back']),('map_front',['front'])]:
        grid,stats=grid_map(leveled,sensor_origins,sides,args.resolution,(low,high));grids[name]=grid;maps[name]=stats
        Image.fromarray(np.flipud(grid)).save(out/(name+'.pgm'))
        (out/(name+'.yaml')).write_text(f'image: {name}.pgm\nmode: trinary\nresolution: {args.resolution}\norigin: [{low[0]:.6f}, {low[1]:.6f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    tf=np.eye(4);tf[:3,:3]=rot;tf[2,3]=height
    report={'kind':'stationary_local_map','bag':str(capture/'bag'),'source_cloud_frame':'livox_frame','map_frame':'towergo_static_map','chassis_control_used':False,'slam_or_trajectory_estimation_used':False,'motion_assumption':'Rigidly stationary during capture; no odometry supplied.','extrinsic_policy':'Coordinates kept as recorded; rear transform not applied a second time. This is not an extrinsic calibration certificate.','resolution_m':args.resolution,'max_sensor_range_m':args.max_range,'voxel_size_m':.03,'minimum_temporal_bins':3,'temporal_bin_seconds':5,'frame_stride':args.frame_stride,'obstacle_height_above_floor_m':[.15,1.60],'sensor_blind_radius_m':.6,'cloud_message_counts':counts,'statistics':statistics,'extrinsic_comparison':alignment,'floor_plane':floor,'map_from_livox_frame':tf.tolist(),'sensor_origins_in_map':{k:v.tolist() for k,v in sensor_origins.items()},'maps':maps,'map_dimensions_cells':[grids['map'].shape[1],grids['map'].shape[0]],'bounds_m':[low.tolist(),high.tolist()],'map_3d_points':len(all_points),'navigation_ready':False,'limitations':['Single stationary viewpoint; occlusions and unobserved areas remain unknown.','No chassis frame, footprint or odometry calibration.','Projected rays are a 2D visibility approximation, not proof of traversability.','Small or moving obstacles can be removed by temporal filtering.']}
    report['input_hashes']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [capture/'MID360_config.json',capture/'driver_launch.py',capture/'bag/metadata.yaml']}
    (out/'map_report.json').write_text(json.dumps(report,indent=2))
    fig,axes=plt.subplots(1,2,figsize=(14,7),layout='constrained');extent=[low[0],low[0]+grids['map'].shape[1]*args.resolution,low[1],low[1]+grids['map'].shape[0]*args.resolution]
    for ax,name,title in zip(axes,['map_front','map'],['Front LiDAR baseline','Dual LiDAR local map']):
        ax.imshow(grids[name],origin='lower',extent=extent,cmap='gray',vmin=0,vmax=255,interpolation='nearest')
        for side,color in [('front','#0077b6'),('back','#e85d04')]:
            o=sensor_origins[side];ax.plot(o[0],o[1],'o',c=color,ms=5,label=side)
        ax.set_title(title);ax.set_xlabel('X [m]');ax.set_ylabel('Y [m]');ax.set_aspect('equal');ax.legend(loc='upper right');ax.grid(alpha=.12)
    fig.suptitle('Stationary capture | 0.05 m/cell | black: obstacles, white: observed rays, gray: unknown',fontsize=12)
    fig.savefig(out/'map_preview.png',dpi=150);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,8),layout='constrained');p=all_points[(all_points[:,2]>=.1)&(all_points[:,2]<2.0)][::2]
    sc=ax.scatter(p[:,0],p[:,1],c=p[:,2],s=.5,cmap='viridis',vmin=0,vmax=2,rasterized=True);fig.colorbar(sc,ax=ax,label='Height above fitted floor [m]');ax.set_aspect('equal');ax.set_xlabel('X [m]');ax.set_ylabel('Y [m]');ax.set_title('Static 3D cloud: top view, floor and ceiling hidden');ax.grid(alpha=.2);fig.savefig(out/'pointcloud_preview.png',dpi=150);plt.close(fig)
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
