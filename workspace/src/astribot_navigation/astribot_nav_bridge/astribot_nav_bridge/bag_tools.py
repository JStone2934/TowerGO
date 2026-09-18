"""Prepare a replay-only bag, dropping commands and map localization transforms."""
import argparse,json
from pathlib import Path

INPUT_TOPICS={'/livox/lidar_front','/livox/lidar_back','/livox/imu_front','/livox/imu_back','/odom','/tf','/tf_static','/ground_truth/odom'}

def main():
    import rosbag2_py
    from rclpy.serialization import deserialize_message,serialize_message
    from tf2_msgs.msg import TFMessage
    from rosidl_runtime_py.utilities import get_message
    p=argparse.ArgumentParser();p.add_argument('input');p.add_argument('output');a=p.parse_args()
    if Path(a.output).exists():raise RuntimeError('Output bag must not exist')
    reader=rosbag2_py.SequentialReader();reader.open(rosbag2_py.StorageOptions(uri=a.input,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    metadata={m.name:m for m in reader.get_all_topics_and_types()}
    required={'/livox/lidar_front','/odom','/tf','/tf_static'}
    if not required<=metadata.keys():raise RuntimeError('Bag lacks required synthetic scan/odom/TF inputs')
    writer=rosbag2_py.SequentialWriter();writer.open(rosbag2_py.StorageOptions(uri=a.output,storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    for name,m in metadata.items():
        if name in INPUT_TOPICS:writer.create_topic(m)
    count=0;removed=0;edges=set();last_stamp=0
    classes={name:get_message(m.type) for name,m in metadata.items() if name in INPUT_TOPICS}
    while reader.has_next():
        topic,data,t=reader.read_next()
        if topic not in INPUT_TOPICS:continue
        if topic in ('/tf','/tf_static'):
            m=deserialize_message(data,TFMessage)
            keep=[]
            for tf in m.transforms:
                edge=(tf.header.frame_id,tf.child_frame_id)
                if edge in {('odom','base_link'),('base_link','lidar_front'),('base_link','lidar_back')}:
                    keep.append(tf);edges.add(edge)
                else:removed+=1
            if not keep:continue
            m.transforms=keep;data=serialize_message(m)
        message=deserialize_message(data,classes[topic])
        stamp=message.transforms[0].header.stamp if topic in ('/tf','/tf_static') else message.header.stamp
        ns=stamp.sec*10**9+stamp.nanosec
        if ns>0:last_stamp=ns
        writer.write(topic,data,ns if ns>0 else last_stamp);count+=1
    del writer
    manifest={'schema':1,'synthetic_only':True,'messages':count,'removed_tf':removed,'edges':sorted(edges),'has_back': '/livox/lidar_back' in metadata}
    (Path(a.output)/'towergo_inputs.json').write_text(json.dumps(manifest,indent=2))
    if not {('odom','base_link'),('base_link','lidar_front')}<=edges:raise RuntimeError('Missing expected transforms; bag not ready')
    print(json.dumps(manifest))
