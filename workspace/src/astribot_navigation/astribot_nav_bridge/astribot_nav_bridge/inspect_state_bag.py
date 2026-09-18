"""Read-only offline parser. Never imports the hardware SDK or publishes ROS data."""
import argparse, json
from pathlib import Path
from .contracts import inspect_state

def main():
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    p=argparse.ArgumentParser(); p.add_argument('bag'); p.add_argument('--storage',default='mcap'); p.add_argument('--output',required=True); a=p.parse_args()
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=a.bag,storage_id=a.storage),rosbag2_py.ConverterOptions('',''))
    types={x.name:x.type for x in r.get_all_topics_and_types()}
    topic='/astribot_chassis/joint_space_states'
    if topic not in types: raise RuntimeError('Bag has no chassis state topic')
    cls=get_message(types[topic]); count=0; invalid=0
    with Path(a.output).open('w') as f:
        while r.has_next():
            name,data,t=r.read_next()
            if name!=topic: continue
            try: row=inspect_state(deserialize_message(data,cls))
            except ValueError as e: row={'error':str(e)}; invalid+=1
            row['record_timestamp_ns']=t; f.write(json.dumps(row)+'\n'); count+=1
    print(json.dumps({'count':count,'invalid':invalid,'semantics':'unconfirmed','published_tf':False}))
