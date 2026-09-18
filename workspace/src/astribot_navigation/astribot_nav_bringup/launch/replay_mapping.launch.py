import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def start(context):
    if os.environ.get('ROS_DOMAIN_ID')!='125': raise RuntimeError('Replay requires the offline ROS domain 125')
    bag=LaunchConfiguration('bag').perform(context)
    if not Path(bag).exists(): raise RuntimeError('Bag path does not exist')
    if not (Path(bag)/'towergo_inputs.json').is_file(): raise RuntimeError('Run astribot_nav_bridge prepare_bag first to remove map TF and actuator commands')
    # Whitelist input topics. Never replay historical actuator commands or map->odom.
    topics=['/livox/lidar_front','/livox/lidar_back','/livox/imu_front','/livox/imu_back','/odom','/tf','/tf_static','/ground_truth/odom']
    return [ExecuteProcess(cmd=['ros2','bag','play',bag,'--clock','--rate','1.0','--topics']+topics,output='screen')]

def generate_launch_description():
    p=get_package_share_directory('astribot_nav_bringup')
    return LaunchDescription([DeclareLaunchArgument('bag'),DeclareLaunchArgument('scan_source',default_value='front'),
      Node(package='astribot_nav_sensors',executable='scan_processor',parameters=[{'use_sim_time':True,'scan_source':LaunchConfiguration('scan_source')}]),
      Node(package='slam_toolbox',executable='sync_slam_toolbox_node',name='slam_toolbox',parameters=[os.path.join(p,'config','slam.yaml')]),OpaqueFunction(function=start)])
