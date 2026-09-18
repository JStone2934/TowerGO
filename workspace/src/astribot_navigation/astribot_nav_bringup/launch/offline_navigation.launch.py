import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def check(context):
    if not Path(LaunchConfiguration('map').perform(context)).is_file(): raise RuntimeError('map must be an existing absolute YAML path')
    return []

def generate_launch_description():
    p=get_package_share_directory('astribot_nav_bringup'); config=os.path.join(p,'config','nav2.yaml')
    nodes=[DeclareLaunchArgument('map'),OpaqueFunction(function=check),IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(p,'launch','simulation.launch.py')),launch_arguments={'mapping':'false'}.items())]
    managed=[]
    for package,exe,name in [('nav2_map_server','map_server','map_server'),('nav2_amcl','amcl','amcl'),('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server'),('nav2_behaviors','behavior_server','behavior_server'),('nav2_bt_navigator','bt_navigator','bt_navigator')]:
        extra={}
        if name=='map_server': extra['yaml_filename']=LaunchConfiguration('map')
        if name=='bt_navigator':
            extra['default_nav_to_pose_bt_xml']=os.path.join(p,'config','navigation.xml')
            extra['default_nav_through_poses_bt_xml']=os.path.join(p,'config','navigation_through.xml')
            extra['plugin_lib_names']=['nav2_compute_path_through_poses_action_bt_node','nav2_compute_path_to_pose_action_bt_node','nav2_follow_path_action_bt_node','nav2_pipeline_sequence_bt_node','nav2_rate_controller_bt_node']
        nodes.append(Node(package=package,executable=exe,name=name,parameters=[config,extra],remappings=[('cmd_vel','/cmd_vel_nav')],output='screen'));managed.append(name)
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[{'use_sim_time':True,'autostart':True,'node_names':managed,'bond_timeout':4.0}],output='screen'))
    return LaunchDescription(nodes)
