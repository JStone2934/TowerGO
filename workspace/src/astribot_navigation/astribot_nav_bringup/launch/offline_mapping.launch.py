import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    p=get_package_share_directory('astribot_nav_bringup')
    return LaunchDescription([
      IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(p,'launch','simulation.launch.py')),launch_arguments={'mapping':'true'}.items()),
      Node(package='slam_toolbox',executable='sync_slam_toolbox_node',name='slam_toolbox',parameters=[os.path.join(p,'config','slam.yaml')],output='screen')])
