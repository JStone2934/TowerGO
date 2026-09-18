#!/bin/bash
set -e
export ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.bash
source /towergo/install/setup.bash
exec "$@"
