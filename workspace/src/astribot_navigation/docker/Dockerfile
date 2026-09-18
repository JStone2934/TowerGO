ARG BASE_IMAGE=ros:humble-ros-base
FROM ${BASE_IMAGE}
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive ROS_DOMAIN_ID=125 ROS_LOCALHOST_ONLY=1
RUN apt-get update && apt-get install -y --no-install-recommends \
  python3-colcon-common-extensions python3-pytest python3-numpy python3-yaml python3-scipy \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-tf2-sensor-msgs ros-humble-sensor-msgs-py ros-humble-pointcloud-to-laserscan \
  ros-humble-rosbag2-storage-mcap ros-humble-rosbag2-py ros-humble-diagnostic-msgs \
  && rm -rf /var/lib/apt/lists/*
WORKDIR /towergo
COPY . /towergo/src/astribot_navigation
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install --parallel-workers 2 \
  && dpkg-query -W > /towergo/dependency-versions.txt
COPY docker/entrypoint.sh /towergo-entrypoint.sh
RUN chmod +x /towergo-entrypoint.sh
ENTRYPOINT ["/towergo-entrypoint.sh"]
CMD ["bash"]
