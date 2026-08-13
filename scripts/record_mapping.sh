#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/go2w_nav_ws_Qin/install/setup.bash

export ROS_DOMAIN_ID=30

cd ~/go2w_nav_ws_Qin/rosbag

echo "Starting rosbag to record data..."
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "bag will save to: $(pwd)"

ros2 bag record -s mcap -o mid360_mapping \
  /livox/lidar \
  /livox/imu