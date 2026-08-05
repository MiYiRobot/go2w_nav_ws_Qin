#!/bin/bash
set -e

WS="${GO2W_NAV_WS:-$HOME/go2w_nav_ws_Qin}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

export ROS_DOMAIN_ID=30
# export LD_LIBRARY_PATH="$WS/third_party/unitree_sdk2/install/lib:$LD_LIBRARY_PATH"

#订阅/cmd_vel速度指令并将它发送给机器人
ros2 run pointcloud_transformer cloud_transform_node