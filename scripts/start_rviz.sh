#!/bin/bash
set -e

WS="${GO2W_NAV_WS:-$HOME/go2w_nav_ws_Qin}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

MAP_PCD="$HOME/go2w_nav_ws_Qin/maps/localization/big_localization_raw.pcd"

export ROS_DOMAIN_ID=30

cd "$WS"

ros2 launch scan_planner rviz.launch.py 

