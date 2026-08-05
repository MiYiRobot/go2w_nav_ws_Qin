#!/bin/bash
set -e

WS="${GO2W_NAV_WS:-$HOME/go2w_nav_ws_Qin}"

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

export GO2W_NAV_WS="$WS" 

export ROS_DOMAIN_ID=30

cd "$WS"

ros2 launch octo_planner web_test.launch.py launch_controller:=false
# ros2 launch octo_planner web_test.launch.py launch_controller:=true
