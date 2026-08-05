#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/go2w_nav_ws_Qin/install/setup.bash

export ROS_DOMAIN_ID=30

cd ~/go2w_nav_ws_Qin/rosbag

echo "Starting rosbag to record data..."
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "bag will save to: $(pwd)"

ros2 bag record -s mcap \
  /tf \
  /tf_static \
  /localization \
  /cloud_registered_map \
  /planning/bspline \
  /planned_path \
  /planned_path_marker \
  /cmd_vel \
  /start_navigation \
  /planning/go2_execution_frozen \
  /robot_description    \
  /a_star_list \
  /global_list \
  /global_point \
  /goal_pose \
  /grid_map/occupancy \
  /grid_map/occupancy_inflate \
  /grid_map/sensor_pose_extrinsic \
  /grid_map/sliding_map_bbox \
  /grid_map/unknown \
  /init_list \
  /initialpose \
  /joint_states \
  /optimal_list \
  /path  \
  /Odometry \
  /preblocked_cells_markers \
  /self_inflation \
  /start_point \
  /traversable_cells_markers \
  /planning/data_display