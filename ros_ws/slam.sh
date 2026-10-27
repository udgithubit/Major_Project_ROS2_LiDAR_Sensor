#!/bin/bash
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"
CFG="$(ros2 pkg prefix my_bot)/share/my_bot/config"
ros2 launch slam_toolbox localization_launch.py map_file:=$CFG/my_map.yaml params_file:=$CFG/slam_localization.yaml use_sim_time:=true base_frame:=base_link odom_frame:=odom
