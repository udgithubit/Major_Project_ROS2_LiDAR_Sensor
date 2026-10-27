#!/bin/bash
# Usage: ./launch.sh [path/to/world]   (default: obstacle.world from the my_bot package)
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/install/setup.bash"
if [ -n "$1" ]; then
  ros2 launch my_bot launch_sim.launch.py world:="$1"
else
  ros2 launch my_bot launch_sim.launch.py
fi
