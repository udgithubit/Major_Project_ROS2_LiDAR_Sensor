#!/bin/bash
# Run inside the course container, after: ign gazebo -r thermal_lidar_world.sdf
# Bridges the Gazebo topics to ROS 2 (message names as used by the Humble ros_gz_bridge in the course image).
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  /lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /thermal_camera@sensor_msgs/msg/Image[gz.msgs.Image
