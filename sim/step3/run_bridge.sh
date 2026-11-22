#!/bin/bash
# Run inside the course container after: ign gazebo -r -s bpearl_world.sdf
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /bpearl/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /thermal_camera@sensor_msgs/msg/Image[gz.msgs.Image
