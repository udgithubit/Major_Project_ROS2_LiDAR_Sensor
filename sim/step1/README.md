# Step 1: 3D LiDAR + thermal camera in Gazebo (Fortress)

Goal: confirm that a simulated 3D LiDAR and thermal camera publish data that reaches ROS 2.
Status: written from the official gz-sim examples, **not yet run** (needs testing in the course container).

## Run (inside the course Docker container)
Copy this folder to `~/exercises_ws/src/thermal_lidar_sim/step1` in WSL (the folder is mounted in the container as `/home/user/exercises_ws/src`).

Terminal 1 (container):
```bash
cd ~/exercises_ws/src/thermal_lidar_sim/step1
ign gazebo -r thermal_lidar_world.sdf
```
Terminal 2 (container), check the Gazebo side:
```bash
ign topic -l | grep -E "lidar|thermal"
ign topic -e -t /lidar | head -30
```
Terminal 3 (container), bridge to ROS 2:
```bash
~/exercises_ws/src/thermal_lidar_sim/step1/run_bridge.sh
```
Terminal 4 (container), check the ROS side and view it:
```bash
ros2 topic list | grep -E "lidar|thermal"
ros2 topic hz /lidar/points
rviz2     # Fixed Frame: set to the lidar frame name shown in the PointCloud2 header; add PointCloud2 (/lidar/points) and Image (/thermal_camera)
```

## What to expect
- 3D point cloud with 16 rings in front of the rig, showing two person-shaped cylinders, a box and a heater.
- Thermal image (16-bit, `mono16`): the two "people" and the heater appear bright, the cold box and ground appear dark.
- The heater is a deliberate thermal false positive: a hot object that is not a human. LiDAR geometry should help tell it apart.

## Things to check and report back
- Do the topic names `/lidar`, `/lidar/points` and `/thermal_camera` exist (`ign topic -l`)?
- Does the bridge accept `gz.msgs.PointCloudPacked` and `gz.msgs.Image` (L16)? If not, send the error.
- What frame id does the PointCloud2 use?
