# Step 3: RS-Bpearl-like LiDAR (dome facing forward) + thermal camera + extrinsic transform

Status: written from the RS-Bpearl user manual and the step 1 working world; **not yet run in Gazebo**.
Offline tests of the maths: `python3 ../step2/test_fusion_lib.py`.

What changed compared with step 1/2
- LiDAR: 32 channels, vertical angles 2.3125..89.5 deg (even spacing as an approximation), 360 deg horizontal
  at 1.125 deg (320 samples = 10 240 points per scan; use 1800 for the real 0.2 deg). With 900 samples the cloud arrived at only about 1 Hz on the CPU-only test laptop, range 0.1..30 m, 10 Hz.
- The dome axis (sensor z) points along the rig's +x (forward): sensor pose pitch = 90 deg.
  The point cloud is in the LiDAR frame, so `fusion_node.py` must rotate it into the camera frame:
  `--lidar-rpy 0 1.5708 0` (this is the extrinsic calibration; on real hardware it must be estimated).
- New test object `warm_box`: 308 K (body temperature) but only 0.5 m tall and box-shaped. Thermal alone labels it
  human-like; geometry (next step: clustering) should reject it. The heater (330 K) is the hot false positive.

## Easiest: one terminal (course container)
`bash ~/exercises_ws/src/thermal_lidar_sim/step3/run_all.sh` starts world, bridge, transform and fusion node, prints the
fusion counts, and Ctrl+C stops everything. (`START_THERMAL_VIEW=1 bash .../run_all.sh` also publishes `/thermal_camera/vis`.)
Then, in a second container terminal: `source /opt/ros/humble/setup.bash && rviz2`.

## Manual (one terminal each, one line at a time)
1. `cd ~/exercises_ws/src/thermal_lidar_sim/step3 && ign gazebo -r -s bpearl_world.sdf`
2. `ign topic -l | grep -E "bpearl|thermal"`   (expect /bpearl/points and /thermal_camera)
3. `~/exercises_ws/src/thermal_lidar_sim/step3/run_bridge.sh`
4. `ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 map rig`
5. `cd ~/exercises_ws/src/thermal_lidar_sim/step2 && source /opt/ros/humble/setup.bash && python3 thermal_view.py`  (optional)
6. `cd ~/exercises_ws/src/thermal_lidar_sim/step2 && source /opt/ros/humble/setup.bash && python3 fusion_node.py --cloud-topic /bpearl/points --lidar-rpy 0 1.5708 0 --output-frame rig`
7. `rviz2`: Fixed Frame `map`; PointCloud2 `/fusion/points`, Color Transformer RGB8, Style Flat Squares, Size 0.05.

## Things to check and report back
- Does Gazebo accept the 90 deg vertical range and 900x32 rays, and how fast does it run (real-time factor)?
  If it is still slow, lower `<samples>` of the horizontal scan further (for example 240) and tell me.
- Are the people green and the heater red as before, and is `warm_box` (308 K) green too? (It should be: that is the thermal-only false positive to fix next.)
- Does the cloud sit in front of the rig (dome forward) and cover the floor?

## Step 3b: objects and people (cluster_node.py, started by run_all.sh)
`cluster_node.py` groups the fused points into objects in the x-y plane (floor removed) and classifies each:
- `person`: at least half of its thermal points are human-like (306-312 K) AND height 0.9-2.3 m AND width <= 1.0 m
- `warm_object`: human temperature but not person-shaped (the 308 K `warm_box` must end up here)
- `hot_object` (the heater), `object` (ambient), `unknown` (fewer than 5 points inside the thermal view)
It logs a table every 2 s and publishes boxes + labels on `/fusion/objects` (RViz: Add -> By topic -> /fusion/objects -> MarkerArray).
Offline test: `python3 ../step2/test_cluster_lib.py`.
Expected in the simulation: 2 persons (person_a, person_b), warm_object (warm_box), hot_object (heater), object (cold box).
