# Step 2: thermal view and LiDAR-thermal fusion (simulation)

Status: maths tested offline (`python3 test_fusion_lib.py`); the ROS nodes are **not yet run** in the container.
Needs step 1 running (world + bridge). The step 1 world was changed so that all objects are inside the
thermal camera's ±30° field of view (heater moved to (4, 1.6), person_b to (5.5, -2.2)).

Copy `sim/step1` and `sim/step2` into `~/exercises_ws/src/thermal_lidar_sim/` (see `sim/step1/README.md`).

## Run (all in the course container, one terminal each, paste one line at a time)
1. `cd ~/exercises_ws/src/thermal_lidar_sim/step1 && ign gazebo -r -s thermal_lidar_world.sdf`
2. `~/exercises_ws/src/thermal_lidar_sim/step1/run_bridge.sh`
3. `ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 map "sensor_rig/rig_link/lidar_3d"`
4. `cd ~/exercises_ws/src/thermal_lidar_sim/step2 && source /opt/ros/humble/setup.bash && python3 thermal_view.py`
   - prints raw min/max and the implied temperature in K. Expected: about 293 K background and about 309 K / 330 K objects.
5. `cd ~/exercises_ws/src/thermal_lidar_sim/step2 && source /opt/ros/humble/setup.bash && python3 fusion_node.py`
6. `rviz2`: Fixed Frame `map`; add PointCloud2 `/fusion/points` with Color Transformer RGB8 (channel `rgb`);
   add Image `/thermal_camera/vis`.

## Expected
- Thermal image: person_a, person_b and the heater bright; ground and the cold box dark.
- `/fusion/points`: points on the two people green, on the heater red, others grey.
- If every temperature looks wrong, check the first `thermal_view.py` line: the K/count scale (default 0.01 from the gz-sensors source) may differ.
