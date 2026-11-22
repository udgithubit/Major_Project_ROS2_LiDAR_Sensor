# Demo cheat sheet (thermal-LiDAR fusion, step 3)

Prerequisite (once, plain WSL `unmona_2@LAPTOP`): `git pull origin claude/dazzling-lovelace-mv0ger` and
`cp -r sim/step1 sim/step2 sim/step3 ~/exercises_ws/src/thermal_lidar_sim/`.

1. Windows: open Docker Desktop, wait for "Engine running".
2. Ubuntu 22.04 terminal: `docker start robotics_essentials_ros2`
3. Terminal 1: `docker exec -it robotics_essentials_ros2 bash`, then
   `START_THERMAL_VIEW=1 bash ~/exercises_ws/src/thermal_lidar_sim/step3/run_all.sh`
   Wait for the `persons called by: {...}` lines (about 1-2 minutes).
4. Terminal 2: `docker exec -it robotics_essentials_ros2 bash`, then
   `source /opt/ros/humble/setup.bash && rviz2 -d ~/exercises_ws/src/thermal_lidar_sim/step3/demo.rviz`
   (if the config does not load: Fixed Frame `map`; add PointCloud2 `/fusion/points` with RGB8, Flat Squares, 0.05;
   MarkerArray `/fusion/objects`; Image `/thermal_camera/vis`.)
5. Expected: green boxes `person` x2, `warm_object`, `hot_object`, `object`; terminal 1 shows
   `persons called by: {'lidar_only': 3, 'thermal_only': 3, 'fusion': 2}`.
6. Stop: Ctrl+C in terminal 1 (stops everything), close RViz.
