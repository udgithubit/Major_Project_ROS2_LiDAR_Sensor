# Handoff: SmartAssist → MSc thesis (thermal–LiDAR fusion)

Paste this file into a new chat and say: "Continue from this handoff. I want to work on <next step>."

## Who and what
- Student: Unmona Das (MSc, University of Eastern Finland, Robotics & AI Research Lab).
- Supervisors: Ilkka Jormanainen (lab lead) and Haris (co-supervisor, Karelia). Coordinator: Olli. Supervisors said they would inform the coordinator themselves.
- Background: bachelor's project "SmartAssist" (Amity University Rajasthan, 2025–26): autonomous shopping-cart robot using 2D LiDAR + simulated thermal sensing, ROS 2 Humble, Gazebo, SLAM Toolbox, Nav2, React dashboard over rosbridge. Published at ICNWC 2026; presented at ISDD 2026 (BITS Pilani). Results (95 % navigation success, 92 % human detection, 18 ms latency) are **simulation only**; the thermal sensor was simulated and the LiDAR was 2D.
- Thesis idea (agreed in the meeting, supervisors did not want to look at the old project): extend to a real **3D LiDAR + thermal camera**, evaluate human detection in low light / occlusion vs a 3D-LiDAR-only baseline.
- Draft documents already made (Word): thesis proposal/question sheet, and `Thesis_Scope_Document` (draft v1: 3 research questions, 5-phase plan, risks). Research questions: RQ1 calibration/time-sync of 3D LiDAR and thermal camera; RQ2 fusion vs LiDAR-only detection in low light/occlusion; RQ3 latency/compute on a mobile robot.
- Hardware status: lab has RoboSense RS-BPearl 360 and Ouster OS0 (3D LiDARs). Haris is checking Karelia's labs. Thermal module (~€150 excl. VAT) not yet available; budget with Markku Tukiainen. Student may use the Robotics Lab. Henki Robotics (Janne Karttunen) cannot supervise but offers technical advice (ROS 2, Nav2, slam-toolbox, Gazebo, 3D LiDARs incl. Ouster). Karelics: contact only after asking Ilkka.

## Machine and platforms
- Laptop: Lenovo IdeaPad Slim 5i, Windows 11, ~7.7 GB RAM (memory is tight; WSLg crashed once when running Gazebo + RViz together).
- **WSL2 Ubuntu 22.04**, user `unmona_2` (an unused Ubuntu 26.04 distro, user `unmona_das`, also exists; Humble does not support 26.04). Always check the prompt.
- ROS 2 Humble + Gazebo 11 (Classic) installed natively in WSL for the SmartAssist project. Node 20 for the dashboard.
- Docker Desktop (WSL integration enabled for Ubuntu-22.04) for the **course** (Henki Robotics `robotics_essentials_ros2`): container `robotics_essentials_ros2` with ROS 2 Humble + Gazebo Fortress + Andino robot. Prompt `user@docker-desktop` = inside container.
- Rule: `unmona_2@LAPTOP` = plain WSL (git, docker, file copying). `user@docker-desktop` = container (ros2, ign, rviz). Only one Gazebo at a time.
- Paste commands one at a time; a second pasted line is sometimes swallowed.

## Repository
- GitHub: `udgithubit/Major_Project_ROS2_LiDAR_Sensor`, working branch **`claude/dazzling-lovelace-mv0ger`** (no PR, not merged to main).
- The original upload was flat and incomplete. Added on the branch:
  - `ros_ws/robot_core.xacro` (reconstructed; original missing), `launch_sim.launch.py` now forwards a `world` argument, scripts without hard-coded paths.
  - `setup_workspace.sh` builds `~/ros_ws/src/my_bot` from the flat folder and installs the rack model locally (worlds rewritten from Fuel URLs to `model://my_store_rack/...`).
  - `dashboard/` restored to `src/` + `public/` layout.
  - `RUNNING.md` (tested setup/run steps), `HANDOFF.md` (this file), `sim/step1/` (below).
- Still missing from the repo (may be with teammate Mudit Mehta): thermal sensor and fusion node, chat backend/MongoDB, the original supermarket world with 68 racks, original `robot_core.xacro`.

## What works (verified on the laptop)
1. SmartAssist simulation: Gazebo supermarket world + robot with 2D LiDAR and camera (`./launch.sh`), rosbridge (port 9090), web_video_server (port 8080), React dashboard at http://localhost:5173 with live camera, robot position on map, teleop.
2. Course Exercise 0–2 environment: Docker container, Andino simulation, SLAM Toolbox mapping (`slam_toolbox_online_async.launch.py`), map saving, Nav2 with 2D Pose Estimate and Nav2 Goal (`andino_gz.launch.py nav2:=True`). Teleop via Gazebo's teleop panel.
3. **Thesis simulation step 1** (`sim/step1/`): Gazebo Fortress world with a static rig carrying a **16-ring 3D LiDAR (`gpu_lidar`) and a thermal camera** (L16), two warm "people" (309 K), a cold box, and a hot heater (330 K, a thermal false positive). Verified: topics `/lidar`, `/lidar/points`, `/thermal_camera` exist; `ros_gz_bridge` (`gz.msgs.*` names) delivers `PointCloud2` (640×16 = 10 240 points, 5 263 finite); visible in RViz.

## Known details and gotchas from step 1
- Run in the container: `ign gazebo -r -s thermal_lidar_world.sdf` (`-s` = no GUI, saves RAM), then `run_bridge.sh`, then RViz.
- The bridge turns the frame `sensor_rig::rig_link::lidar_3d` into **`sensor_rig/rig_link/lidar_3d`** (slashes). RViz needs a TF:
  `ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 map "sensor_rig/rig_link/lidar_3d"`, Fixed Frame `map`.
- RViz PointCloud2 topic must be `/lidar/points` (not `/lidar`, which is the 2D LaserScan). Style Points uses pixel size.
- Not yet verified: the thermal image in RViz (16-bit, will look dark; needs scaling), `/camera_info` for the thermal camera, `ignition_frame_id` to shorten the frame name.

## Next steps (suggested order)
1. Check the thermal image (`/thermal_camera`, mono16); write a scaling node.
2. Fix/shorten the sensor frame name in the world file.
3. Write the **fusion node**: project 3D LiDAR points into the thermal image (camera 320×240, hFOV 1.047 rad, identity extrinsic in sim), read temperature, label points human-like (≈306–310 K) vs not; heater should be non-human.
4. Put the rig on a moving robot; compare with a LiDAR-only baseline.
5. Literature review (thermal–LiDAR fusion, LiDAR–camera calibration, 3D human detection); verify references of the ICNWC paper.
6. Follow up with supervisors (hardware, next meeting); ask Mudit for the missing files; send the scope document to Ilkka and Haris.
7. Course: finish Exercise 2 (Nav2), then Exercise 3 (create a ROS 2 package) in `~/robotics_essentials_ros2`.

## Quick start commands
```bash
# Course container
cd ~/robotics_essentials_ros2/docker && docker compose up        # leave open
docker exec -it robotics_essentials_ros2 bash                    # each new container terminal
ros2 launch andino_gz andino_gz.launch.py                        # course simulation
# Thesis step 1 (in container, files under ~/exercises_ws/src/thermal_lidar_sim/step1)
ign gazebo -r -s thermal_lidar_world.sdf
./run_bridge.sh
rviz2
# SmartAssist (plain WSL)
cd ~/ros_ws && ./launch.sh ; ./rosbridge.sh ; ./webvideo.sh
cd ~/Major_Project_ROS2_LiDAR_Sensor/dashboard && npm run dev
```
