# Running the simulation (WSL2, Ubuntu 22.04, ROS 2 Humble)

Tested on: Windows 11 + WSL2 (WSLg) + Ubuntu 22.04 + ROS 2 Humble + Gazebo 11 + Node 20.

## One-time setup
```bash
sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-robot-state-publisher ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-rosbridge-suite ros-humble-web-video-server ros-humble-teleop-twist-joy \
  ros-humble-teleop-twist-keyboard ros-humble-ament-lint-auto ros-humble-ament-lint-common \
  python3-colcon-common-extensions
# Node 20 (Ubuntu's default Node 12 is too old for Vite) via NodeSource, then:
git clone -b claude/dazzling-lovelace-mv0ger https://github.com/udgithubit/Major_Project_ROS2_LiDAR_Sensor.git
cd Major_Project_ROS2_LiDAR_Sensor && ./setup_workspace.sh
cd dashboard && npm install
```

## Run (one terminal each)
```bash
cd ~/ros_ws && ./launch.sh        # Gazebo supermarket + robot
cd ~/ros_ws && ./rosbridge.sh     # websocket, port 9090
cd ~/ros_ws && ./webvideo.sh      # camera stream, port 8080
cd ~/Major_Project_ROS2_LiDAR_Sensor/dashboard && npm run dev   # http://localhost:5173
ros2 run teleop_twist_keyboard teleop_twist_keyboard            # optional keyboard driving
```

## Known gaps
- `robot_core.xacro` was missing from the original upload and is a reconstruction.
- No thermal-sensor or fusion node, and no chat backend / MongoDB, are in the repo.
- If Gazebo shows an empty world, kill stale processes: `killall gzserver gzclient`.
