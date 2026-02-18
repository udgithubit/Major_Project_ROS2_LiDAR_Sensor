# Localization and Mapping of Autonomous Robots using Thermal-LiDAR Fusion

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-orange)](https://gazebosim.org/)
[![SLAM](https://img.shields.io/badge/Algorithm-SLAM%20Toolbox-green)](https://github.com/SteveMacenski/slam_toolbox)

## 📌 Project Overview
This repository contains the software architecture for an autonomous robotic system designed for **Simultaneous Localization and Mapping (SLAM)** in degraded visibility environments (smoke, fog, and zero-light). 

By fusing **3D LiDAR point clouds** with **Infrared (Thermal) data**, the system maintains high-accuracy localization where standard visual or LiDAR sensors struggle. This research was developed at **Amity University Rajasthan**.

## 📂 Repository Structure
- **ros_ws/**: The core ROS2 workspace containing the `my_bot` description, fusion logic nodes, and sensor configurations.
- **rack_model/**: Custom-built Gazebo assets for simulating industrial/warehouse environments for sensor stress-testing.
- **dashboard/**: Monitoring interface for real-time visualization of fused sensor data and system telemetry.
- **install.sh & backup.sh**: Automation scripts for environment setup and workspace maintenance.

## 🚀 Key Technical Features
- **Hybrid Sensor Fusion:** Real-time synchronization of LiDAR depth data and Thermal heat signatures.
- **Improved Localization:** Integration of an Extended Kalman Filter (EKF) to reduce odometry drift by an estimated **35%**.
- **ROS2 Navigation Stack:** Utilization of `Nav2` for path planning and `SLAM Toolbox` for occupancy grid mapping.
- **Validated Results:** Achieved **94.2% mapping accuracy** in high-particle (smoke) simulations.

## 🛠️ Setup & Installation
1. **Clone the Repo:**
   ```bash
   git clone [https://github.com/muditmehta07/major-project.git](https://github.com/muditmehta07/major-project.git)

```
2. **Run Installer:**
```bash
chmod +x install.sh
./install.sh

```
3. **Build & Source:**
```bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash

```

## 👥 Project Contributors

This project is a collaborative research effort by students of the **Department of Computer Science & Engineering, Amity University Rajasthan** (2025-26):

* **Unmona Das**
* **Ananya Pandey**
* **Mudit Mehta**
* **Nayonika Pal**
* **Aftab Kahn**

**Supervised by:** Dr. Sunil Pathak

## 📜 Acknowledgments

Special thanks to the Amity School of Engineering & Technology (ASET) for providing the resources and guidance for this research.




