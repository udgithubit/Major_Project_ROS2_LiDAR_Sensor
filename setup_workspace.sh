#!/bin/bash
# Builds a colcon workspace (default ~/ros_ws) from the flat ros_ws/ folder of this repo.
set -e
REPO="$(cd "$(dirname "$0")" && pwd)"
WS="${1:-$HOME/ros_ws}"
PKG="$WS/src/my_bot"
mkdir -p "$PKG"/{config,description,launch,worlds}
cd "$REPO/ros_ws"
cp CMakeLists.txt package.xml "$PKG/"
cp *.xacro "$PKG/description/"
cp launch_sim.launch.py rsp.launch.py "$PKG/launch/"
cp *.world "$PKG/worlds/"
cp my_controllers.yaml mapper_params_online_async.yaml slam_localization.yaml my_map.yaml *.rviz "$PKG/config/"
cp ../dashboard/my_map.pgm "$PKG/config/" 2>/dev/null || true
cp launch.sh slam.sh rosbridge.sh webvideo.sh controller_teleop.sh "$WS/"
chmod +x "$WS"/*.sh
# Local rack model (the worlds originally downloaded the mesh from Ignition Fuel, which is unreliable)
RACK="$HOME/.gazebo/models/my_store_rack"
rm -rf "$RACK" "$HOME/.gazebo/models/rack_model"
mkdir -p "$RACK/meshes"
cp "$REPO/rack_model/model.config" "$REPO/rack_model/model.sdf" "$REPO/rack_model/thumbnail.png" "$RACK/" 2>/dev/null || true
cp "$REPO/rack_model/base_link.STL" "$RACK/meshes/"
# Point the installed worlds at the local mesh
sed -i 's#https://fuel.ignitionrobotics.org/1.0/sanjay/models/supermarket_rack/1/files/meshes/base_link.STL#model://my_store_rack/meshes/base_link.STL#g' "$PKG"/worlds/*.world
source /opt/ros/humble/setup.bash
cd "$WS" && colcon build --symlink-install
echo "Done. Run:  cd $WS && ./launch.sh"
