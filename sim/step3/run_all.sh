#!/bin/bash
# Start the whole step 3 pipeline in ONE terminal (run inside the course container). Ctrl+C stops everything.
#   world (Gazebo server) -> ros_gz bridge -> static transform map->rig -> fusion node (-> optional thermal_view)
# Logs: /tmp/thermal_lidar_logs/*.log   Set START_THERMAL_VIEW=1 to also publish /thermal_camera/vis for RViz.
source /opt/ros/humble/setup.bash
DIR="$(cd "$(dirname "$0")" && pwd)"
STEP2="$DIR/../step2"
LOG=/tmp/thermal_lidar_logs
mkdir -p "$LOG"
PIDS=()
cleanup() { trap - INT TERM EXIT; echo; echo "Stopping..."; kill "${PIDS[@]}" 2>/dev/null; sleep 1; pkill -f parameter_bridge 2>/dev/null; pkill -f "ign gazebo" 2>/dev/null; exit 0; }
trap cleanup INT TERM EXIT

# remove leftovers from earlier runs so nothing is published twice
pkill -f parameter_bridge 2>/dev/null; pkill -f "ign gazebo" 2>/dev/null; pkill -f fusion_node.py 2>/dev/null; pkill -f cluster_node.py 2>/dev/null
pkill -f static_transform_publisher 2>/dev/null; pkill -f thermal_view.py 2>/dev/null
sleep 2

echo "[1/4] starting Gazebo world (server only)..."
ign gazebo -r -s "$DIR/bpearl_world.sdf" > "$LOG/world.log" 2>&1 &
PIDS+=($!)
for i in $(seq 1 120); do
  if ign topic -l 2>/dev/null | grep -q "/bpearl/points"; then break; fi
  sleep 1
done
if ! ign topic -l 2>/dev/null | grep -q "/bpearl/points"; then
  echo "World did not publish /bpearl/points within 2 minutes. See $LOG/world.log"; exit 1
fi
echo "      world is up"

echo "[2/4] starting bridge..."
ros2 run ros_gz_bridge parameter_bridge \
  /bpearl/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /thermal_camera@sensor_msgs/msg/Image[gz.msgs.Image > "$LOG/bridge.log" 2>&1 &
PIDS+=($!)

echo "[3/4] publishing static transform map -> rig..."
ros2 run tf2_ros static_transform_publisher 0 0 0.6 0 0 0 map rig > "$LOG/tf.log" 2>&1 &
PIDS+=($!)

if [ "${START_THERMAL_VIEW:-0}" = "1" ]; then
  (cd "$STEP2" && python3 thermal_view.py > "$LOG/thermal_view.log" 2>&1) &
  PIDS+=($!)
fi

echo "[4/4] starting fusion node..."
(cd "$STEP2" && python3 fusion_node.py --cloud-topic /bpearl/points --lidar-rpy 0 1.5708 0 --output-frame rig > "$LOG/fusion.log" 2>&1) &
PIDS+=($!)

echo "      starting object detection (clustering) node..."
(cd "$STEP2" && python3 cluster_node.py > "$LOG/cluster.log" 2>&1) &
PIDS+=($!)

echo
echo "Running. In another container terminal start rviz2 (Fixed Frame: map, PointCloud2 /fusion/points, Color Transformer RGB8)."
echo "Also add a MarkerArray display on /fusion/objects (boxes + labels). Output below (Ctrl+C stops everything):"
sleep 3
tail -f "$LOG/cluster.log" "$LOG/fusion.log"
