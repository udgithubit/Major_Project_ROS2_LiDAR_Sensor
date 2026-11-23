#!/bin/bash
# One-command demo. Run in PLAIN WSL (prompt unmona_2@LAPTOP), not inside the container:
#     bash ~/Major_Project_ROS2_LiDAR_Sensor/sim/step3/start_demo.sh
# Restarts the container, starts world + bridge + fusion + object detection in the background, waits for the result,
# then opens RViz with demo.rviz. Closing RViz (or Ctrl+C here) stops everything.
C=robotics_essentials_ros2
STEP3=/home/user/exercises_ws/src/thermal_lidar_sim/step3      # path INSIDE the container

if ! docker info >/dev/null 2>&1; then
  echo "Docker is not running. Open Docker Desktop, wait for 'Engine running', then run this again."; exit 1
fi
if ! docker ps -a --format '{{.Names}}' | grep -qx "$C"; then
  echo "Container $C does not exist. Run:  cd ~/robotics_essentials_ros2/docker && docker compose up   (leave it open), then run this again."; exit 1
fi

stop_all() {
  trap - EXIT INT TERM
  echo; echo "Stopping the pipeline..."
  # the [x] trick keeps pkill from matching its own command line
  docker exec "$C" bash -c "pkill -f '[r]un_all.sh'; pkill -f '[p]arameter_bridge'; pkill -f '[i]gn gazebo'; pkill -f '[f]usion_node.py'; pkill -f '[c]luster_node.py'; pkill -f '[t]hermal_view.py'; pkill -f '[s]tatic_transform_publisher'; pkill -f '[r]viz2'" >/dev/null 2>&1
  echo "Done."; exit 0
}
trap stop_all EXIT INT TERM

echo "[1/4] Restarting the container (clears old processes)..."
docker restart "$C" >/dev/null || { echo "Could not restart the container."; exit 1; }
sleep 3
docker exec "$C" bash -c "rm -rf /tmp/thermal_lidar_logs"

echo "[2/4] Starting the pipeline in the background (this takes 1-2 minutes)..."
docker exec -d "$C" bash -c "START_THERMAL_VIEW=1 bash $STEP3/run_all.sh > /tmp/run_all.out 2>&1"

echo "[3/4] Waiting for the first result..."
ok=0
for i in $(seq 1 240); do
  if docker exec "$C" bash -c "grep -q 'persons called by' /tmp/thermal_lidar_logs/cluster.log 2>/dev/null"; then ok=1; break; fi
  printf "."; sleep 1
done
echo
if [ "$ok" != 1 ]; then
  echo "No result after 4 minutes. Last output:"; docker exec "$C" bash -c "tail -n 15 /tmp/run_all.out; tail -n 5 /tmp/thermal_lidar_logs/world.log 2>/dev/null"
  echo "Use your backup screenshot, or stop (Ctrl+C) and try again."; sleep 30; exit 1
fi
echo "Result:"
docker exec "$C" bash -c "grep -A8 'persons called by' /tmp/thermal_lidar_logs/cluster.log | tail -n 9"

echo "[4/4] Opening RViz. Close the RViz window (or press Ctrl+C) to stop everything."
docker exec "$C" bash -c "source /opt/ros/humble/setup.bash && rviz2 -d $STEP3/demo.rviz"
