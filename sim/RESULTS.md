# Simulation result: thermal-LiDAR fusion vs single-sensor baselines

Date: run in the course Docker container (ROS 2 Humble, Gazebo Fortress), CPU-only laptop. Code at commit 59c7e3b
(`sim/step3/run_all.sh`; `cluster_lib.py`, `fusion_lib.py`, `fusion_node.py`, `cluster_node.py` in `sim/step2`).

## Setup (controlled, static scene; simulation only)
- Sensor rig at 0.6 m height: RS-Bpearl-like 3D LiDAR (32 channels, dome axis pointing forward, 1.125 deg azimuth, 0.1-30 m)
  and a thermal camera (320x240, 60 deg horizontal FOV), same pose. Extrinsic: LiDAR rotated 90 deg (pitch) w.r.t. the camera.
- Objects: person_a, person_b (cylinders 1.7 m, 309 K); warm_box (0.4 x 0.4 x 0.5 m, 308 K); heater (0.5 x 0.5 x 0.8 m, 330 K);
  cold_box (0.8 x 0.8 x 1.0 m, ambient).
- Person rule (fusion): at least 50 % of the object's thermal points at 306-312 K AND height 0.9-2.3 m AND width 0.15-1.0 m.
  LiDAR-only baseline = shape rule only. Thermal-only baseline = temperature rule only.

## Result (one scan, repeated identically because the scene is static)
persons called by: LiDAR only = 3, thermal only = 3, fusion = 2 (truth = 2).
- LiDAR-only false positive: cold_box (person-sized, ambient temperature).
- Thermal-only false positive: warm_box (body temperature, only 0.5 m tall).
- Fusion: person_a (x 2.8, y 0.6, h 1.70 m) and person_b (x 5.3, y -2.2, h 1.66 m); no false positive.
- Heater (330 K) is rejected by temperature (`hot_object`).

## Limitations
- One static, hand-made scene designed to expose each sensor's failure; it is not a performance estimate.
- Thermal camera and LiDAR share one pose (no calibration error), perfect time sync, no noise.
- Only 36 % of the LiDAR points fall inside the thermal camera view (1813 of 5099); 3286 are outside it.
- Simple x-y grid clustering can split an object into fragments (a 38-point fragment of warm_box appears as `object`).
- LiDAR azimuth resolution is coarser than the real device (1.125 deg instead of 0.2 deg) so that the simulation runs on a CPU-only laptop.
