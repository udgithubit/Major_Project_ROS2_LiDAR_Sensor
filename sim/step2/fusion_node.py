#!/usr/bin/env python3
"""Thermal-LiDAR fusion, step 2: colour every LiDAR point by the temperature seen at its pixel.

Subscribes: /lidar/points (PointCloud2, sensor frame)  and  /thermal_camera (Image, mono16)
Publishes:  /fusion/points (PointCloud2 with x,y,z,rgb)
  green = human-like temperature (default 306-312 K), red = hotter than human (e.g. the heater),
  grey = ambient, blue = outside the thermal camera's field of view.
In simulation the LiDAR and the thermal camera share the same pose, so the extrinsic is the identity.
On real hardware this is where the calibration transform (RQ1 of the thesis) goes.
The newest thermal image is used for each cloud (fine for the static test scene; proper time sync comes later).
Run inside the course container:  python3 fusion_node.py
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from fusion_lib import classify, colorize, OUTSIDE, AMBIENT, HUMAN, HOT


class Fusion(Node):
    def __init__(self, a):
        super().__init__('thermal_lidar_fusion')
        self.a = a
        self.thermal = None
        self.pub = self.create_publisher(PointCloud2, '/fusion/points', 10)
        self.create_subscription(Image, '/thermal_camera', self.on_thermal, 10)
        self.create_subscription(PointCloud2, '/lidar/points', self.on_cloud, 10)
        self.last_print = 0.0

    def on_thermal(self, msg):
        dtype = '>u2' if msg.is_bigendian else '<u2'
        self.thermal = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width).copy()

    def on_cloud(self, msg):
        if self.thermal is None:
            return
        pts = pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if isinstance(pts, np.ndarray) and pts.dtype.names:
            xyz = np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float32)
        else:
            xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        if len(xyz) == 0:
            return
        labels, temp = classify(xyz, self.thermal, self.a.hfov, self.a.kelvin_per_count,
                                self.a.human_min_k, self.a.human_max_k)

        out = np.zeros(len(xyz), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])
        out['x'], out['y'], out['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        out['rgb'] = colorize(labels)
        cloud = PointCloud2()
        cloud.header = msg.header
        cloud.height, cloud.width = 1, len(xyz)
        cloud.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                        for n, o in (('x', 0), ('y', 4), ('z', 8), ('rgb', 12))]
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = 16 * len(xyz)
        cloud.is_dense = True
        cloud.data = out.tobytes()
        self.pub.publish(cloud)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_print > 2.0:
            self.last_print = now
            c = {n: int((labels == l).sum()) for n, l in
                 (('outside_fov', OUTSIDE), ('ambient', AMBIENT), ('human_like', HUMAN), ('hot', HOT))}
            self.get_logger().info(f'points={len(xyz)} {c}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hfov', type=float, default=1.047, help='thermal camera horizontal FOV (rad), from the SDF')
    ap.add_argument('--kelvin-per-count', type=float, default=0.01)
    ap.add_argument('--human-min-k', type=float, default=306.0)
    ap.add_argument('--human-max-k', type=float, default=312.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = Fusion(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
