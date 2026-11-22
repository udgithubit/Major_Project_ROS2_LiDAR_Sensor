#!/usr/bin/env python3
"""Step 3b: group the fused points into objects and decide which ones are people.

Subscribes: /fusion/points (x, y, z, rgb, label), published by fusion_node.py, in the rig frame
Publishes:  /fusion/objects (visualization_msgs/MarkerArray): one box + text per object
  green = person, orange = warm but not person-shaped, red = hot, grey = ambient object, blue = outside thermal view
Logs a table of the objects every two seconds. Rules and thresholds: see cluster_lib.py.
Run inside the course container:  python3 cluster_node.py
"""
import argparse
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from fusion_lib import cloud_to_xyzl
from cluster_lib import find_objects, DEFAULTS, PERSON, WARM_OBJECT, HOT_OBJECT, OBJECT, UNKNOWN

COLOR = {PERSON: (0.0, 1.0, 0.0), WARM_OBJECT: (1.0, 0.6, 0.0), HOT_OBJECT: (1.0, 0.0, 0.0),
         OBJECT: (0.6, 0.6, 0.6), UNKNOWN: (0.3, 0.3, 1.0)}


class ClusterNode(Node):
    def __init__(self, sensor_height, max_rate):
        super().__init__('thermal_lidar_objects')
        self.sensor_height = sensor_height
        self.min_dt = 1.0 / max_rate
        self.last = 0.0
        self.last_print = 0.0
        self.pub = self.create_publisher(MarkerArray, '/fusion/objects', 10)
        self.create_subscription(PointCloud2, '/fusion/points', self.on_cloud, 10)

    def on_cloud(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last < self.min_dt:      # python clustering: do not try to keep up with every scan
            return
        self.last = now
        xyz, labels = cloud_to_xyzl(msg)
        objs = find_objects(xyz, labels, sensor_height=self.sensor_height)
        objs.sort(key=lambda o: o[1]['centroid'][0])

        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (cls, d) in enumerate(objs):
            lo, hi = d['lo'], d['hi']
            r, g, b = COLOR[cls]
            box = Marker()
            box.header = msg.header
            box.ns, box.id, box.type, box.action = 'objects', i, Marker.CUBE, Marker.ADD
            c = (lo + hi) / 2.0
            box.pose.position.x, box.pose.position.y, box.pose.position.z = float(c[0]), float(c[1]), float(c[2])
            box.pose.orientation.w = 1.0
            s = np.maximum(hi - lo, 0.05)
            box.scale.x, box.scale.y, box.scale.z = float(s[0]), float(s[1]), float(s[2])
            box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, 0.35
            box.lifetime = Duration(seconds=1.0).to_msg()
            arr.markers.append(box)
            text = Marker()
            text.header = msg.header
            text.ns, text.id, text.type, text.action = 'labels', i, Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x, text.pose.position.y, text.pose.position.z = float(c[0]), float(c[1]), float(hi[2] + 0.2)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 1.0, 1.0, 1.0
            text.text = f"{cls} {d['height']:.1f}m"
            text.lifetime = Duration(seconds=1.0).to_msg()
            arr.markers.append(text)
        self.pub.publish(arr)

        if now - self.last_print > 2.0:
            self.last_print = now
            lines = [f"{cls:12s} x={d['centroid'][0]:.1f} y={d['centroid'][1]:.1f} n={d['n']:4d} "
                     f"h={d['height']:.2f} w={d['width']:.2f} human={d['frac_human']:.2f} hot={d['frac_hot']:.2f}"
                     for cls, d in objs]
            counts = {k: sum(1 for c, _ in objs if c == k) for k in (PERSON, WARM_OBJECT, HOT_OBJECT, OBJECT, UNKNOWN)}
            self.get_logger().info(f"objects={len(objs)} {counts}\n  " + "\n  ".join(lines))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sensor-height', type=float, default=DEFAULTS['sensor_height'],
                    help='height of the rig above the floor (m); must match the world (0.6)')
    ap.add_argument('--max-rate', type=float, default=5.0, help='process at most this many scans per second')
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = ClusterNode(args.sensor_height, args.max_rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
