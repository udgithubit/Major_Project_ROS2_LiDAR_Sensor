#!/usr/bin/env python3
"""Make the 16-bit thermal image viewable and print its temperature statistics.

Subscribes: /thermal_camera (sensor_msgs/Image, mono16, raw counts; Gazebo default 0.01 K per count)
Publishes:  /thermal_camera/vis (mono8, scaled between --tmin and --tmax Kelvin) for the RViz Image display
Run inside the course container:  python3 thermal_view.py
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ThermalView(Node):
    def __init__(self, kelvin_per_count, tmin, tmax):
        super().__init__('thermal_view')
        self.k = kelvin_per_count
        self.tmin, self.tmax = tmin, tmax
        self.pub = self.create_publisher(Image, '/thermal_camera/vis', 10)
        self.create_subscription(Image, '/thermal_camera', self.cb, 10)
        self.last_print = 0.0

    def cb(self, msg):
        dtype = '>u2' if msg.is_bigendian else '<u2'
        raw = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width)
        kelvin = raw.astype(np.float32) * self.k
        vis = np.clip((kelvin - self.tmin) / (self.tmax - self.tmin), 0.0, 1.0)
        out = Image()
        out.header = msg.header
        out.height, out.width = msg.height, msg.width
        out.encoding = 'mono8'
        out.step = msg.width
        out.data = (vis * 255).astype(np.uint8).tobytes()
        self.pub.publish(out)

        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_print > 2.0:
            self.last_print = now
            self.get_logger().info(
                f'encoding={msg.encoding} size={msg.width}x{msg.height} raw min/max={raw.min()}/{raw.max()} '
                f'-> {kelvin.min():.1f}..{kelvin.max():.1f} K (assuming {self.k} K/count)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kelvin-per-count', type=float, default=0.01)
    ap.add_argument('--tmin', type=float, default=285.0)
    ap.add_argument('--tmax', type=float, default=335.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = ThermalView(args.kelvin_per_count, args.tmin, args.tmax)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
