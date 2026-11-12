"""Pure-numpy helpers for thermal-LiDAR fusion (no ROS imports, so they can be unit-tested)."""
import math
import numpy as np

# labels
OUTSIDE, AMBIENT, HUMAN, HOT = 0, 1, 2, 3
COLORS = {  # r, g, b
    OUTSIDE: (60, 60, 200),
    AMBIENT: (150, 150, 150),
    HUMAN: (0, 255, 0),
    HOT: (255, 0, 0),
}


def intrinsics(width, height, hfov):
    """Pinhole intrinsics (square pixels) from image size and horizontal field of view."""
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def project(xyz, width, height, hfov, min_depth=0.1):
    """Project points given in the camera/sensor frame into the image.

    Gazebo camera frame: x forward, y left, z up.  Returns (u, v, valid) as arrays.
    """
    fx, fy, cx, cy = intrinsics(width, height, hfov)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    ahead = x > min_depth
    safe_x = np.where(ahead, x, 1.0)
    u = cx - fx * y / safe_x
    v = cy - fy * z / safe_x
    valid = ahead & (u >= 0) & (u < width) & (v >= 0) & (v < height)
    return u, v, valid


def classify(xyz, thermal_raw, hfov, kelvin_per_count=0.01, human_min_k=306.0, human_max_k=312.0):
    """Label each point: OUTSIDE (not in thermal view), AMBIENT, HUMAN (human-like temperature), HOT."""
    h, w = thermal_raw.shape
    u, v, valid = project(xyz, w, h, hfov)
    labels = np.full(len(xyz), OUTSIDE, dtype=np.uint8)
    temp_k = np.full(len(xyz), np.nan, dtype=np.float32)
    ui = np.clip(np.floor(u[valid]).astype(int), 0, w - 1)
    vi = np.clip(np.floor(v[valid]).astype(int), 0, h - 1)
    t = thermal_raw[vi, ui].astype(np.float32) * kelvin_per_count
    temp_k[valid] = t
    lab = np.where(t > human_max_k, HOT, np.where(t >= human_min_k, HUMAN, AMBIENT))
    labels[valid] = lab
    return labels, temp_k


def colorize(labels):
    """Pack labels into float32 'rgb' values as used by RViz (0x00RRGGBB viewed as float32)."""
    rgb = np.zeros(len(labels), dtype=np.uint32)
    for lab, (r, g, b) in COLORS.items():
        rgb[labels == lab] = (r << 16) | (g << 8) | b
    return rgb.view(np.float32)
