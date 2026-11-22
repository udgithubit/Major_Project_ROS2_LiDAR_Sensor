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


def rotation_from_rpy(roll, pitch, yaw):
    """SDF/ROS convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def transform_points(xyz, rotation, translation):
    """Express points given in the LiDAR frame in the camera frame: p_cam = R @ p_lidar + t.

    (rotation, translation) is the pose of the LiDAR in the camera frame, i.e. the extrinsic calibration.
    """
    return (xyz @ np.asarray(rotation, dtype=np.float32).T + np.asarray(translation, dtype=np.float32)).astype(np.float32)


_DTYPES = {1: 'i1', 2: 'u1', 3: '<i2', 4: '<u2', 5: '<i4', 6: '<u4', 7: '<f4', 8: '<f8'}


def cloud_to_xyz(msg):
    """Fast PointCloud2 -> (N, 3) float32 array of finite points (little-endian clouds)."""
    dtype = np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [_DTYPES[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    xyz = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float32)
    return xyz[np.isfinite(xyz).all(axis=1)]


def cloud_to_xyzl(msg):
    """PointCloud2 with x,y,z,label fields -> (xyz (N,3) float32, label (N,) uint8), finite points only."""
    dtype = np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [_DTYPES[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    arr = np.frombuffer(bytes(msg.data), dtype=dtype)
    xyz = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float32)
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok], arr['label'][ok].astype(np.uint8)
