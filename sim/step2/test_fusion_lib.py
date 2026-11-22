"""Offline sanity test of the projection/classification maths (no ROS needed): python3 test_fusion_lib.py"""
import numpy as np
import types
from fusion_lib import classify, project, OUTSIDE, AMBIENT, HUMAN, HOT, rotation_from_rpy, transform_points, cloud_to_xyz

W, H, HFOV = 320, 240, 1.047
# thermal image: 293 K everywhere, a 309 K blob and a 330 K blob (raw counts at 0.01 K per count)
img = np.full((H, W), 29300, dtype=np.uint16)

# Points in the sensor frame (x forward, y left, z up)
pts = np.array([
    [3.0, 0.6, 0.0],    # where person_a is (left of centre)  -> should project at u ~ 105
    [4.0, 1.6, 0.0],    # heater
    [3.5, -0.8, 0.0],   # cold box
    [2.5, -2.0, 0.0],   # outside the +-30 degree FOV
    [-1.0, 0.0, 0.0],   # behind the sensor
], dtype=np.float32)

u, v, valid = project(pts, W, H, HFOV)
print("u", np.round(u, 1), "v", np.round(v, 1), "valid", valid)
assert valid.tolist() == [True, True, True, False, False]
assert abs(u[0] - 104.6) < 1.0

# paint warm blobs around the projected pixels of person (309 K) and heater (330 K)
for idx, kelvin in ((0, 309.0), (1, 330.0)):
    ui, vi = int(u[idx]), int(v[idx])
    img[max(vi - 5, 0):vi + 6, max(ui - 5, 0):ui + 6] = int(kelvin / 0.01)

labels, temp = classify(pts, img, HFOV)
print("labels", labels.tolist(), "temps", np.round(temp, 1))
assert labels.tolist() == [HUMAN, HOT, AMBIENT, OUTSIDE, OUTSIDE]
print("ok")


# --- extrinsic: a LiDAR whose z axis (dome axis) points along the camera's forward (x) axis, as in the Bpearl-forward mount
R = rotation_from_rpy(0.0, np.pi / 2, 0.0)
dome_axis = np.array([[0.0, 0.0, 1.0]], dtype=np.float32)
out = transform_points(dome_axis, R, [0, 0, 0])
print("dome axis in camera frame", np.round(out, 3))
assert np.allclose(out, [[1.0, 0.0, 0.0]], atol=1e-6)
# a point 3 m along the lidar's own horizon (+x_lidar) ends up below the camera (-z)
assert np.allclose(transform_points(np.array([[3.0, 0.0, 0.0]], dtype=np.float32), R, [0, 0, 0]), [[0.0, 0.0, -3.0]], atol=1e-5)

# --- cloud_to_xyz on a synthetic PointCloud2-like message (x,y,z float32 + intensity + ring uint16, point_step 32)
pts32 = np.zeros(3, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('pad', '<f4'), ('intensity', '<f4'), ('ring', '<u2'), ('pad2', 'V10')])
pts32['x'], pts32['y'], pts32['z'] = [1.0, np.inf, 3.0], [2.0, 0.0, 4.0], [3.0, 0.0, 5.0]
fields = [types.SimpleNamespace(name=n, offset=o, datatype=d) for n, o, d in
          (('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('intensity', 16, 7), ('ring', 24, 4))]
msg = types.SimpleNamespace(fields=fields, point_step=32, data=pts32.tobytes())
got = cloud_to_xyz(msg)
print("cloud_to_xyz", got.tolist())
assert got.tolist() == [[1.0, 2.0, 3.0], [3.0, 4.0, 5.0]]
print("ok (extrinsic + parsing)")
