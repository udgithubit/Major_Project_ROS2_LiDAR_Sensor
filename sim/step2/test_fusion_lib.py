"""Offline sanity test of the projection/classification maths (no ROS needed): python3 test_fusion_lib.py"""
import numpy as np
from fusion_lib import classify, project, OUTSIDE, AMBIENT, HUMAN, HOT

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
