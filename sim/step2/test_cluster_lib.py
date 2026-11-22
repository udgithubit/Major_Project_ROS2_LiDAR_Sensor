"""Offline test of the clustering/classification logic with a synthetic scene (no ROS): python3 test_cluster_lib.py"""
import numpy as np
from fusion_lib import OUTSIDE, AMBIENT, HUMAN, HOT
from cluster_lib import find_objects, PERSON, WARM_OBJECT, HOT_OBJECT, OBJECT, UNKNOWN

rng = np.random.default_rng(0)
H = 0.6   # sensor height; floor at z = -0.6


def cylinder(cx, cy, r, z0, z1, n=300):
    a = rng.uniform(0, 2 * np.pi, n)
    return np.column_stack([cx + r * np.cos(a), cy + r * np.sin(a), rng.uniform(z0, z1, n)])


def box(cx, cy, sx, sy, z0, z1, n=300):
    return np.column_stack([cx + rng.uniform(-sx / 2, sx / 2, n), cy + rng.uniform(-sy / 2, sy / 2, n), rng.uniform(z0, z1, n)])


parts = [
    (cylinder(3.0, 0.6, 0.22, -H, 1.1), HUMAN),     # person_a, 1.7 m tall, 309 K
    (cylinder(5.5, -2.2, 0.22, -H, 1.1), HUMAN),    # person_b
    (box(2.4, 0.0, 0.4, 0.4, -H, -0.1), HUMAN),     # warm_box, 0.5 m tall, 308 K  -> must NOT be a person
    (box(4.0, 1.6, 0.5, 0.5, -H, 0.2), HOT),        # heater, 330 K
    (box(3.5, -0.8, 0.8, 0.8, -H, 0.4), AMBIENT),   # cold box
    (cylinder(2.0, 4.5, 0.22, -H, 1.1), OUTSIDE),   # a person outside the thermal view -> unknown
]
xyz = np.vstack([p for p, _ in parts]).astype(np.float32)
labels = np.concatenate([np.full(len(p), l) for p, l in parts]).astype(np.uint8)
floor = np.column_stack([rng.uniform(1, 7, 2000), rng.uniform(-4, 5, 2000), np.full(2000, -H)]).astype(np.float32)
xyz = np.vstack([xyz, floor]); labels = np.concatenate([labels, np.full(2000, AMBIENT)]).astype(np.uint8)

objs = find_objects(xyz, labels, sensor_height=H)
for cls, d in sorted(objs, key=lambda o: o[1]['centroid'][0]):
    print(f"{cls:12s} at x={d['centroid'][0]:.1f} y={d['centroid'][1]:.1f}  n={d['n']:3d}  height={d['height']:.2f}  width={d['width']:.2f}  human={d['frac_human']:.2f} hot={d['frac_hot']:.2f}")
by_pos = {(round(float(d['centroid'][0])), round(float(d['centroid'][1]))): c for c, d in objs}
assert len(objs) == 6, len(objs)
assert by_pos[(3, 1)] == PERSON          # person_a
assert by_pos[(6, -2)] == PERSON         # person_b
assert by_pos[(2, 0)] == WARM_OBJECT     # warm_box rejected by shape
assert by_pos[(4, 2)] == HOT_OBJECT      # heater
assert by_pos[(4, -1)] == OBJECT         # cold box
assert by_pos[(2, 4)] == UNKNOWN or by_pos[(2, 5)] == UNKNOWN   # outside thermal view
print("ok: 2 persons, warm box rejected, heater hot, cold box object, outside-view person unknown")
