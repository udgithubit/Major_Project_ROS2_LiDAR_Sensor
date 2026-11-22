"""Group fused points into objects and decide which are people (pure numpy, no ROS imports).

Input: points in the rig frame (x forward, y left, z up), with a label per point from fusion_lib
(OUTSIDE=0 not in thermal view, AMBIENT=1, HUMAN=2 human-like temperature, HOT=3 hotter than a human).
Idea: thermal says "warm", geometry says "person-shaped". A person needs both.
"""
import numpy as np
from fusion_lib import OUTSIDE, AMBIENT, HUMAN, HOT

PERSON, WARM_OBJECT, HOT_OBJECT, OBJECT, UNKNOWN = 'person', 'warm_object', 'hot_object', 'object', 'unknown'

DEFAULTS = dict(
    sensor_height=0.6,      # height of the rig above the floor (m); the floor is at z = -sensor_height
    ground_margin=0.15,     # points lower than floor + margin are treated as floor and dropped (m)
    cell=0.12,              # clustering grid cell in the x-y plane (m); links points closer than about 0.2 m
    min_points=8,           # smaller clusters are ignored as noise
    min_thermal_points=5,   # need at least this many points inside the thermal view to judge temperature
    thermal_fraction=0.5,   # share of thermal points that must be human-like (or hot)
    person_min_height=0.9, person_max_height=2.3, person_max_width=1.0,   # metres
)


def cluster_xy(xyz, cell, min_points):
    """Connected components of occupied x-y grid cells (8-neighbourhood). Heights are ignored so vertical objects stay in one piece."""
    if len(xyz) == 0:
        return []
    ij = np.floor(xyz[:, :2] / cell).astype(np.int64)
    cells = {}
    for k, key in enumerate(map(tuple, ij)):
        cells.setdefault(key, []).append(k)
    seen, clusters = set(), []
    nbrs = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]
    for start in cells:
        if start in seen:
            continue
        seen.add(start)
        stack, members = [start], []
        while stack:
            c = stack.pop()
            members.extend(cells[c])
            for dx, dy in nbrs:
                nb = (c[0] + dx, c[1] + dy)
                if nb in cells and nb not in seen:
                    seen.add(nb)
                    stack.append(nb)
        if len(members) >= min_points:
            clusters.append(np.array(members))
    return clusters


def describe(xyz, labels, idx, sensor_height):
    p, lab = xyz[idx], labels[idx]
    lo, hi = p.min(axis=0), p.max(axis=0)
    thermal = lab != OUTSIDE
    n_th = int(thermal.sum())
    return dict(
        n=len(idx), n_thermal=n_th, centroid=p.mean(axis=0), lo=lo, hi=hi,
        height=float(hi[2] + sensor_height),                   # top above the floor (floor is at -sensor_height)
        width=float(max(hi[0] - lo[0], hi[1] - lo[1])),
        frac_human=float((lab == HUMAN).sum()) / n_th if n_th else 0.0,
        frac_hot=float((lab == HOT).sum()) / n_th if n_th else 0.0,
    )


def classify_cluster(d, prm):
    if d['n_thermal'] < prm['min_thermal_points']:
        return UNKNOWN                                    # mostly outside the thermal view: cannot judge temperature
    if d['frac_hot'] >= prm['thermal_fraction']:
        return HOT_OBJECT
    if d['frac_human'] >= prm['thermal_fraction']:
        shape_ok = (prm['person_min_height'] <= d['height'] <= prm['person_max_height']
                    and d['width'] <= prm['person_max_width'])
        return PERSON if shape_ok else WARM_OBJECT        # human temperature but not person-shaped
    return OBJECT


def find_objects(xyz, labels, **overrides):
    """Returns a list of (class, description) for every object-like cluster above the floor."""
    prm = dict(DEFAULTS, **overrides)
    above = xyz[:, 2] > (-prm['sensor_height'] + prm['ground_margin'])
    xyz, labels = xyz[above], labels[above]
    out = []
    for idx in cluster_xy(xyz, prm['cell'], prm['min_points']):
        d = describe(xyz, labels, idx, prm['sensor_height'])
        out.append((classify_cluster(d, prm), d))
    return out


def shape_is_person(d, prm):
    """LiDAR-only baseline: person-shaped (height and width), temperature ignored."""
    return prm['person_min_height'] <= d['height'] <= prm['person_max_height'] and d['width'] <= prm['person_max_width']


def temp_is_human(d, prm):
    """Thermal-only baseline: enough thermal points and mostly human temperature, shape ignored."""
    return d['n_thermal'] >= prm['min_thermal_points'] and d['frac_human'] >= prm['thermal_fraction']


def baseline_counts(objs, **overrides):
    """Number of objects each approach calls a person: LiDAR only, thermal only, fusion (both)."""
    prm = dict(DEFAULTS, **overrides)
    return dict(lidar_only=sum(shape_is_person(d, prm) for _, d in objs),
                thermal_only=sum(temp_is_human(d, prm) for _, d in objs),
                fusion=sum(1 for c, _ in objs if c == PERSON))
