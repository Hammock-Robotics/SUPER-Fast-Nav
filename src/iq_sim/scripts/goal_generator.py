#!/usr/bin/env python

import numpy as np
from geographiclib.geodesic import Geodesic
import math
from pathlib import Path
from typing import List, Tuple

# --- Configuration ---
HOME_LAT = -35.3632621
HOME_LON = 149.1652374
spacing = 9
rows = 5
TREE_COORDS = [(i*spacing, j*spacing) for i in range(rows) for j in range(rows) if not (i == 0 and j == 0)]
RADIUS = 1.0
NUM_POINTS = 4
ALTITUDE = 5.0
HOLD_TIME = 2.0
YAW_RATE = 30.0
OUTPUT_FILE = Path("multi_tree_orbit.waypoints")

Waypoint = Tuple[float, float, float, float]

def compute_orbit_waypoints(center: Tuple[float, float]) -> np.ndarray:
    cx, cy = center
    angles = np.linspace(0, 2*np.pi, NUM_POINTS, endpoint=False)
    dx = cx + RADIUS * np.cos(angles)
    dy = cy + RADIUS * np.sin(angles)
    # yaw: face toward center
    yaw = np.arctan2(cy - dy, cx - dx)
    alt = np.full(NUM_POINTS, ALTITUDE)
    return np.stack([dx, dy, alt, yaw], axis=1)  # shape (4,4)

# optimise heading to the nearest point on the next tree
def reorder_block(pts: np.ndarray, current_pos: Tuple[float, float]) -> np.ndarray:
    # pts: (N,2) dx,dy
    arr = pts[:, :2]
    cur = np.array(current_pos)
    d2 = np.sum((arr - cur)**2, axis=1)
    k = np.argmin(d2)
    return np.vstack((pts[k:], pts[:k]))

def main():
    # geodesic helper
    geo = Geodesic.WGS84

    # build full waypoint list
    mission_blocks = []
    current = (0.0, 0.0)
    for tree in TREE_COORDS:
        block = compute_orbit_waypoints(tree)  # (4,4)
        block = reorder_block(block, current)
        mission_blocks.append(block)
        # update current to last point's dx,dy
        current = tuple(block[-1, :2])
    mission = np.vstack(mission_blocks)  # shape (4*T,4)

    # write mission file
    with OUTPUT_FILE.open("w") as f:
        f.write("QGC WPL 110\n")
        # dummy home
        f.write(f"0\t1\t3\t16\t0\t0\t0\t0\t{HOME_LAT:.7f}\t{HOME_LON:.7f}\t{ALTITUDE:.3f}\t1\n")
        idx = 1
        for dx, dy, alt, yaw_rad in mission:
            # convert to lat/lon
            dist = math.hypot(dx, dy)
            az = (math.degrees(math.atan2(dx, dy))) % 360
            pt = geo.Direct(HOME_LAT, HOME_LON, az, dist)
            lat, lon = pt['lat2'], pt['lon2']
            yaw_deg = (math.degrees(yaw_rad) + 360) % 360
            # NAV_WAYPOINT
            f.write(f"{idx}\t{1 if idx==1 else 0}\t3\t16\t"
                    f"{HOLD_TIME:.1f}\t0\t0\t{yaw_deg:.1f}\t"
                    f"{lat:.7f}\t{lon:.7f}\t{alt:.3f}\t1\n")
            idx += 1
            # CONDITION_YAW
            f.write(f"{idx}\t0\t3\t115\t{yaw_deg:.1f}\t{YAW_RATE:.1f}\t0\t0\t0\t0\t0\t1\n")
            idx += 1

            print(f"Wrote goal: ({dx:.3f}, {dy:.3f}), heading: {yaw_deg:.1f} ")

        print(f"wrote {idx//2} goals to file {OUTPUT_FILE}")

if __name__ == "__main__":
    main()


