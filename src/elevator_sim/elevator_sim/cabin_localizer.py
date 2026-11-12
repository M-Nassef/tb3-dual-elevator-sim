"""Localize a robot inside an elevator cabin from one LiDAR scan (pure numpy, no ROS).

AMCL is unreliable inside the small, almost symmetric cabin and does not update while
the robot stands still during a ride. The cabin geometry is known exactly, so we fit
the scan to the cabin's three walls (back wall + two side walls) with point-to-line
ICP (Gauss-Newton), starting from the current AMCL estimate.
"""
import math

import numpy as np


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, sensor_pose=(0.0, 0.0, 0.0)):
    """LaserScan -> Nx2 points in the robot base frame. sensor_pose = (x, y, yaw) of the LiDAR."""
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r, a = r[ok], a[ok] + sensor_pose[2]
    return np.column_stack((sensor_pose[0] + r * np.cos(a), sensor_pose[1] + r * np.sin(a)))


def fit_cabin_pose(points, guess, interior, gate=0.25, iterations=15, min_inliers=40):
    """Fit robot pose (x, y, yaw) in the map frame.

    points:   Nx2 scan points in the robot frame
    guess:    initial (x, y, yaw), e.g. from AMCL
    interior: (x_min, x_max, y_min, y_max) of the cabin's free interior; walls are the
              lines x = x_min (back), y = y_min and y = y_max (sides). x_max is the open front.
    Returns (pose, info) with info = {'inliers', 'rms', 'ok', 'walls'}.
    """
    x0, x1, y0, y1 = interior
    pose = np.array(guess, dtype=float)
    pts = np.asarray(points, dtype=float)
    info = {'inliers': 0, 'rms': float('inf'), 'ok': False, 'walls': 0}
    if len(pts) == 0:
        return tuple(pose), info
    for it in range(iterations):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        pm = pts @ np.array([[c, s], [-s, c]]) + pose[:2]           # robot -> map
        # candidate residuals to each wall, only for points lying along that wall segment
        d_back = np.where((pm[:, 1] > y0 - 0.05) & (pm[:, 1] < y1 + 0.05), pm[:, 0] - x0, np.inf)
        along_side = (pm[:, 0] > x0 - 0.05) & (pm[:, 0] < x1 - 0.05)
        d_lo = np.where(along_side, pm[:, 1] - y0, np.inf)
        d_hi = np.where(along_side, pm[:, 1] - y1, np.inf)
        D = np.column_stack((d_back, d_lo, d_hi))
        k = np.argmin(np.abs(D), axis=1)
        r = D[np.arange(len(D)), k]
        g = max(gate * (0.6 ** it), 0.04)                           # shrinking gate
        m = np.abs(r) < g
        if m.sum() < 6:
            break
        J = np.zeros((m.sum(), 3))
        kk, pmm = k[m], pm[m]
        xw = kk == 0                                                 # x = const wall
        J[xw, 0] = 1.0
        J[xw, 2] = -(pmm[xw, 1] - pose[1])
        J[~xw, 1] = 1.0
        J[~xw, 2] = pmm[~xw, 0] - pose[0]
        delta, *_ = np.linalg.lstsq(J, -r[m], rcond=None)
        pose += delta
        info.update(inliers=int(m.sum()), rms=float(np.sqrt(np.mean(r[m] ** 2))),
                    walls=int(len(set(kk.tolist()))))
        if np.linalg.norm(delta[:2]) < 1e-4 and abs(delta[2]) < 1e-4:
            break
    pose[2] = math.atan2(math.sin(pose[2]), math.cos(pose[2]))
    inside = x0 < pose[0] < x1 and y0 < pose[1] < y1
    info['ok'] = bool(info['inliers'] >= min_inliers and info['rms'] < 0.04 and info['walls'] >= 2 and inside)
    return tuple(pose), info
