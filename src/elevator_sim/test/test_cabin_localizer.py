"""Cabin localizer: synthetic scans of the cabin walls, recovered from a wrong initial guess."""
import math
import os

import numpy as np
import pytest

from elevator_sim.building import Building
from elevator_sim.cabin_localizer import fit_cabin_pose, scan_points

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'building.yaml')


def raycast(pose, interior, n=360, max_range=3.5, noise=0.01, rng=None):
    """Ranges seen from pose inside a 3-walled box open towards +x (x_max side)."""
    x0, x1, y0, y1 = interior
    px, py, th = pose
    out = []
    for i in range(n):
        a = th + 2 * math.pi * i / n
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        if dx < 0:
            t = (x0 - px) / dx
            if y0 <= py + t * dy <= y1:
                best = min(best, t)
        for yw in (y0, y1):
            if abs(dy) > 1e-9:
                t = (yw - py) / dy
                if t > 0 and x0 <= px + t * dx <= x1:
                    best = min(best, t)
        if best > max_range:
            best = math.inf
        elif rng is not None:
            best += rng.normal(0, noise)
        out.append(best)
    return out


@pytest.fixture(scope='module')
def b():
    return Building.load(CFG)


@pytest.mark.parametrize('eid', ['e1', 'e2'])
@pytest.mark.parametrize('truth_off, guess_err', [
    ((0.0, 0.0, 0.0), (0.0, 0.3, math.radians(10))),          # facing the back wall, AMCL 30 cm off
    ((0.2, -0.45, math.pi), (-0.1, 0.3, math.radians(-12))),  # facing the doors (after turning)
    ((-0.3, 0.3, 2.5), (0.15, -0.2, math.radians(8))),
])
def test_recovers_pose(b, eid, truth_off, guess_err):
    rng = np.random.default_rng(1)
    interior = b.cabin_interior(eid)
    cx, cy = b.elevators[eid]['cabin_center']
    truth = (cx + truth_off[0], cy + truth_off[1], truth_off[2])
    lidar = (-0.032, 0.0, 0.0)
    # scan taken from the LiDAR mount, expressed back in the robot frame
    lp = (truth[0] + lidar[0] * math.cos(truth[2]), truth[1] + lidar[0] * math.sin(truth[2]), truth[2])
    ranges = raycast(lp, interior, rng=rng)
    pts = scan_points(ranges, 0.0, 2 * math.pi / 360, 0.12, 3.5, lidar)
    guess = (truth[0] + guess_err[0], truth[1] + guess_err[1], truth[2] + guess_err[2])
    pose, info = fit_cabin_pose(pts, guess, interior)
    assert info['ok'], info
    assert math.hypot(pose[0] - truth[0], pose[1] - truth[1]) < 0.03
    assert abs(math.remainder(pose[2] - truth[2], 2 * math.pi)) < math.radians(1.5)


def test_rejects_when_outside_cabin(b):
    interior = b.cabin_interior('e1')
    pose, info = fit_cabin_pose(np.zeros((0, 2)), (-4.8, 1.5, 0.0), interior)
    assert not info['ok']
