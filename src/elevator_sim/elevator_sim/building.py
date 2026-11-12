"""Building model: loads config/building.yaml and provides geometry helpers.

Pure Python (no ROS imports) so it can be unit-tested and used by the generator,
the elevator controller and the mission node alike.
"""
import math
import os

import yaml

# Mechanical constants of the generated elevator model (metres).
CABIN_WALL = 0.04          # cabin wall thickness
CABIN_SIDE_GAP = 0.08      # gap between cabin and shaft walls (sides/back)
CABIN_FRONT_GAP = 0.07     # gap between cabin floor and the shaft front line
PLATFORM_THICKNESS = 0.05
CABIN_HEIGHT = 2.2
THRESHOLD_DEPTH = 0.05     # landing sill that bridges the gap in front of the cabin
DOOR_THICKNESS = 0.04
DOOR_OFFSET = 0.08         # door panel centre is this far in front of the shaft line
DOOR_HEIGHT = 2.2


def default_config_path():
    """Path of building.yaml in the installed share dir (falls back to source tree)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('elevator_sim'), 'config', 'building.yaml')
    except Exception:  # pragma: no cover - only outside a ROS environment
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(here, '..', 'config', 'building.yaml')


class Building:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.world_name = cfg['world_name']
        self.floors = {int(k): float(v['z']) for k, v in cfg['floors'].items()}
        self.footprint = cfg['footprint']
        self.wall_t = float(cfg['wall']['thickness'])
        self.wall_h = float(cfg['wall']['height'])
        self.slab_t = float(cfg['slab_thickness'])
        self.walls = [tuple(map(float, w)) for w in cfg['walls']]
        self.rooms = {k: tuple(map(float, v)) for k, v in cfg['rooms'].items()}
        self.boxes = [tuple(map(float, b)) for b in cfg.get('boxes', [])]
        self.elevators = cfg['elevators']
        self.robots = cfg['robots']
        self.map_cfg = cfg['map']

    @classmethod
    def load(cls, path=None):
        with open(path or default_config_path()) as f:
            return cls(yaml.safe_load(f))

    # ------------------------------------------------------------------ floors
    @property
    def floor_numbers(self):
        return sorted(self.floors)

    def floor_z(self, floor: int) -> float:
        if floor not in self.floors:
            raise ValueError(f'unknown floor {floor}; valid floors: {self.floor_numbers}')
        return self.floors[floor]

    def floor_from_height(self, z: float, tol: float = 0.02) -> int:
        """Floor whose level is within tol of z, else -1."""
        for f, fz in self.floors.items():
            if abs(fz - z) <= tol:
                return f
        return -1

    # --------------------------------------------------------------- elevators
    def cabin_box(self, eid):
        """(x_min, x_max, y_min, y_max) of the cabin platform."""
        s = self.elevators[eid]['shaft']
        return (s['x_min'] + CABIN_SIDE_GAP, s['x_max'] - CABIN_FRONT_GAP,
                s['y_min'] + CABIN_SIDE_GAP, s['y_max'] - CABIN_SIDE_GAP)

    def cabin_interior(self, eid):
        """Free area inside the cabin walls."""
        x0, x1, y0, y1 = self.cabin_box(eid)
        return (x0 + CABIN_WALL, x1, y0 + CABIN_WALL, y1 - CABIN_WALL)

    def door_travel(self, eid) -> float:
        d = self.elevators[eid]['door']
        return d['y_max'] - d['y_min']

    def door_panel_center(self, eid):
        d = self.elevators[eid]['door']
        return (d['x'] + DOOR_OFFSET, 0.5 * (d['y_min'] + d['y_max']))

    def doorway_zone(self, eid, margin=0.35):
        """Area in which a robot centre counts as 'standing in the doorway'."""
        d = self.elevators[eid]['door']
        _, x1, _, _ = self.cabin_interior(eid)
        return (x1 - margin, d['x'] + margin, d['y_min'] - 0.05, d['y_max'] + 0.05)

    def in_cabin(self, eid, x, y, robot_radius=0.0) -> bool:
        x0, x1, y0, y1 = self.cabin_interior(eid)
        return (x0 + robot_radius <= x <= x1 - robot_radius) and (y0 + robot_radius <= y <= y1 - robot_radius)

    def board_distance(self, eid) -> float:
        lx, ly, _ = self.elevators[eid]['landing']
        cx, cy = self.elevators[eid]['cabin_center']
        return math.hypot(lx - cx, ly - cy)

    # -------------------------------------------------------------- geometry
    def wall_rect(self, w):
        """Wall centre-line -> axis-aligned rectangle (x0, x1, y0, y1)."""
        x1, y1, x2, y2 = w
        h = self.wall_t / 2.0
        if abs(y1 - y2) < 1e-9:          # horizontal
            return (min(x1, x2) - h, max(x1, x2) + h, y1 - h, y1 + h)
        if abs(x1 - x2) < 1e-9:          # vertical
            return (x1 - h, x1 + h, min(y1, y2) - h, max(y1, y2) + h)
        raise ValueError(f'walls must be axis-aligned: {w}')

    def slab_rects(self):
        """Floor slab as rectangles: footprint minus elevator shafts, plus door sills."""
        fp = self.footprint
        holes = [self.elevators[e]['shaft'] for e in sorted(self.elevators)]
        xs = sorted({fp['x_min'], fp['x_max']} | {h[k] for h in holes for k in ('x_min', 'x_max')})
        ys = sorted({fp['y_min'], fp['y_max']} | {h[k] for h in holes for k in ('y_min', 'y_max')})
        out = []
        for i in range(len(xs) - 1):
            cur = None                       # rectangle being grown along +y
            for j in range(len(ys) - 1):
                cx, cy = 0.5 * (xs[i] + xs[i + 1]), 0.5 * (ys[j] + ys[j + 1])
                if any(h['x_min'] < cx < h['x_max'] and h['y_min'] < cy < h['y_max'] for h in holes):
                    if cur:
                        out.append(cur)
                    cur = None
                elif cur is None:
                    cur = (xs[i], xs[i + 1], ys[j], ys[j + 1])
                else:
                    cur = (cur[0], cur[1], cur[2], ys[j + 1])
            if cur:
                out.append(cur)
        for e in sorted(self.elevators):
            d = self.elevators[e]['door']
            out.append((d['x'] - THRESHOLD_DEPTH, d['x'], d['y_min'], d['y_max']))
        return out


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
