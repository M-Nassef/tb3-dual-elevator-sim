"""Consistency tests between building.yaml, the generated map and the world."""
import os
import xml.etree.ElementTree as ET

import pytest

from elevator_sim.building import Building
from elevator_sim.building_gen import occupancy_grid, world_sdf
from elevator_sim.mission_core import build_plan

HERE = os.path.dirname(os.path.abspath(__file__))
CFG = os.path.join(HERE, '..', 'config', 'building.yaml')


@pytest.fixture(scope='module')
def b():
    return Building.load(CFG)


@pytest.fixture(scope='module')
def grid(b):
    rows, w, h, origin = occupancy_grid(b)
    res = b.map_cfg['resolution']

    def at(x, y):
        c = int((x - origin[0]) / res)
        r = h - 1 - int((y - origin[1]) / res)
        return rows[r][c]
    return at


def test_three_floors(b):
    assert b.floor_numbers == [1, 2, 3]
    assert b.floor_z(3) == 6.0


def test_each_robot_has_own_elevator(b):
    used = [r['elevator'] for r in b.robots.values()]
    assert len(set(used)) == len(used) == 2


@pytest.mark.parametrize('point', ['homes', 'landings', 'rooms', 'cabins'])
def test_key_points_are_free_in_map(b, grid, point):
    pts = {'homes': [r['home'][:2] for r in b.robots.values()],
           'landings': [e['landing'][:2] for e in b.elevators.values()],
           'rooms': [v[:2] for v in b.rooms.values()],
           'cabins': [e['cabin_center'] for e in b.elevators.values()]}[point]
    for x, y in pts:
        assert grid(x, y) == 254, f'{point} {x},{y} not free'


def test_door_openings_free_and_walls_occupied(b, grid):
    for e in b.elevators.values():
        d = e['door']
        assert grid(d['x'], (d['y_min'] + d['y_max']) / 2) == 254
    assert grid(-6.0, 0.0) == 0         # wall between the two shafts
    assert grid(8.0, 0.0) == 0          # outer wall


def test_cabin_fits_in_shaft_and_robots_fit_in_cabin(b):
    for eid, e in b.elevators.items():
        s = e['shaft']
        x0, x1, y0, y1 = b.cabin_box(eid)
        assert s['x_min'] < x0 < x1 < s['x_max'] and s['y_min'] < y0 < y1 < s['y_max']
        cx, cy = e['cabin_center']
        for r in b.robots.values():
            assert b.in_cabin(eid, cx, cy, r['radius'] + 0.3)


def test_slab_has_shaft_holes(b):
    for eid, e in b.elevators.items():
        cx, cy = e['cabin_center']
        for x0, x1, y0, y1 in b.slab_rects():
            assert not (x0 < cx < x1 and y0 < cy < y1), f'slab covers shaft of {eid}'


def test_world_sdf_is_valid_xml_with_elevators(b):
    root = ET.fromstring(world_sdf(b).split('\n', 1)[1])
    models = {m.get('name') for m in root.iter('model')}
    assert {'building', 'e1', 'e2'} <= models
    joints = {j.get('name') for m in root.iter('model') if m.get('name') == 'e1' for j in m.iter('joint')}
    assert joints == {'lift', 'door_f1_joint', 'door_f2_joint', 'door_f3_joint'}


def test_plan_structure(b):
    steps = build_plan(b, 'tb3_1', 3, 'room_b')
    kinds = [s.kind for s in steps]
    assert kinds == ['nav', 'call', 'board', 'ride', 'exit', 'close', 'nav',
                     'nav', 'call', 'board', 'ride', 'exit', 'close', 'nav']
    assert steps[3].floor == 3 and steps[10].floor == 1
    assert steps[6].target == tuple(b.rooms['room_b'])


def test_plan_same_floor_skips_elevator(b):
    assert [s.kind for s in build_plan(b, 'tb3_2', 1, 'room_a')] == ['nav', 'nav']


def test_plan_rejects_bad_input(b):
    with pytest.raises(ValueError):
        build_plan(b, 'tb3_9', 2, 'room_a')
    with pytest.raises(ValueError):
        build_plan(b, 'tb3_1', 2, 'kitchen')
    with pytest.raises(ValueError):
        build_plan(b, 'tb3_1', 5, 'room_a')


def test_open_doors_do_not_hit_walls(b):
    """A fully open door panel must stay clear of every wall (it slides sideways)."""
    from elevator_sim.building import DOOR_THICKNESS
    for eid, e in b.elevators.items():
        dx, dcy = b.door_panel_center(eid)
        travel = b.door_travel(eid)
        cy = dcy + e['door']['open_dir'] * travel
        px0, px1 = dx - DOOR_THICKNESS / 2, dx + DOOR_THICKNESS / 2
        py0, py1 = cy - travel / 2, cy + travel / 2
        for w in b.walls:
            x0, x1, y0, y1 = b.wall_rect(w)
            overlap = px0 < x1 and x0 < px1 and py0 < y1 and y0 < py1
            assert not overlap, f'{eid} open door hits wall {w}'
