"""Generate the Gazebo Harmonic world and the Nav2 occupancy map from building.yaml.

    ros2 run elevator_sim generate_building --config config/building.yaml --out <pkg dir>

Writes <out>/worlds/building.sdf, <out>/maps/building.pgm and <out>/maps/building.yaml.
"""
import argparse
import os

from elevator_sim.building import (
    Building, CABIN_HEIGHT, CABIN_WALL, DOOR_HEIGHT, DOOR_THICKNESS, PLATFORM_THICKNESS)

FLOOR_TINTS = {1: '0.85 0.82 0.75', 2: '0.75 0.82 0.88', 3: '0.80 0.88 0.78'}
ELEVATOR_COLORS = {'e1': '0.15 0.35 0.75', 'e2': '0.85 0.45 0.10'}


def _material(rgb, alpha=1.0):
    return (f'<material><ambient>{rgb} {alpha}</ambient><diffuse>{rgb} {alpha}</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


def _box(name, pose, size, rgb, collide=True, alpha=1.0):
    p = ' '.join(f'{v:.4f}' for v in pose)
    s = ' '.join(f'{v:.4f}' for v in size)
    out = (f'<visual name="{name}_v"><pose>{p}</pose><geometry><box><size>{s}</size></box></geometry>'
           f'{_material(rgb, alpha)}</visual>')
    if collide:
        out += (f'<collision name="{name}_c"><pose>{p}</pose><geometry><box><size>{s}</size></box>'
                f'</geometry></collision>')
    return out


def _inertial(mass, sx, sy, sz):
    ixx = mass / 12.0 * (sy * sy + sz * sz)
    iyy = mass / 12.0 * (sx * sx + sz * sz)
    izz = mass / 12.0 * (sx * sx + sy * sy)
    return (f'<inertial><mass>{mass}</mass><inertia><ixx>{ixx:.4f}</ixx><iyy>{iyy:.4f}</iyy>'
            f'<izz>{izz:.4f}</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>')


def building_model(b: Building) -> str:
    parts = ['<model name="building"><static>true</static>']
    for f in b.floor_numbers:
        z = b.floor_z(f)
        parts.append(f'<link name="floor_{f}">')
        for i, (x0, x1, y0, y1) in enumerate(b.slab_rects()):
            parts.append(_box(f'slab_{f}_{i}', ((x0 + x1) / 2, (y0 + y1) / 2, z - b.slab_t / 2, 0, 0, 0),
                              (x1 - x0, y1 - y0, b.slab_t), '0.55 0.55 0.55'))
        for i, w in enumerate(b.walls):
            x0, x1, y0, y1 = b.wall_rect(w)
            parts.append(_box(f'wall_{f}_{i}', ((x0 + x1) / 2, (y0 + y1) / 2, z + b.wall_h / 2, 0, 0, 0),
                              (x1 - x0, y1 - y0, b.wall_h), FLOOR_TINTS.get(f, '0.8 0.8 0.8')))
        parts.append('</link>')
    parts.append('</model>')
    return '\n'.join(parts)


def elevator_model(b: Building, eid: str) -> str:
    e = b.elevators[eid]
    rgb = ELEVATOR_COLORS.get(eid, '0.5 0.5 0.5')
    x0, x1, y0, y1 = b.cabin_box(eid)
    lx, ly = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    z_lo, z_hi = min(b.floors.values()), max(b.floors.values())
    door = e['door']
    travel = b.door_travel(eid)
    dx, dcy = b.door_panel_center(eid)
    lo, hi = (0.0, travel) if door['open_dir'] > 0 else (-travel, 0.0)

    cabin = (
        f'<link name="cabin"><pose>{cx:.4f} {cy:.4f} {z_lo:.4f} 0 0 0</pose><gravity>false</gravity>'
        + _inertial(80.0, lx, ly, 0.3)
        + _box('platform', (0, 0, -PLATFORM_THICKNESS / 2, 0, 0, 0), (lx, ly, PLATFORM_THICKNESS), '0.3 0.3 0.32')
        + _box('back', (-lx / 2 + CABIN_WALL / 2, 0, CABIN_HEIGHT / 2, 0, 0, 0), (CABIN_WALL, ly, CABIN_HEIGHT),
               '0.7 0.72 0.75')
        + _box('left', (0, ly / 2 - CABIN_WALL / 2, CABIN_HEIGHT / 2, 0, 0, 0), (lx, CABIN_WALL, CABIN_HEIGHT),
               '0.7 0.72 0.75')
        + _box('right', (0, -ly / 2 + CABIN_WALL / 2, CABIN_HEIGHT / 2, 0, 0, 0), (lx, CABIN_WALL, CABIN_HEIGHT),
               '0.7 0.72 0.75')
        + _box('stripe', (-lx / 2 + CABIN_WALL + 0.005, 0, 1.0, 0, 0, 0), (0.01, ly * 0.8, 0.15), rgb,
               collide=False)
        + '</link>'
        f'<joint name="lift" type="prismatic"><parent>world</parent><child>cabin</child>'
        f'<axis><xyz>0 0 1</xyz><limit><lower>{z_lo - 0.01:.3f}</lower><upper>{z_hi - z_lo + 0.01:.3f}</upper>'
        f'<velocity>2.0</velocity><effort>100000</effort></limit></axis></joint>'
    )
    doors = []
    plugins = [
        f'<plugin filename="gz-sim-joint-position-controller-system" '
        f'name="gz::sim::systems::JointPositionController"><joint_name>lift</joint_name>'
        f'<topic>/{eid}/lift_cmd</topic><use_velocity_commands>true</use_velocity_commands>'
        f'<p_gain>4.0</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>'
        f'<cmd_max>1.0</cmd_max><cmd_min>-1.0</cmd_min><initial_position>0</initial_position></plugin>'
    ]
    for f in b.floor_numbers:
        fz = b.floor_z(f)
        ln, jn = f'door_f{f}', f'door_f{f}_joint'     # link and joint names must differ in SDF
        doors.append(
            f'<link name="{ln}"><pose>{dx:.4f} {dcy:.4f} {fz + 0.01 + DOOR_HEIGHT / 2:.4f} 0 0 0</pose>'
            f'<gravity>false</gravity>' + _inertial(15.0, DOOR_THICKNESS, travel, DOOR_HEIGHT)
            + _box('panel', (0, 0, 0, 0, 0, 0), (DOOR_THICKNESS, travel, DOOR_HEIGHT), rgb)
            + _box('label', (DOOR_THICKNESS / 2 + 0.003, 0, 0.6, 0, 0, 0), (0.005, 0.5, 0.3), '0.95 0.95 0.95',
                   collide=False)
            + '</link>'
            f'<joint name="{jn}" type="prismatic"><parent>world</parent><child>{ln}</child>'
            f'<axis><xyz>0 1 0</xyz><limit><lower>{lo - 0.01:.3f}</lower><upper>{hi + 0.01:.3f}</upper>'
            f'<velocity>2.0</velocity><effort>10000</effort></limit></axis></joint>')
        plugins.append(
            f'<plugin filename="gz-sim-joint-position-controller-system" '
            f'name="gz::sim::systems::JointPositionController"><joint_name>{jn}</joint_name>'
            f'<topic>/{eid}/{ln}_cmd</topic><use_velocity_commands>true</use_velocity_commands>'
            f'<p_gain>4.0</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>'
            f'<cmd_max>1.0</cmd_max><cmd_min>-1.0</cmd_min><initial_position>0</initial_position></plugin>')
    plugins.append(
        f'<plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">'
        f'<topic>/{eid}/joint_states</topic><update_rate>30</update_rate></plugin>')
    return f'<model name="{eid}"><self_collide>false</self_collide>' + cabin + ''.join(doors) + ''.join(
        plugins) + '</model>'


def box_models(b: Building) -> str:
    out = []
    for i, (f, x, y, sx, sy, h) in enumerate(b.boxes):
        z = b.floor_z(int(f))
        out.append(f'<model name="box_{i}"><static>true</static><pose>{x} {y} {z + h / 2:.3f} 0 0 0</pose>'
                   f'<link name="link">{_box("box", (0, 0, 0, 0, 0, 0), (sx, sy, h), "0.6 0.4 0.2")}</link></model>')
    return '\n'.join(out)


def world_sdf(b: Building) -> str:
    return f'''<?xml version="1.0"?>
<!-- GENERATED by elevator_sim/building_gen.py from config/building.yaml - do not edit by hand -->
<sdf version="1.9">
<world name="{b.world_name}">
  <physics name="default" type="dart">
    <max_step_size>0.003</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
  <scene>
    <ambient>0.6 0.6 0.6 1</ambient>
    <background>0.75 0.8 0.9 1</background>
    <shadows>false</shadows>
    <grid>false</grid>
  </scene>
  <gui fullscreen="0">
    <camera name="user_camera">
      <pose>-2 -16 16 0 0.75 1.57</pose>
    </camera>
  </gui>
  <light type="directional" name="sun">
    <cast_shadows>false</cast_shadows>
    <pose>0 0 30 0 0 0</pose>
    <diffuse>0.9 0.9 0.9 1</diffuse>
    <specular>0.2 0.2 0.2 1</specular>
    <direction>-0.4 0.3 -1</direction>
  </light>
  <model name="pit_floor"><static>true</static><pose>0 0 -0.6 0 0 0</pose>
    <link name="link">{_box("pit", (0, 0, 0, 0, 0, 0), (40, 40, 0.1), "0.35 0.4 0.35")}</link></model>
{building_model(b)}
{"".join(elevator_model(b, e) for e in sorted(b.elevators))}
{box_models(b)}
</world>
</sdf>
'''


def occupancy_grid(b: Building):
    """Returns (rows, width, height, origin). rows[0] is the TOP row (max y), PGM order."""
    res = float(b.map_cfg['resolution'])
    m = float(b.map_cfg['margin'])
    fp = b.footprint
    ox, oy = fp['x_min'] - m, fp['y_min'] - m
    w = int(round((fp['x_max'] - fp['x_min'] + 2 * m) / res))
    h = int(round((fp['y_max'] - fp['y_min'] + 2 * m) / res))
    rects = [b.wall_rect(wl) for wl in b.walls]
    half = res / 2.0
    rows = []
    for r in range(h):
        y = oy + (h - 1 - r + 0.5) * res
        row = bytearray()
        for c in range(w):
            x = ox + (c + 0.5) * res
            if any(x0 - half <= x <= x1 + half and y0 - half <= y <= y1 + half for x0, x1, y0, y1 in rects):
                row.append(0)
            elif fp['x_min'] <= x <= fp['x_max'] and fp['y_min'] <= y <= fp['y_max']:
                row.append(254)
            else:
                row.append(205)
        rows.append(bytes(row))
    return rows, w, h, (ox, oy)


def write_map(b: Building, out_dir: str, name='building'):
    rows, w, h, (ox, oy) = occupancy_grid(b)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, f'{name}.pgm'), 'wb') as f:
        f.write(f'P5\n# generated from building.yaml\n{w} {h}\n255\n'.encode())
        for row in rows:
            f.write(row)
    with open(os.path.join(out_dir, f'{name}.yaml'), 'w') as f:
        f.write(f'image: {name}.pgm\nmode: trinary\nresolution: {b.map_cfg["resolution"]}\n'
                f'origin: [{ox}, {oy}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config', default=None, help='building.yaml (default: installed share copy)')
    ap.add_argument('--out', default='.', help='package directory to write worlds/ and maps/ into')
    a = ap.parse_args(argv)
    b = Building.load(a.config)
    os.makedirs(os.path.join(a.out, 'worlds'), exist_ok=True)
    with open(os.path.join(a.out, 'worlds', 'building.sdf'), 'w') as f:
        f.write(world_sdf(b))
    write_map(b, os.path.join(a.out, 'maps'))
    print(f'wrote {a.out}/worlds/building.sdf and {a.out}/maps/building.(pgm|yaml)')


if __name__ == '__main__':
    main()
