"""Draw docs/layout.png (top view of one floor) from building.yaml. Usage: python3 draw_layout.py <building.yaml> <out.png>"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.patches as mp  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
import yaml  # noqa: E402

cfg = yaml.safe_load(open(sys.argv[1]))
fig, ax = plt.subplots(figsize=(11, 7.2))
t = cfg['wall']['thickness']
for x1, y1, x2, y2 in cfg['walls']:
    ax.add_patch(mp.Rectangle((min(x1, x2) - t / 2, min(y1, y2) - t / 2), abs(x2 - x1) + t, abs(y2 - y1) + t,
                              color='#333'))
col = {'e1': '#2a5bd7', 'e2': '#e07a10'}
for eid, e in cfg['elevators'].items():
    s, d = e['shaft'], e['door']
    ax.add_patch(mp.Rectangle((s['x_min'], s['y_min']), s['x_max'] - s['x_min'], s['y_max'] - s['y_min'],
                              color=col[eid], alpha=0.18))
    ax.plot([d['x'] + 0.08] * 2, [d['y_min'], d['y_max']], color=col[eid], lw=5)
    ax.text(*e['cabin_center'], eid.upper(), ha='center', va='center', fontsize=13, weight='bold', color=col[eid])
    lx, ly, _ = e['landing']
    ax.plot(lx, ly, 'o', color=col[eid], ms=9)
    ax.text(lx + 0.25, ly, f'landing {eid}', va='center', fontsize=9, color=col[eid])
for name, (x, y, _) in cfg['rooms'].items():
    ax.plot(x, y, '*', color='#2b9348', ms=15)
    ax.text(x, y - 0.45, name, ha='center', fontsize=10, color='#2b9348')
for name, r in cfg['robots'].items():
    x, y, _ = r['home']
    c = col[r['elevator']]
    ax.add_patch(mp.Circle((x, y), r['radius'], color=c))
    ax.text(x + 0.35, y, f'{name} ({r["model"]})', va='center', fontsize=9, color=c)
for f, x, y, sx, sy, _ in cfg['boxes']:
    ax.add_patch(mp.Rectangle((x - sx / 2, y - sy / 2), sx, sy, color='#8b5a2b', alpha=0.7))
    ax.text(x, y + sy / 2 + 0.1, f'box F{f}', ha='center', fontsize=8, color='#8b5a2b')
ax.text(-7.0, 4.2, 'storage', ha='center', color='#888')
ax.text(-7.0, -4.2, 'storage', ha='center', color='#888')
ax.text(-4.5, -2.9, 'lobby', ha='center', color='#888')
ax.text(2.5, 0.0, 'corridor', ha='center', color='#888')
ax.set_aspect('equal')
ax.set_xlim(-8.6, 8.6)
ax.set_ylim(-5.6, 5.6)
ax.set_xlabel('x [m]')
ax.set_ylabel('y [m]')
ax.set_title('Floor plan (identical on floors 1, 2, 3 - boxes differ per floor)')
ax.grid(alpha=0.2)
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=110)
