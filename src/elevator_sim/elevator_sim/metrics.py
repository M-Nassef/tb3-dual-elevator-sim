"""CSV metrics for missions (one row per mission)."""
import csv
import datetime
import os

from elevator_sim.mission_core import CATEGORIES

FIELDS = (['timestamp', 'mode', 'robot', 'elevator', 'target_floor', 'goal', 'success', 'failed_step', 'message',
           'sim_total_s', 'wall_total_s']
          + [f'sim_{c}_s' for c in CATEGORIES]
          + ['nav_retries', 'elevator_retries', 'steps_done', 'steps_total'])


def default_metrics_path():
    base = os.environ.get('ELEVATOR_METRICS_DIR') or os.path.join(os.path.expanduser('~'), '.ros', 'elevator_metrics')
    return os.path.join(base, 'missions.csv')


def result_row(res, mode='single'):
    row = {
        'timestamp': datetime.datetime.now().isoformat(timespec='seconds'),
        'mode': mode, 'robot': res.robot, 'elevator': res.elevator, 'target_floor': res.target_floor,
        'goal': res.goal, 'success': int(res.success), 'failed_step': res.failed_step, 'message': res.message,
        'sim_total_s': f'{res.sim_total:.2f}', 'wall_total_s': f'{res.wall_total:.2f}',
        'nav_retries': res.nav_retries, 'elevator_retries': res.elevator_retries,
        'steps_done': res.steps_done, 'steps_total': res.steps_total,
    }
    for c in CATEGORIES:
        row[f'sim_{c}_s'] = f'{res.sim_durations.get(c, 0.0):.2f}'
    return row


def append_result(path, res, mode='single'):
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    new = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, 'a', newline='') as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if new:
            w.writeheader()
        w.writerow(result_row(res, mode))
    return path
