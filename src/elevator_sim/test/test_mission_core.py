"""Mission executor tests against a fake robot + elevator (no ROS)."""
import os
from types import SimpleNamespace

import pytest

from elevator_sim.building import Building
from elevator_sim.mission_core import MissionExecutor
from elevator_sim.metrics import append_result, FIELDS

CFG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config', 'building.yaml')


class FakeWorld:
    """Teleporting robot + instant elevator, with fault injection."""

    def __init__(self, b, robot, nav_failures=0, elevator_fault=False, stuck_in_doorway=False):
        self.b, self.robot = b, robot
        self.eid = b.robots[robot]['elevator']
        self.x, self.y, self.yaw = b.robots[robot]['home']
        self.t = 0.0
        self.cabin_floor, self.doors_open = 1, False
        self.nav_failures, self.elevator_fault, self.stuck = nav_failures, elevator_fault, stuck_in_doorway
        self.calls, self.stopped, self.cleared = [], False, 0

    def now(self):
        return self.t

    def sleep(self, s):
        self.t += s

    def navigate(self, x, y, yaw, timeout):
        self.t += 10
        if self.nav_failures > 0:
            self.nav_failures -= 1
            return False, 'aborted'
        self.x, self.y, self.yaw = x, y, yaw
        return True, 'ok'

    def drive_to(self, x, y, speed):
        self.t += 5
        if self.stuck:
            self.x = self.b.elevators[self.eid]['door']['x']   # stops in the doorway
            return False, 'collision ahead'
        self.x, self.y = x, y
        return True, 'ok'

    def face(self, x, y):
        return True, 'ok'

    def relocalize_in_cabin(self, interior):
        self.relocalized = getattr(self, 'relocalized', 0) + 1
        return True, 'cabin fit ok'

    def pose(self):
        return (self.x, self.y, self.yaw)

    def elevator_goto(self, floor, timeout):
        self.calls.append(floor)
        self.t += 8
        if self.elevator_fault:
            return False, 'doorway blocked'
        self.cabin_floor, self.doors_open = floor, True
        return True, 'arrived'

    def elevator_close(self):
        self.doors_open = False
        return True, 'closing'

    def elevator_state(self):
        return SimpleNamespace(state=('FAULT' if self.elevator_fault else
                                      'DOORS_OPEN' if self.doors_open else 'IDLE'),
                               current_floor=self.cabin_floor, door_opening=1.0 if self.doors_open else 0.0,
                               fault='doorway blocked' if self.elevator_fault else '')

    def clear_costmaps(self):
        self.cleared += 1

    def safe_stop(self):
        self.stopped = True


@pytest.fixture(scope='module')
def b():
    return Building.load(CFG)


def run(b, robot='tb3_1', floor=3, goal='room_b', **kw):
    w = FakeWorld(b, robot, **kw)
    res = MissionExecutor(b, robot, floor, goal, w, log=lambda *_: None).run()
    return res, w


def test_full_round_trip(b):
    res, w = run(b)
    assert res.success, res.message
    assert w.calls == [1, 3, 3, 1]                        # call@1, ride->3, call@3, ride->1
    assert (w.x, w.y) == tuple(b.robots['tb3_1']['home'][:2])
    assert res.steps_done == res.steps_total == 14
    assert all(res.sim_durations[c] > 0 for c in res.sim_durations)
    assert w.relocalized == 6          # after boarding, after turning, before exiting - per trip


def test_robot2_uses_elevator2(b):
    res, _ = run(b, robot='tb3_2', floor=2, goal='room_c')
    assert res.success and res.elevator == 'e2'


def test_nav_retry_recovers(b):
    res, w = run(b, nav_failures=2)
    assert res.success and res.nav_retries == 2 and w.cleared >= 2


def test_nav_gives_up_and_stops(b):
    res, w = run(b, nav_failures=10)
    assert not res.success and w.stopped
    assert res.failed_step.startswith('nav landing')


def test_elevator_fault_aborts_safely(b):
    res, w = run(b, elevator_fault=True)
    assert not res.success and w.stopped
    assert 'FAULT' in res.message and res.failed_step.startswith('call')


def test_stuck_in_doorway_fails_boarding(b):
    res, w = run(b, stuck_in_doorway=True)
    assert not res.success and res.failed_step.startswith('board') and w.stopped


def test_metrics_csv(b, tmp_path):
    res, _ = run(b)
    path = str(tmp_path / 'm' / 'missions.csv')
    append_result(path, res, 'parallel')
    append_result(path, res, 'parallel')
    lines = open(path).read().strip().splitlines()
    assert lines[0].split(',') == FIELDS and len(lines) == 3


def test_stale_elevator_state_is_waited_for(b):
    """Right after the action result the 10 Hz state topic can still show DOORS_OPENING."""
    class Stale(FakeWorld):
        def __init__(self, *a, **k):
            super().__init__(*a, **k)
            self.stale_reads = 0

        def elevator_goto(self, floor, timeout):
            self.stale_reads = 3
            return super().elevator_goto(floor, timeout)

        def elevator_state(self):
            st = super().elevator_state()
            if self.stale_reads > 0:
                self.stale_reads -= 1
                st.state, st.door_opening = 'DOORS_OPENING', 0.9
            return st

    w = Stale(b, 'tb3_1')
    res = MissionExecutor(b, 'tb3_1', 3, 'room_b', w, log=lambda *_: None).run()
    assert res.success, res.message


def test_elevator_never_ready_fails_after_wait(b):
    class NeverOpen(FakeWorld):
        def elevator_state(self):
            st = super().elevator_state()
            st.state = 'DOORS_OPENING'
            return st

    w = NeverOpen(b, 'tb3_1')
    res = MissionExecutor(b, 'tb3_1', 3, 'room_b', w, log=lambda *_: None).run()
    assert not res.success and res.failed_step.startswith('board') and 'not ready' in res.message
