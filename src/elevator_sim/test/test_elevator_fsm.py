"""Unit tests for the elevator state machine using an ideal simulated plant."""
import pytest

from elevator_sim import elevator_fsm as fsm

FLOORS = {1: 0.0, 2: 3.0, 3: 6.0}


class Plant:
    """Joints follow commands perfectly (kinematic carrier)."""

    def __init__(self, e: fsm.ElevatorFSM, blocked=lambda t: False, door_stuck=False):
        self.e, self.t, self.dt = e, 0.0, 0.02
        self.lift, self.doors = e.lift.pos, {f: 0.0 for f in FLOORS}
        self.blocked, self.door_stuck = blocked, door_stuck

    def run(self, seconds, until=None):
        for _ in range(int(seconds / self.dt)):
            self.t += self.dt
            lift_cmd, door_cmd = self.e.step(self.t, self.dt, self.lift, self.doors, self.blocked(self.t))
            self.lift = lift_cmd
            if not self.door_stuck:
                self.doors = dict(door_cmd)
            if until and until():
                return True
        return False


def make(**kw):
    return fsm.ElevatorFSM(FLOORS, door_travel=1.2, door_dir=1, speed=0.4, door_speed=0.4, **kw)


def test_call_to_current_floor_opens_doors():
    e = make()
    p = Plant(e)
    assert e.request_floor(1, 'tb3_1')[0]
    assert p.run(20, until=lambda: e.goal_result is not None)
    assert e.goal_result[0] is True
    assert e.state == fsm.DOORS_OPEN and e.current_floor == 1
    assert p.doors[1] == pytest.approx(1.2, abs=0.03)


def test_ride_closes_moves_opens():
    e = make()
    p = Plant(e)
    e.request_floor(1, 'r')
    p.run(20, until=lambda: e.goal_result is not None)
    seen = set()
    assert e.request_floor(3, 'r')[0]
    assert p.run(60, until=lambda: (seen.add(e.state), e.goal_result is not None)[1])
    assert e.goal_result == (True, 'arrived at floor 3, doors open')
    assert {fsm.DOORS_CLOSING, fsm.MOVING, fsm.DOORS_OPENING} <= seen
    assert p.lift == pytest.approx(6.0, abs=0.02)
    assert p.doors[1] == pytest.approx(0.0, abs=0.02)      # floor-1 doors closed behind us
    assert p.doors[3] == pytest.approx(1.2, abs=0.03)


def test_never_moves_with_doors_open():
    e = make()
    p = Plant(e)
    e.request_floor(1, 'r')
    p.run(20, until=lambda: e.goal_result is not None)
    e.request_floor(2, 'r')
    lifts_while_open = []
    for _ in range(3000):
        p.run(0.02)
        if abs(p.doors[1]) > 0.05 or abs(p.doors[2]) > 0.05:
            lifts_while_open.append(p.lift)
        if e.goal_result is not None:
            break
    # cabin only leaves floor 1 after its doors are shut, and only opens floor-2 doors after arriving
    assert all(abs(z - 0.0) < 0.02 or abs(z - 3.0) < 0.02 for z in lifts_while_open)


def test_busy_rejects_second_requester():
    e = make()
    assert e.request_floor(2, 'tb3_1')[0]
    ok, msg = e.request_floor(3, 'intruder')
    assert not ok and 'busy' in msg


def test_unknown_floor_rejected():
    assert not make().request_floor(7, 'r')[0]


def test_obstruction_reopens_then_faults():
    e = make(max_reopen=2)
    p = Plant(e, blocked=lambda t: t > 5.0)   # something stands in the doorway after doors opened
    e.request_floor(1, 'r')
    p.run(20, until=lambda: e.goal_result is not None)
    e.goal_result = None
    e.request_floor(2, 'r')
    p.run(120, until=lambda: e.state == fsm.FAULT)
    assert e.state == fsm.FAULT and 'blocked' in e.fault
    assert e.goal_result[0] is False
    assert p.lift == pytest.approx(0.0, abs=0.01)           # never moved


def test_transient_obstruction_recovers():
    e = make()
    p = Plant(e, blocked=lambda t: 8.0 < t < 9.0)
    e.request_floor(1, 'r')
    p.run(20, until=lambda: e.goal_result is not None)
    e.request_floor(2, 'r')
    assert p.run(120, until=lambda: e.goal_result is not None)
    assert e.goal_result[0] is True and e.current_floor == 2


def test_door_timeout_faults():
    e = make()
    p = Plant(e, door_stuck=True)
    e.request_floor(1, 'r')
    p.run(60, until=lambda: e.state == fsm.FAULT)
    assert e.state == fsm.FAULT and 'timeout' in e.fault


def test_emergency_and_reset():
    e = make()
    p = Plant(e)
    e.request_floor(3, 'r')
    p.run(5)                                  # somewhere between floors
    assert e.state == fsm.MOVING
    e.emergency(p.t)
    z = p.lift
    p.run(3)
    assert e.state == fsm.EMERGENCY and p.lift == pytest.approx(z)
    assert e.goal_result[0] is False
    assert not e.request_floor(1, 'r')[0]      # refused while in EMERGENCY
    ok, msg = e.reset(p.t, p.lift, p.doors)
    assert ok and 'recovering' in msg
    p.run(60, until=lambda: e.state == fsm.IDLE)
    assert e.state == fsm.IDLE and e.current_floor in FLOORS


def test_close_request_and_hold_timeout():
    e = make(door_hold_timeout=10.0)
    p = Plant(e)
    e.request_floor(1, 'r')
    p.run(20, until=lambda: e.state == fsm.DOORS_OPEN)
    assert e.request_close()[0]
    assert p.run(20, until=lambda: e.state == fsm.IDLE)
    e.request_open(p.t)
    p.run(20, until=lambda: e.state == fsm.DOORS_OPEN)
    assert p.run(30, until=lambda: e.state == fsm.IDLE)     # auto-close after hold timeout


def test_profile_respects_limits():
    pr = fsm.Profile(0.0, vmax=0.4, amax=0.3)
    prev, v_max = 0.0, 0.0
    for _ in range(2000):
        pos = pr.step(6.0, 0.02)
        v_max = max(v_max, (pos - prev) / 0.02)
        prev = pos
    assert pos == pytest.approx(6.0) and v_max <= 0.4 + 1e-6
