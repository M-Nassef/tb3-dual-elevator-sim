"""Elevator controller state machine (pure Python, no ROS) - one instance per elevator.

States
  IDLE           doors closed, cabin parked at current_floor
  DOORS_OPENING  doors of current_floor opening
  DOORS_OPEN     doors held open (until a goal or CLOSE_DOORS, or hold timeout)
  DOORS_CLOSING  doors closing; doorway obstruction -> re-open (bounded retries)
  MOVING         cabin travelling to target floor (doors closed, interlocked)
  FAULT          timeout / repeated obstruction: motion frozen until RESET
  EMERGENCY      operator stop: motion frozen until RESET

The FSM is advanced by step(); it consumes *measured* joint positions and a
doorway-obstruction flag, and produces *commanded* joint positions following a
velocity/acceleration-limited profile.
"""
import math

IDLE = 'IDLE'
DOORS_OPENING = 'DOORS_OPENING'
DOORS_OPEN = 'DOORS_OPEN'
DOORS_CLOSING = 'DOORS_CLOSING'
MOVING = 'MOVING'
FAULT = 'FAULT'
EMERGENCY = 'EMERGENCY'


class Profile:
    """1-D velocity/acceleration-limited set-point generator."""

    def __init__(self, pos, vmax, amax):
        self.pos, self.vel, self.vmax, self.amax = pos, 0.0, vmax, amax

    def step(self, target, dt):
        err = target - self.pos
        if abs(err) < 1e-6:
            self.pos, self.vel = target, 0.0
            return self.pos
        direction = math.copysign(1.0, err)
        v_stop = math.sqrt(2.0 * self.amax * abs(err))          # max speed that can still stop
        v_des = direction * min(self.vmax, v_stop)
        dv = max(-self.amax * dt, min(self.amax * dt, v_des - self.vel))
        self.vel += dv
        new = self.pos + self.vel * dt
        if (target - new) * direction <= 0.0:                    # overshoot -> snap
            new, self.vel = target, 0.0
        self.pos = new
        return self.pos

    def hold(self, pos):
        self.pos, self.vel = pos, 0.0

    def at(self, target, tol=1e-4):
        return abs(self.pos - target) < tol and abs(self.vel) < 1e-3


class ElevatorFSM:
    POS_TOL = 0.015        # m, measured cabin level tolerance
    DOOR_TOL = 0.02        # m

    def __init__(self, floors: dict, door_travel: float, door_dir: int = 1, speed=0.4, door_speed=0.4,
                 accel=0.3, max_reopen=3, door_hold_timeout=120.0, start_floor=None):
        self.floors = dict(floors)                       # floor -> z
        self.z0 = min(self.floors.values())
        self.door_travel = door_travel
        self.door_dir = 1 if door_dir >= 0 else -1
        self.max_reopen = max_reopen
        self.door_hold_timeout = door_hold_timeout
        self.current_floor = start_floor if start_floor is not None else min(self.floors)
        self.state = IDLE
        self.lift = Profile(self.floors[self.current_floor] - self.z0, speed, accel)
        self.doors = {f: Profile(0.0, door_speed, 1.0) for f in self.floors}
        self.speed, self.door_speed = speed, door_speed
        self.target_floor = None
        self.owner = ''
        self.fault = ''
        self.reopen_count = 0
        self.blocked_override = False
        self.phase_deadline = None
        self.door_open_since = None
        self.goal_result = None        # (success, message) once the active goal finishes
        self.close_requested = False
        self.last_blocked = False
        self.events = []               # human-readable log lines, drained by the node

    # ----------------------------------------------------------------- helpers
    def _log(self, msg):
        self.events.append(msg)

    def _set(self, state, now, timeout=None):
        if state != self.state:
            self._log(f'{self.state} -> {state}')
        self.state = state
        self.phase_deadline = (now + timeout) if timeout is not None else None

    def joint_target(self, floor):
        return self.floors[floor] - self.z0

    def door_open_pos(self):
        return self.door_dir * self.door_travel

    def door_fraction(self, door_meas):
        return max(0.0, min(1.0, abs(door_meas) / self.door_travel)) if self.door_travel else 0.0

    def _door_time(self):
        return 3.0 * self.door_travel / self.door_speed + 5.0

    def _move_time(self, floor):
        return 2.0 * abs(self.joint_target(floor) - self.lift.pos) / self.speed + 10.0

    def _finish_goal(self, success, msg):
        if self.target_floor is not None:
            self.goal_result = (success, msg)
            self._log(f'goal floor {self.target_floor} for "{self.owner}" -> '
                      f'{"SUCCEEDED" if success else "ABORTED"}: {msg}')
        self.target_floor = None
        self.owner = ''

    def _enter_fault(self, now, why, emergency=False):
        self.fault = why
        self._set(EMERGENCY if emergency else FAULT, now)
        self._finish_goal(False, why)

    # -------------------------------------------------------------- requests
    def busy(self):
        return self.target_floor is not None

    def request_floor(self, floor, requester=''):
        if floor not in self.floors:
            return False, f'unknown floor {floor}'
        if self.state in (FAULT, EMERGENCY):
            return False, f'elevator in {self.state}: {self.fault}'
        if self.busy():
            return False, f'busy serving "{self.owner}" (floor {self.target_floor})'
        self.target_floor, self.owner, self.goal_result = floor, requester, None
        self.close_requested = False
        self.reopen_count = 0
        self._log(f'goal: floor {floor} requested by "{requester}"')
        return True, 'accepted'

    def cancel_goal(self, now):
        if not self.busy():
            return
        if self.state == MOVING:
            # never stop between floors: finish the trip, open doors, report nobody
            self._log('goal canceled while moving - completing trip to floor %d' % self.target_floor)
            self.goal_result = (False, 'canceled')
            self.owner = '(canceled)'
        else:
            self._finish_goal(False, 'canceled')

    def request_close(self):
        if self.state in (DOORS_OPEN, DOORS_OPENING):
            self.close_requested = True
            return True, 'closing'
        if self.state in (IDLE, DOORS_CLOSING):
            return True, 'already closed/closing'
        return False, f'cannot close in {self.state}'

    def request_open(self, now):
        if self.state in (IDLE, DOORS_CLOSING):
            self.close_requested = False
            self._set(DOORS_OPENING, now, self._door_time())
            return True, 'opening'
        if self.state in (DOORS_OPEN, DOORS_OPENING):
            return True, 'already open/opening'
        return False, f'cannot open in {self.state}'

    def emergency(self, now):
        self._enter_fault(now, 'emergency stop', emergency=True)

    def reset(self, now, lift_meas, door_meas):
        if self.state not in (FAULT, EMERGENCY):
            return False, f'nothing to reset ({self.state})'
        self.fault = ''
        self.reopen_count = 0
        self.lift.hold(lift_meas)
        for f, p in self.doors.items():
            p.hold(door_meas.get(f, 0.0))
        f = None
        for fl in self.floors:
            if abs(self.joint_target(fl) - lift_meas) < self.POS_TOL:
                f = fl
        if f is None:
            # between floors: go to the nearest floor with doors closed (no owner)
            f = min(self.floors, key=lambda fl: abs(self.joint_target(fl) - lift_meas))
            self.current_floor = f
            self.target_floor, self.owner = f, 'reset'
            self._set(MOVING, now, self._move_time(f))
            return True, f'recovering to floor {f}'
        self.current_floor = f
        open_now = abs(door_meas.get(f, 0.0)) > self.DOOR_TOL
        self._set(DOORS_OPEN if open_now else IDLE, now)
        self.door_open_since = now if open_now else None
        return True, 'reset'

    # ------------------------------------------------------------------ step
    def step(self, now, dt, lift_meas, door_meas, doorway_blocked=False):
        """Advance the FSM. Returns (lift_cmd, {floor: door_cmd})."""
        blocked = doorway_blocked or self.blocked_override
        if blocked and not self.last_blocked:
            self._log('doorway obstruction detected')
        self.last_blocked = blocked
        f = self.current_floor
        dmeas = door_meas.get(f, 0.0)

        if self.state in (FAULT, EMERGENCY):
            pass  # freeze: profiles keep their last set-points

        elif self.phase_deadline is not None and now > self.phase_deadline:
            self._enter_fault(now, f'timeout in {self.state}')

        elif self.state == IDLE:
            if self.busy():
                if self.target_floor == f:
                    self._set(DOORS_OPENING, now, self._door_time())
                else:
                    self._set(MOVING, now, self._move_time(self.target_floor))

        elif self.state == DOORS_OPENING:
            self.doors[f].step(self.door_open_pos(), dt)
            if abs(dmeas - self.door_open_pos()) < self.DOOR_TOL:
                self._set(DOORS_OPEN, now)
                self.door_open_since = now
                if self.busy() and self.target_floor == f and not self.close_requested:
                    self._finish_goal(True, f'arrived at floor {f}, doors open')

        elif self.state == DOORS_OPEN:
            want_close = self.close_requested or (self.busy() and self.target_floor != f)
            held_too_long = (not self.busy() and self.door_open_since is not None
                             and now - self.door_open_since > self.door_hold_timeout)
            if self.busy() and self.target_floor == f:
                self._finish_goal(True, f'already at floor {f}, doors open')
            elif want_close or held_too_long:
                if held_too_long:
                    self._log('door hold timeout - closing')
                self._set(DOORS_CLOSING, now, self._door_time())

        elif self.state == DOORS_CLOSING:
            if blocked:
                self.reopen_count += 1
                if self.reopen_count > self.max_reopen:
                    self._enter_fault(now, f'doorway blocked: gave up after {self.max_reopen} re-openings')
                else:
                    self._log(f'doorway blocked - re-opening ({self.reopen_count}/{self.max_reopen})')
                    self.close_requested = False
                    self.doors[f].hold(dmeas)
                    self._set(DOORS_OPENING, now, self._door_time())
            else:
                self.doors[f].step(0.0, dt)
                if abs(dmeas) < self.DOOR_TOL and self.doors[f].at(0.0):
                    self.close_requested = False
                    if self.busy() and self.target_floor != f:
                        self._set(MOVING, now, self._move_time(self.target_floor))
                    else:
                        self._set(IDLE, now)

        elif self.state == MOVING:
            # interlock: never move with any door open
            if any(abs(door_meas.get(fl, 0.0)) > 3 * self.DOOR_TOL for fl in self.floors):
                self._enter_fault(now, 'interlock: door open while moving')
            else:
                tgt = self.joint_target(self.target_floor)
                self.lift.step(tgt, dt)
                if self.lift.at(tgt) and abs(lift_meas - tgt) < self.POS_TOL:
                    self.current_floor = self.target_floor
                    if self.owner == 'reset':
                        self._finish_goal(True, 'recovered')
                        self._set(IDLE, now)
                    else:
                        self._set(DOORS_OPENING, now, self._door_time())

        return self.lift.pos, {fl: p.pos for fl, p in self.doors.items()}

    def phase_name(self):
        return {DOORS_CLOSING: 'CLOSING_DOORS', MOVING: 'MOVING', DOORS_OPENING: 'OPENING_DOORS',
                DOORS_OPEN: 'DONE'}.get(self.state, self.state)
