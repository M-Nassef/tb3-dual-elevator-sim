"""Mission planning + execution logic (pure Python; ROS access goes through an adapter).

A mission takes one robot from its home on the ground floor to a named room on a
target floor using *its own* elevator, then brings it back home:

  NAV landing(1) -> CALL(1) -> BOARD -> RIDE(T) -> EXIT -> CLOSE
  -> NAV room(T) -> NAV landing(T) -> CALL(T) -> BOARD -> RIDE(1) -> EXIT -> CLOSE -> NAV home(1)

Failure handling
  * navigation: bounded retries, costmaps cleared between attempts
  * elevator calls/rides: bounded retries on rejection/abort (FAULT/EMERGENCY are not retried)
  * boarding/exiting: guarded by elevator state (doors open, cabin at robot's floor) and
    verified with the localized pose afterwards; one corrective attempt
  * any unrecoverable step -> safe stop (cancel Nav2, zero velocity), mission FAILED,
    metrics row records the failing step and reason
"""
import math
import time
from dataclasses import dataclass, field

CATEGORIES = ('nav', 'wait_elevator', 'board_exit', 'ride')


@dataclass
class Step:
    kind: str                 # nav | call | board | ride | exit | close
    floor: int
    label: str
    target: tuple = ()        # (x, y, yaw) for nav
    category: str = 'nav'


@dataclass
class MissionResult:
    robot: str
    elevator: str
    target_floor: int
    goal: str
    success: bool = False
    failed_step: str = ''
    message: str = ''
    sim_durations: dict = field(default_factory=lambda: {c: 0.0 for c in CATEGORIES})
    sim_total: float = 0.0
    wall_total: float = 0.0
    nav_retries: int = 0
    elevator_retries: int = 0
    steps_done: int = 0
    steps_total: int = 0


def build_plan(b, robot: str, floor: int, goal: str, home_floor: int = 1):
    if robot not in b.robots:
        raise ValueError(f'unknown robot "{robot}"; known: {sorted(b.robots)}')
    if goal not in b.rooms:
        raise ValueError(f'unknown goal "{goal}"; known rooms: {sorted(b.rooms)}')
    b.floor_z(floor)  # validates floor
    r = b.robots[robot]
    e = b.elevators[r['elevator']]
    home = tuple(r['home'])
    landing = tuple(e['landing'])
    room = b.rooms[goal]
    if floor == home_floor:
        return [Step('nav', floor, f'nav {goal}@F{floor}', room),
                Step('nav', home_floor, 'nav home', home)]

    def trip(src, dst):
        return [Step('nav', src, f'nav landing@F{src}', landing),
                Step('call', src, f'call {r["elevator"]} to F{src}', category='wait_elevator'),
                Step('board', src, f'board @F{src}', category='board_exit'),
                Step('ride', dst, f'ride F{src}->F{dst}', category='ride'),
                Step('exit', dst, f'exit @F{dst}', category='board_exit'),
                Step('close', dst, f'close doors @F{dst}', category='board_exit')]

    return (trip(home_floor, floor)
            + [Step('nav', floor, f'nav {goal}@F{floor}', room)]
            + trip(floor, home_floor)
            + [Step('nav', home_floor, 'nav home', home)])


class MissionExecutor:
    def __init__(self, b, robot, floor, goal, adapter, log=print, nav_retries=3, elevator_retries=2,
                 nav_timeout=300.0, call_timeout=180.0, ride_timeout=180.0, drive_speed=0.12):
        self.b, self.robot, self.floor, self.goal = b, robot, floor, goal
        self.eid = b.robots[robot]['elevator']
        self.radius = float(b.robots[robot]['radius'])
        self.a = adapter
        self.log = log
        self.nav_retries, self.elevator_retries = nav_retries, elevator_retries
        self.nav_timeout, self.call_timeout, self.ride_timeout = nav_timeout, call_timeout, ride_timeout
        self.drive_speed = drive_speed
        self.plan = build_plan(b, robot, floor, goal)
        self.result = MissionResult(robot, self.eid, floor, goal, steps_total=len(self.plan))

    # ------------------------------------------------------------- run
    def run(self) -> MissionResult:
        res = self.result
        t0_sim, t0_wall = self.a.now(), time.time()
        self.log(f'mission {self.robot}: F{self.floor} {self.goal} via {self.eid} ({len(self.plan)} steps)')
        try:
            for i, step in enumerate(self.plan, 1):
                self.log(f'[{i}/{len(self.plan)}] {step.label}')
                ts = self.a.now()
                ok, msg = getattr(self, f'_do_{step.kind}')(step)
                res.sim_durations[step.category] += self.a.now() - ts
                if not ok:
                    res.failed_step, res.message = step.label, msg
                    self.log(f'step "{step.label}" FAILED: {msg} -> safe stop')
                    self.a.safe_stop()
                    break
                res.steps_done = i
            else:
                res.success, res.message = True, 'mission completed'
        except Exception as ex:  # noqa: BLE001 - any crash must still stop the robot
            res.failed_step = res.failed_step or 'exception'
            res.message = f'{type(ex).__name__}: {ex}'
            self.a.safe_stop()
        res.sim_total = self.a.now() - t0_sim
        res.wall_total = time.time() - t0_wall
        self.log(f'mission {self.robot} {"SUCCEEDED" if res.success else "FAILED"} '
                 f'in {res.sim_total:.1f}s sim / {res.wall_total:.1f}s wall: {res.message}')
        return res

    # ----------------------------------------------------------- steps
    def _do_nav(self, step):
        x, y, yaw = step.target
        last = ''
        for attempt in range(1, self.nav_retries + 1):
            ok, last = self.a.navigate(x, y, yaw, self.nav_timeout)
            if ok:
                return True, 'reached'
            self.result.nav_retries += 1
            self.log(f'  navigation attempt {attempt}/{self.nav_retries} failed: {last}')
            self.a.clear_costmaps()
        return False, f'navigation failed after {self.nav_retries} attempts: {last}'

    def _elevator(self, floor, timeout):
        last = ''
        for attempt in range(1, self.elevator_retries + 2):
            ok, last = self.a.elevator_goto(floor, timeout)
            if ok:
                return True, last
            st = self.a.elevator_state()
            if st is not None and st.state in ('FAULT', 'EMERGENCY'):
                return False, f'elevator {self.eid} in {st.state}: {st.fault or last}'
            self.result.elevator_retries += 1
            self.log(f'  elevator request attempt {attempt} failed: {last}; retrying')
            self.a.sleep(2.0)
        return False, f'elevator request failed: {last}'

    def _do_call(self, step):
        return self._elevator(step.floor, self.call_timeout)

    def _do_ride(self, step):
        ok, msg = self._elevator(step.floor, self.ride_timeout)
        if ok:
            self.a.clear_costmaps()     # obstacles seen on the previous floor are stale now
        return ok, msg

    def _doors_ready(self, floor, wait=5.0):
        """True once the elevator reports doors fully open at `floor`. The state topic runs at
        10 Hz, so right after an action result it may still show the previous state: re-check
        for up to `wait` seconds before giving up."""
        deadline = self.a.now() + wait
        while True:
            st = self.a.elevator_state()
            if st is not None and st.current_floor == floor and st.state == 'DOORS_OPEN' \
                    and st.door_opening >= 0.95:
                return True, ''
            if self.a.now() >= deadline:
                if st is None:
                    return False, 'no elevator state received'
                return False, (f'elevator not ready (state={st.state}, floor={st.current_floor}, '
                               f'door={st.door_opening:.2f}, expected floor {floor})')
            self.a.sleep(0.2)

    def _relocalize(self):
        ok, msg = self.a.relocalize_in_cabin(self.b.cabin_interior(self.eid))
        self.log(f'  {msg}' if ok else f'  warning: cabin re-localization failed: {msg} (keeping AMCL)')
        return ok

    def _center_in_cabin(self, tol=0.10):
        """Drive onto the cabin centre if the (re-localized) pose is noticeably off it."""
        cx, cy = self.b.elevators[self.eid]['cabin_center']
        p = self.a.pose()
        if p and math.hypot(p[0] - cx, p[1] - cy) > tol:
            self.log(f'  re-centering in cabin (offset {math.hypot(p[0] - cx, p[1] - cy):.2f} m)')
            self.a.drive_to(cx, cy, self.drive_speed)

    def _do_board(self, step):
        ok, msg = self._doors_ready(step.floor)
        if not ok:
            return False, msg
        cx, cy = self.b.elevators[self.eid]['cabin_center']
        for attempt in (1, 2):
            ok, msg = self.a.drive_to(cx, cy, self.drive_speed)
            p = self.a.pose()
            if p and self.b.in_cabin(self.eid, p[0], p[1], self.radius * 0.5):
                break
            self.log(f'  boarding attempt {attempt}: not inside cabin yet ({msg}, pose={p})')
            ok2, msg2 = self._doors_ready(step.floor)
            if not ok2:
                return False, f'doors no longer open while boarding: {msg2}'
        else:
            return False, 'robot could not get inside the cabin'
        if self._relocalize():
            self._center_in_cabin()
        lx, ly, _ = self.b.elevators[self.eid]['landing']
        ok, msg = self.a.face(lx, ly)                    # turn around: face the doors
        if ok:
            self._relocalize()                           # heading after the turn, from the walls
        return ok, msg

    def _do_exit(self, step):
        ok, msg = self._doors_ready(step.floor)
        if not ok:
            return False, msg
        if self._relocalize():
            self._center_in_cabin()                      # straight line centre -> landing = door centre
        lx, ly, _ = self.b.elevators[self.eid]['landing']
        door_x = self.b.elevators[self.eid]['door']['x']
        for attempt in (1, 2):
            ok, msg = self.a.drive_to(lx, ly, self.drive_speed)
            p = self.a.pose()
            if p and p[0] > door_x + self.radius + 0.2:
                return True, 'out of the cabin'
            self.log(f'  exit attempt {attempt}: still in doorway/cabin ({msg}, pose={p})')
            if p and self.b.in_cabin(self.eid, p[0], p[1]) and self._relocalize():
                self._center_in_cabin(tol=0.05)
        return False, 'robot could not leave the cabin'

    def _do_close(self, step):
        ok, msg = self.a.elevator_close()
        if not ok:
            self.log(f'  warning: close request not accepted ({msg}); doors will auto-close')
        return True, msg


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))
