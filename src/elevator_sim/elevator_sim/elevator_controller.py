"""ROS 2 elevator controller - one node per elevator.

Interfaces (for elevator <eid>, e.g. e1):
  action  /<eid>/goto_floor      elevator_interfaces/action/GotoFloor
  service /<eid>/command         elevator_interfaces/srv/ElevatorCommand
  topic   /<eid>/state           elevator_interfaces/msg/ElevatorState   (10 Hz)
Gazebo side (via ros_gz_bridge):
  pub /<eid>/lift_cmd, /<eid>/door_f<N>_cmd   std_msgs/Float64  (joint position set-points)
  sub /<eid>/joint_states                       sensor_msgs/JointState
  sub /gz_poses                                 tf2_msgs/TFMessage (ground-truth model poses,
                                                used as a simulated doorway light curtain)
"""
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

from elevator_interfaces.action import GotoFloor
from elevator_interfaces.msg import ElevatorState
from elevator_interfaces.srv import ElevatorCommand
from elevator_sim.building import Building
from elevator_sim import elevator_fsm as fsm

RATE_HZ = 20.0


class ElevatorController(Node):
    def __init__(self):
        super().__init__('elevator_controller')
        self.declare_parameter('elevator_id', 'e1')
        self.declare_parameter('building_config', '')
        self.declare_parameter('robot_names', ['tb3_1', 'tb3_2'])
        self.eid = self.get_parameter('elevator_id').value
        cfg = self.get_parameter('building_config').value or None
        self.b = Building.load(cfg)
        e = self.b.elevators[self.eid]
        self.robot_names = list(self.get_parameter('robot_names').value)
        self.fsm = fsm.ElevatorFSM(self.b.floors, self.b.door_travel(self.eid), e['door']['open_dir'],
                                   speed=e['speed'], door_speed=e['door_speed'])
        self.lock = threading.Lock()

        self.lift_pub = self.create_publisher(Float64, f'/{self.eid}/lift_cmd', 10)
        self.door_pubs = {f: self.create_publisher(Float64, f'/{self.eid}/door_f{f}_cmd', 10)
                          for f in self.b.floor_numbers}
        self.state_pub = self.create_publisher(ElevatorState, f'/{self.eid}/state', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/elevator_markers', 10)
        self.create_subscription(JointState, f'/{self.eid}/joint_states', self.on_joints, 10)
        # Runs on wall-clock timers but uses simulation time (joint-state stamps) for all control
        # decisions; launch with use_sim_time:=false so rclpy does not subscribe to /clock.
        self.create_subscription(TFMessage, '/gz_poses', self.on_poses, 10)

        self.lift_meas = None
        self.door_meas = {f: 0.0 for f in self.b.floor_numbers}
        self.robot_pose = {}           # name -> (x, y, z)
        self.last_t = None
        self.sim_t = 0.0               # latest simulation time (from joint-state stamps)
        self.active = None             # (goal handle, Future) of the running goal
        self.fb_count = 0

        self.srv = self.create_service(ElevatorCommand, f'/{self.eid}/command', self.on_command)
        self.action = ActionServer(self, GotoFloor, f'/{self.eid}/goto_floor', execute_callback=self.execute,
                                   goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.create_timer(0.1, self.publish_state)
        self.get_logger().info(f'elevator {self.eid} ready (floors {self.b.floor_numbers})')

    # -------------------------------------------------------------- sensing
    def on_joints(self, msg: JointState):
        # The joint-state stamp is Gazebo's simulation time: use it as this node's clock instead of
        # subscribing to /clock (hundreds of Hz - expensive in Python).
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp > 0.0:
            self.sim_t = stamp
        for name, pos in zip(msg.name, msg.position):
            if name == 'lift':
                self.lift_meas = pos
            elif name.startswith('door_f') and name.endswith('_joint'):
                self.door_meas[int(name[len('door_f'):-len('_joint')])] = pos

    def on_poses(self, msg: TFMessage):
        for t in msg.transforms:
            if t.child_frame_id in self.robot_names:
                tr = t.transform.translation
                self.robot_pose[t.child_frame_id] = (tr.x, tr.y, tr.z)

    def doorway_blocked(self):
        f = self.fsm.current_floor
        fz = self.b.floor_z(f)
        x0, x1, y0, y1 = self.b.doorway_zone(self.eid)
        for name, (x, y, z) in self.robot_pose.items():
            if abs(z - fz) < 0.5 and x0 <= x <= x1 and y0 <= y <= y1:
                return True
        return False

    # ----------------------------------------------------------------- loop
    def now(self):
        return self.sim_t

    def stamp(self):
        sec = int(self.sim_t)
        return rclpy.time.Time(seconds=sec, nanoseconds=int((self.sim_t - sec) * 1e9)).to_msg()

    def tick(self):
        if self.lift_meas is None:
            return                       # Gazebo not up yet
        t = self.now()
        dt = 0.0 if self.last_t is None else max(0.0, min(0.1, t - self.last_t))
        self.last_t = t
        if dt == 0.0:
            self.service_goal()
            return
        with self.lock:
            lift_cmd, door_cmd = self.fsm.step(t, dt, self.lift_meas, self.door_meas, self.doorway_blocked())
            events, self.fsm.events = self.fsm.events, []
        self.lift_pub.publish(Float64(data=float(lift_cmd)))
        for f, p in door_cmd.items():
            self.door_pubs[f].publish(Float64(data=float(p)))
        for ev in events:
            self.get_logger().info(f'[{self.eid}] {ev}')
        self.service_goal()

    def current_floor_reported(self):
        if self.fsm.state == fsm.MOVING or self.lift_meas is None:
            return -1
        return self.fsm.current_floor

    def publish_state(self):
        m = ElevatorState()
        m.header.stamp = self.stamp()
        m.elevator_id = self.eid
        with self.lock:
            m.state = self.fsm.state
            m.current_floor = self.current_floor_reported()
            m.target_floor = self.fsm.target_floor if self.fsm.target_floor is not None else -1
            m.height = float((self.lift_meas or 0.0) + self.fsm.z0)
            m.door_opening = self.fsm.door_fraction(self.door_meas.get(self.fsm.current_floor, 0.0))
            m.doorway_blocked = bool(self.fsm.last_blocked)
            m.owner = self.fsm.owner
            m.fault = self.fsm.fault
        self.state_pub.publish(m)
        self.publish_marker(m)

    def publish_marker(self, m):
        cx, cy = self.b.elevators[self.eid]['cabin_center']
        x0, x1, y0, y1 = self.b.cabin_box(self.eid)
        stamp = m.header.stamp
        box = Marker(type=Marker.CUBE, action=Marker.ADD, ns=self.eid, id=0)
        box.header.frame_id, box.header.stamp = 'map', stamp
        box.pose.position.x, box.pose.position.y, box.pose.position.z = cx, cy, 0.05
        box.pose.orientation.w = 1.0
        box.scale.x, box.scale.y, box.scale.z = x1 - x0, y1 - y0, 0.1
        color = {'MOVING': (1.0, 0.8, 0.0), 'DOORS_OPEN': (0.1, 0.8, 0.2), 'FAULT': (1.0, 0.0, 0.0),
                 'EMERGENCY': (1.0, 0.0, 0.0)}.get(m.state, (0.4, 0.6, 1.0))
        box.color.r, box.color.g, box.color.b, box.color.a = (*color, 0.5)
        txt = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns=self.eid, id=1)
        txt.header.frame_id, txt.header.stamp = 'map', stamp
        txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = cx, cy, 0.6
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.3
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        floor = m.current_floor if m.current_floor > 0 else f'{m.height:.1f}m'
        txt.text = f'{self.eid.upper()} F{floor}\n{m.state}' + (f'\n{m.fault}' if m.fault else '')
        self.marker_pub.publish(MarkerArray(markers=[box, txt]))

    # -------------------------------------------------------------- action
    def on_goal(self, goal):
        with self.lock:
            ok, why = (False, 'gazebo not connected') if self.lift_meas is None else \
                (not self.fsm.busy() and self.fsm.state not in (fsm.FAULT, fsm.EMERGENCY), '')
        if not ok:
            self.get_logger().warn(f'[{self.eid}] rejecting goal floor {goal.floor} from '
                                   f'"{goal.requester}" ({why or self.fsm.state})')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    async def execute(self, handle):
        """Non-blocking: the goal is completed from tick() via a future, so a single-threaded
        executor suffices (rclpy's MultiThreadedExecutor costs a lot of CPU)."""
        g = handle.request
        res = GotoFloor.Result()
        ok, msg = self.fsm.request_floor(g.floor, g.requester)
        if not ok:
            res.success, res.floor, res.message = False, self.fsm.current_floor, msg
            handle.abort()
            return res
        fut = Future()
        self.active = (handle, fut)
        outcome, (success, message) = await fut
        self.active = None
        res.success, res.message, res.floor = success, message, self.current_floor_reported()
        {'succeed': handle.succeed, 'abort': handle.abort, 'canceled': handle.canceled}[outcome]()
        return res

    def service_goal(self):
        """Called from tick(): feedback, cancellation and completion of the active goal."""
        if self.active is None:
            return
        handle, fut = self.active
        if fut.done():
            return
        if handle.is_cancel_requested:
            self.fsm.cancel_goal(self.now())
            fut.set_result(('canceled', (False, 'canceled')))
            return
        result = self.fsm.goal_result
        if result is not None:
            fut.set_result(('succeed' if result[0] else 'abort', result))
            return
        self.fb_count = (self.fb_count + 1) % 4          # feedback at 5 Hz
        if self.fb_count == 0:
            fb = GotoFloor.Feedback()
            fb.phase = self.fsm.phase_name()
            fb.current_floor = self.current_floor_reported()
            fb.height = float((self.lift_meas or 0.0) + self.fsm.z0)
            fb.door_opening = self.fsm.door_fraction(self.door_meas.get(self.fsm.current_floor, 0.0))
            handle.publish_feedback(fb)

    # ------------------------------------------------------------- service
    def on_command(self, req, resp):
        C = ElevatorCommand.Request
        t = self.now()
        with self.lock:
            if req.command == C.CLOSE_DOORS:
                ok, msg = self.fsm.request_close()
            elif req.command == C.OPEN_DOORS:
                ok, msg = self.fsm.request_open(t)
            elif req.command == C.BLOCK_DOOR:
                self.fsm.blocked_override, ok, msg = True, True, 'simulated obstruction ON'
            elif req.command == C.CLEAR_DOOR:
                self.fsm.blocked_override, ok, msg = False, True, 'simulated obstruction OFF'
            elif req.command == C.EMERGENCY:
                self.fsm.emergency(t)
                ok, msg = True, 'EMERGENCY'
            elif req.command == C.RESET:
                ok, msg = self.fsm.reset(t, self.lift_meas or 0.0, self.door_meas)
            else:
                ok, msg = False, f'unknown command {req.command}'
        self.get_logger().info(f'[{self.eid}] command {req.command} from "{req.requester}": {msg}')
        resp.accepted, resp.message = ok, msg
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = ElevatorController()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
