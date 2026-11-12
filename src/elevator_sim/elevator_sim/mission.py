"""Mission runner for one robot.

    ros2 run elevator_sim mission --robot tb3_1 --floor 3 --goal room_b

Goes from the robot's home on floor 1 to <goal> on <floor> with the robot's own
elevator and back home. Appends a row to the metrics CSV (see --metrics).
"""
import argparse
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from elevator_interfaces.action import GotoFloor
from elevator_interfaces.msg import ElevatorState
from elevator_interfaces.srv import ElevatorCommand
from elevator_sim.building import Building, yaw_to_quat
from elevator_sim.cabin_localizer import fit_cabin_pose, scan_points
from elevator_sim.metrics import append_result, default_metrics_path
from elevator_sim.mission_core import MissionExecutor, normalize_angle


class Nav2ElevatorAdapter:
    """Implements the adapter interface used by MissionExecutor on top of Nav2 + elevator ROS API."""

    def __init__(self, robot, eid, radius=0.15):
        self.robot, self.eid, self.radius = robot, eid, radius
        self.nav = BasicNavigator(node_name=f'{robot}_mission', namespace=robot)
        n = self.nav
        n.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.tf = Buffer()
        self.tfl = TransformListener(self.tf, n, spin_thread=False)
        self.state = None
        n.create_subscription(ElevatorState, f'/{eid}/state', self._on_state, 10)
        self.goto = ActionClient(n, GotoFloor, f'/{eid}/goto_floor')
        self.cmd = n.create_client(ElevatorCommand, f'/{eid}/command')
        self.cmd_vel = n.create_publisher(Twist, f'/{robot}/cmd_vel', 10)
        self.scan = None
        self.lidar_pose = None
        self.initpose_pub = n.create_publisher(PoseWithCovarianceStamped, f'/{robot}/initialpose', 10)
        n.create_subscription(LaserScan, f'/{robot}/scan', self._on_scan, rclpy.qos.qos_profile_sensor_data)

    # ------------------------------------------------------------ basics
    def _on_state(self, msg):
        self.state = msg

    def _on_scan(self, msg):
        self.scan = msg

    def front_clearance(self, half_angle=0.6):
        """Smallest LiDAR range within +-half_angle of straight ahead (inf if no scan)."""
        s = self.scan
        if s is None:
            return float('inf')
        best = float('inf')
        for i, r in enumerate(s.ranges):
            a = normalize_angle(s.angle_min + i * s.angle_increment)
            if abs(a) <= half_angle and s.range_min <= r <= s.range_max:
                best = min(best, r)
        return best

    def spin(self, sec=0.05):
        rclpy.spin_once(self.nav, timeout_sec=sec)

    def now(self):
        return self.nav.get_clock().now().nanoseconds * 1e-9

    def sleep(self, sec):
        end = self.now() + sec
        while rclpy.ok() and self.now() < end:
            self.spin(0.05)

    def wait_ready(self, timeout=180.0):
        self.nav.info('waiting for Nav2 to become active ...')
        # Not waitUntilNav2Active(): it would re-publish an initial pose and reset AMCL.
        # AMCL gets its start pose from its parameters (set_initial_pose) instead.
        self.nav._waitForNodeToActivate('amcl')
        self.nav._waitForNodeToActivate('bt_navigator')
        t0 = time.time()
        while rclpy.ok() and (self.state is None or not self.goto.server_is_ready()):
            self.spin(0.1)
            if time.time() - t0 > timeout:
                raise RuntimeError(f'elevator {self.eid} not available')
        while rclpy.ok() and self.pose() is None:
            self.spin(0.1)
            if time.time() - t0 > timeout:
                raise RuntimeError('no localization (map -> base_footprint TF)')

    def elevator_state(self):
        self.spin(0.01)
        return self.state

    def pose(self):
        try:
            t = self.tf.lookup_transform('map', f'{self.robot}/base_footprint', Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    # ------------------------------------------------- cabin re-localization
    def _lidar_mount(self):
        if self.lidar_pose is None:
            try:
                t = self.tf.lookup_transform(f'{self.robot}/base_footprint', f'{self.robot}/base_scan', Time())
                q = t.transform.rotation
                self.lidar_pose = (t.transform.translation.x, t.transform.translation.y,
                                   math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            except Exception:  # noqa: BLE001
                return None
        return self.lidar_pose

    def relocalize_in_cabin(self, interior):
        """Fit the current LiDAR scan to the cabin walls and reset AMCL to the result."""
        self.sleep(0.5)                                   # make sure the scan is from standstill
        mount, guess, scan = self._lidar_mount(), self.pose(), self.scan
        if mount is None or guess is None or scan is None:
            return False, 'no scan / pose / LiDAR transform'
        pts = scan_points(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max, mount)
        pose, info = fit_cabin_pose(pts, guess, interior)
        if not info['ok']:
            return False, f'cabin fit rejected ({info})'
        m = PoseWithCovarianceStamped()
        m.header.frame_id = 'map'
        m.header.stamp = self.nav.get_clock().now().to_msg()
        m.pose.pose.position.x, m.pose.pose.position.y = float(pose[0]), float(pose[1])
        _, _, m.pose.pose.orientation.z, m.pose.pose.orientation.w = yaw_to_quat(pose[2])
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.03 ** 2
        cov[35] = math.radians(2.0) ** 2
        m.pose.covariance = cov
        self.initpose_pub.publish(m)
        self.sleep(1.0)                                   # let AMCL re-seed and update map->odom
        corr = math.hypot(pose[0] - guess[0], pose[1] - guess[1])
        dyaw = math.degrees(normalize_angle(pose[2] - guess[2]))
        return True, (f'cabin fit: corrected {corr:.2f} m / {dyaw:.1f} deg '
                      f'(inliers {info["inliers"]}, rms {info["rms"] * 100:.1f} cm)')

    # ---------------------------------------------------------- navigation
    def _wait_task(self, timeout):
        t_end = self.now() + timeout
        while not self.nav.isTaskComplete():
            if self.now() > t_end:
                self.nav.cancelTask()
                return False, f'timeout after {timeout:.0f}s'
        r = self.nav.getResult()
        return (r == TaskResult.SUCCEEDED), str(r)

    def navigate(self, x, y, yaw, timeout):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.nav.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        _, _, p.pose.orientation.z, p.pose.orientation.w = yaw_to_quat(yaw)
        if not self.nav.goToPose(p):
            return False, 'goal rejected'
        return self._wait_task(timeout)

    def turn(self, angle):
        if abs(angle) < 0.03:
            return True, 'aligned'
        if not self.nav.spin(spin_dist=float(angle), time_allowance=30):
            return False, 'spin rejected'
        return self._wait_task(60.0)

    def face(self, x, y):
        p = self.pose()
        if p is None:
            return False, 'no pose'
        return self.turn(normalize_angle(math.atan2(y - p[1], x - p[0]) - p[2]))

    def drive_to(self, x, y, speed):
        """Turn towards (x, y), then drive straight onto it (DriveOnHeading)."""
        ok, msg = self.face(x, y)
        if not ok:
            return False, f'align: {msg}'
        # Marks recorded before the turn can be misplaced by odometry error during the turn
        # (e.g. cabin walls appearing in the doorway): start the straight drive from a fresh local costmap.
        try:
            self.nav.clearLocalCostmap()
        except Exception:  # noqa: BLE001
            pass
        self.sleep(0.6)                                  # > one LiDAR period (5 Hz)
        p = self.pose()
        dist = math.hypot(x - p[0], y - p[1])
        if dist < 0.05:
            return True, 'already there'
        allowance = int(dist / speed * 2.0 + 10)
        if self.nav.driveOnHeading(dist=float(dist), speed=float(speed), time_allowance=allowance):
            ok, msg = self._wait_task(allowance + 10.0)
            if ok:
                return True, msg
        else:
            msg = 'drive rejected'
        # DriveOnHeading checks the inflated costmap, which near the door frame can refuse a move
        # that is physically clear. Fall back to a slow LiDAR-guarded creep onto the target.
        self.nav.info(f'  DriveOnHeading stopped ({msg}); creeping to target with LiDAR guard')
        return self.creep_to(x, y, min(speed, 0.08))

    def creep_to(self, x, y, speed, tol=0.06):
        p = self.pose()
        if p is None:
            return False, 'no pose'
        t_end = self.now() + math.hypot(x - p[0], y - p[1]) / speed * 3.0 + 15.0
        stop_dist = self.radius + 0.08
        try:
            while rclpy.ok():
                self.spin(0.05)
                p = self.pose()
                if p is None:
                    continue
                dist = math.hypot(x - p[0], y - p[1])
                if dist < tol:
                    return True, 'reached (creep)'
                if self.now() > t_end:
                    return False, 'creep timeout'
                err = normalize_angle(math.atan2(y - p[1], x - p[0]) - p[2])
                cmd = Twist()
                cmd.angular.z = max(-0.4, min(0.4, 1.5 * err))
                if abs(err) < 0.35:
                    if self.front_clearance() < stop_dist:
                        self.cmd_vel.publish(Twist())
                        return False, f'obstacle within {stop_dist:.2f} m ahead (creep)'
                    cmd.linear.x = min(speed, 0.5 * dist + 0.02)
                self.cmd_vel.publish(cmd)
        finally:
            for _ in range(3):
                self.cmd_vel.publish(Twist())

    def clear_costmaps(self):
        try:
            self.nav.clearAllCostmaps()
        except Exception as ex:  # noqa: BLE001
            self.nav.warn(f'clear costmaps failed: {ex}')

    def safe_stop(self):
        try:
            self.nav.cancelTask()
        except Exception:  # noqa: BLE001
            pass
        for _ in range(5):
            self.cmd_vel.publish(Twist())
            self.spin(0.05)

    # ------------------------------------------------------------ elevator
    def elevator_goto(self, floor, timeout):
        if not self.goto.wait_for_server(timeout_sec=10.0):
            return False, 'elevator action server unavailable'
        g = GotoFloor.Goal(floor=int(floor), requester=self.robot)
        last_phase = ['']

        def fb(msg):
            if msg.feedback.phase != last_phase[0]:
                last_phase[0] = msg.feedback.phase
                self.nav.info(f'  {self.eid}: {msg.feedback.phase} (h={msg.feedback.height:.2f} m)')

        fut = self.goto.send_goal_async(g, feedback_callback=fb)
        rclpy.spin_until_future_complete(self.nav, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return False, 'goal rejected by elevator'
        rfut = gh.get_result_async()
        t_end = self.now() + timeout
        while rclpy.ok() and not rfut.done():
            self.spin(0.1)
            if self.now() > t_end:
                gh.cancel_goal_async()
                return False, f'elevator timeout after {timeout:.0f}s'
        r = rfut.result().result
        return bool(r.success), r.message

    def elevator_close(self):
        if not self.cmd.wait_for_service(timeout_sec=5.0):
            return False, 'command service unavailable'
        fut = self.cmd.call_async(ElevatorCommand.Request(command=ElevatorCommand.Request.CLOSE_DOORS,
                                                          requester=self.robot))
        rclpy.spin_until_future_complete(self.nav, fut, timeout_sec=5.0)
        r = fut.result()
        return (bool(r.accepted), r.message) if r else (False, 'no response')


def main(argv=None):
    argv = rclpy.utilities.remove_ros_args(sys.argv if argv is None else argv)[1:]
    ap = argparse.ArgumentParser(description='Run one elevator mission for one robot.')
    ap.add_argument('--robot', required=True, help='tb3_1 or tb3_2')
    ap.add_argument('--floor', required=True, type=int, help='target floor (1 = ground)')
    ap.add_argument('--goal', required=True, help='room name on that floor, e.g. room_b')
    ap.add_argument('--config', default=None, help='building.yaml (default: installed copy)')
    ap.add_argument('--metrics', default=default_metrics_path(), help='CSV file to append results to')
    ap.add_argument('--mode', default='single', help='label stored in the metrics row')
    a = ap.parse_args(argv)

    b = Building.load(a.config)
    try:
        MissionExecutor(b, a.robot, a.floor, a.goal, adapter=None)   # validate arguments early
    except ValueError as ex:
        ap.error(str(ex))

    # Keep the ROS context alive on Ctrl-C / SIGTERM so the robot can still be stopped safely.
    rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)

    def _interrupt(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _interrupt)
    signal.signal(signal.SIGINT, _interrupt)
    adapter = Nav2ElevatorAdapter(a.robot, b.robots[a.robot]['elevator'], float(b.robots[a.robot]['radius']))
    log = adapter.nav.get_logger().info
    try:
        adapter.wait_ready()
        ex = MissionExecutor(b, a.robot, a.floor, a.goal, adapter, log=log)
        res = ex.run()
        path = append_result(a.metrics, res, mode=a.mode)
        log(f'metrics appended to {path}')
        code = 0 if res.success else 1
    except KeyboardInterrupt:
        adapter.nav.get_logger().warn('interrupted - cancelling navigation and stopping the robot')
        adapter.safe_stop()
        code = 130
    finally:
        adapter.nav.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
