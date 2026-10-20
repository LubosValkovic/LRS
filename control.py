# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""MAVROS mission state machine shared by indoor and outdoor flights."""

import math
import threading
import time
from operator import attrgetter

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from .mission import unwrap_near, yaw_from_quaternion


class FlightNode(Node):
    """Common service handshake, setpoint timer and safety state handling."""

    def __init__(self, name):
        super().__init__(name)
        self.lock = threading.RLock()
        self.sensor_group = MutuallyExclusiveCallbackGroup()
        self.logic_group = MutuallyExclusiveCallbackGroup()
        self.output_group = MutuallyExclusiveCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.vehicle = self.pose = self.velocity = None
        self.last_state = self.last_pose = self.last_velocity = 0.0
        self.mode_requested = self.arm_requested = 0.0
        self.takeoff_requested = self.land_requested = False
        self.pending = self.pending_future = None
        self.pending_since = 0.0
        self.state = 'IDLE'
        self.entered = time.monotonic()
        self.desired = self.resume_target = None
        self.desired_yaw = 0.0
        self.arrived_since = self.navigation_bad_since = self.landed_since = None
        self.after_land = 'DONE'
        self.takeoff_height = 0.0
        self.launch_local = None

        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self._subscribe_sensor(State, '/mavros/state', 'vehicle', 'last_state', qos=10)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb,
                                 qos_profile_sensor_data, callback_group=self.sensor_group)
        self._subscribe_sensor(TwistStamped, '/mavros/local_position/velocity_local',
                               'velocity', 'last_velocity', 'twist.linear')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode', callback_group=self.service_group)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming',
                                             callback_group=self.service_group)
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff',
                                                 callback_group=self.service_group)
        self.create_service(Trigger, '~/abort', self._abort_cb, callback_group=self.service_group)
        self.create_timer(0.1, self._tick_locked, callback_group=self.logic_group)
        self.create_timer(0.05, self._publish_locked, callback_group=self.output_group)

    def _parameter(self, name, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _subscribe_sensor(self, msg_type, topic, attribute, timestamp, field=None, qos=qos_profile_sensor_data):
        """Store a sensor value and its receive time using the sensor lock/group."""
        extract = attrgetter(field) if field else lambda msg: msg

        def receive(msg):
            with self.lock:
                setattr(self, attribute, extract(msg))
                setattr(self, timestamp, time.monotonic())

        self.create_subscription(msg_type, topic, receive, qos, callback_group=self.sensor_group)

    def _pose_cb(self, msg):
        with self.lock:
            p, q = msg.pose.position, msg.pose.orientation
            if (not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                    or q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w < 1e-12):
                self.last_pose = 0.0
                return
            self.pose = msg.pose
            self.last_pose = time.monotonic()

    def _enter(self, state):
        if self.state == state:
            return
        self.state = state
        self.entered = time.monotonic()
        self.arrived_since = None
        if state == 'LAND':
            self.landed_since = None
        self.get_logger().info(f'state -> {state}')

    def _call(self, client, request, accepted, label):
        if self.pending or not client.service_is_ready():
            return False
        self.pending = label
        self.pending_since = time.monotonic()
        try:
            future = client.call_async(request)
        except Exception as error:
            self.pending = None
            self._fail(f'{label} failed: {error}')
            return False
        self.pending_future = future

        def finished(done):
            with self.lock:
                if self.pending_future is not done:
                    return  # Ignore a late response to a timed-out request.
                self.pending = None
                self.pending_future = None
                if self.state in ('DONE', 'PASSIVE'):
                    return
                try:
                    response = done.result()
                    if not accepted(response):
                        self._fail(f'{label} rejected (result {getattr(response, "result", "?")})')
                except Exception as error:
                    self._fail(f'{label} failed: {error}')

        future.add_done_callback(finished)
        return True

    def _set_mode(self, mode):
        return self._call(self.mode_client, SetMode.Request(custom_mode=mode),
                          lambda r: r.mode_sent, f'mode {mode}')

    def _arm(self):
        return self._call(self.arm_client, CommandBool.Request(value=True), lambda r: r.success, 'arm')

    def _takeoff(self):
        request = CommandTOL.Request(altitude=float(self.takeoff_height))
        if self._call(self.takeoff_client, request, lambda r: r.success, 'takeoff'):
            self.takeoff_requested = True

    def _fail(self, message):
        self.get_logger().error(message)
        self.desired = None
        self._enter('ABORT')
        self.mode_requested = 0.0

    def _abort_cb(self, request, response):
        del request
        with self.lock:
            self._fail('abort requested')
            response.success = True
            response.message = 'setpoints stopped; requesting RTL when connected and still in GUIDED'
        return response

    def position(self):
        if self.pose is None:
            return None
        p = self.pose.position
        return (p.x, p.y, p.z)

    def yaw(self):
        return yaw_from_quaternion(self.pose.orientation) if self.pose else None

    def speed(self):
        if self.velocity is None or time.monotonic() - self.last_velocity > 3.0:
            return math.inf
        v = self.velocity
        return math.hypot(v.x, v.y, v.z) if all(math.isfinite(x) for x in (v.x, v.y, v.z)) else math.inf

    def ready(self):
        return True

    def start_mission(self):
        """Called once after position and link are ready."""
        self.launch_local = self.position()

    def after_takeoff(self):
        raise NotImplementedError

    def mission_tick(self):
        raise NotImplementedError

    def navigation_resumed(self, duration):
        """Allow timed trajectories to exclude a temporary sensor outage."""

    def _navigation_ready(self, now):
        if not self.ready():
            self.arrived_since = None
            if self.navigation_bad_since is None:
                self.navigation_bad_since = now
                self.get_logger().warn('navigation sensors temporarily unavailable')
            elif now - self.navigation_bad_since > 5.0:
                self._fail('required navigation sensors lost for over 5 s')
            return False
        if self.navigation_bad_since is not None:
            self.navigation_resumed(now - self.navigation_bad_since)
            self.navigation_bad_since = None
        return True

    def _reached(self, target, radius, max_speed=None, dwell=0.0):
        pos = self.position()
        if (pos is None or target is None
                or not all(math.isfinite(v) for v in (*pos, *target))
                or math.dist(pos, target) > radius
                or (max_speed is not None and self.speed() > max_speed)):
            self.arrived_since = None
            return False
        now = time.monotonic()
        if self.arrived_since is None:
            self.arrived_since = now
        return now - self.arrived_since >= dwell

    def _tick_locked(self):
        with self.lock:
            self._tick()

    def _tick(self):
        now = time.monotonic()
        if self.state in ('DONE', 'PASSIVE'):
            return
        if self.pending is not None and now - self.pending_since > 5.0:
            label, future = self.pending, self.pending_future
            self.pending = self.pending_future = None
            if future is not None:
                future.cancel()
            self._fail(f'{label} service timeout')
        if self.state == 'ABORT':
            if (self.vehicle and self.vehicle.connected and self.vehicle.armed
                    and self.vehicle.mode == 'GUIDED'
                    and self.pending is None and now - self.mode_requested > 3.0):
                if self._set_mode('RTL'):
                    self.mode_requested = now
            return
        if self.vehicle is None or not self.vehicle.connected or now - self.last_state > 5.0:
            if self.state != 'IDLE':
                self._fail('MAVROS state lost')
            return
        if self.pose is None or now - self.last_pose > 5.0:
            if self.state != 'IDLE':
                self._fail('local pose lost')
            return
        if self.state == 'IDLE':
            if self.ready():
                self.start_mission()
                if self.state == 'IDLE':
                    self._enter('SET_MODE')
            return
        if self.state == 'LAND' and self.vehicle.mode not in ('GUIDED', 'LAND'):
            self.get_logger().warn(f'pilot/mode takeover: {self.vehicle.mode}; stopping mission')
            self._enter('PASSIVE')
            return
        if self.state not in ('SET_MODE', 'LAND') and self.vehicle.mode != 'GUIDED':
            self.get_logger().warn(f'pilot/mode takeover: {self.vehicle.mode}; stopping setpoints')
            self.desired = None
            self._enter('PASSIVE')
            return
        if self.state != 'LAND' and not self._navigation_ready(now):
            return
        if self.state == 'SET_MODE':
            if self.vehicle.mode == 'GUIDED':
                self._enter('ARM')
            elif self.pending is None and now - self.mode_requested > 3.0:
                if self._set_mode('GUIDED'):
                    self.mode_requested = now
            return
        if self.state == 'ARM':
            if self.vehicle.armed:
                self.takeoff_requested = False
                self._enter('TAKEOFF')
            elif self.pending is None and now - self.arm_requested > 3.0:
                if self._arm():
                    self.arm_requested = now
            return
        if self.state == 'TAKEOFF':
            if not self.vehicle.armed:
                self._fail('vehicle disarmed during takeoff')
                return
            if not self.takeoff_requested and self.pending is None:
                self._takeoff()
            if self._reached(self.desired, 0.25, 0.25, 1.0):
                self.after_takeoff()
            elif now - self.entered > 40.0:
                self._fail('takeoff altitude timeout')
            return
        if self.state == 'LAND':
            self.desired = None
            if not self.vehicle.armed:
                if self.landed_since is None:
                    self.landed_since = now
                if self.after_land == 'DONE':
                    self._enter('DONE')
                elif now - self.landed_since >= 2.0:
                    self.desired = self.resume_target
                    self._enter('SET_MODE')
                return
            self.landed_since = None
            if not self.land_requested and self.pending is None:
                if self._set_mode('LAND'):
                    self.land_requested = True
            if now - self.entered > 60.0:
                self._fail('landing timeout')
            return
        if not self.vehicle.armed:
            self._fail('unexpected disarm')
            return
        self.mission_tick()

    def _publish_locked(self):
        with self.lock:
            self._publish()

    def _can_publish(self):
        if self.state in ('IDLE', 'SET_MODE', 'ARM', 'TAKEOFF', 'LAND', 'DONE', 'PASSIVE', 'ABORT') or self.desired is None:
            return False
        now = time.monotonic()
        if (self.vehicle is None or not self.vehicle.connected or not self.vehicle.armed
                or self.vehicle.mode != 'GUIDED' or self.pose is None
                or now - self.last_state > 5.0 or now - self.last_pose > 5.0):
            return False
        return self._navigation_ready(now)

    def _publish(self):
        if not self._can_publish():
            return False
        current_yaw = self.yaw()
        if (current_yaw is None or not math.isfinite(current_yaw)
                or not all(math.isfinite(v) for v in (*self.desired, self.desired_yaw))):
            self._fail('non-finite position or yaw setpoint')
            return False
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
        target.position.x, target.position.y, target.position.z = self.desired
        target.yaw = float(unwrap_near(self.desired_yaw, current_yaw))
        self.setpoint_pub.publish(target)
        return True


def spin(node):
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
