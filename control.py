# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""MAVROS mission state machine shared by indoor and outdoor flights."""

import math
import threading
import time

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
        self.vehicle = None
        self.pose = None
        self.velocity = None
        self.last_state = 0.0
        self.last_pose = 0.0
        self.mode_requested = 0.0
        self.arm_requested = 0.0
        self.takeoff_requested = False
        self.pending = None
        self.state = 'IDLE'
        self.entered = time.monotonic()
        self.desired = None
        self.desired_yaw = 0.0
        self.arrived_since = None
        self.navigation_bad_since = None
        self.after_land = 'DONE'
        self.resume_target = None
        self.takeoff_height = 0.0
        self.launch_local = None
        self.land_requested = False

        self.setpoint_pub = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.create_subscription(State, '/mavros/state', self._state_cb, 10,
                                 callback_group=self.sensor_group)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._pose_cb,
                                 qos_profile_sensor_data, callback_group=self.sensor_group)
        self.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
                                 self._velocity_cb, qos_profile_sensor_data,
                                 callback_group=self.sensor_group)
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode', callback_group=self.service_group)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming',
                                             callback_group=self.service_group)
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff',
                                                 callback_group=self.service_group)
        self.create_service(Trigger, '~/abort', self._abort_cb, callback_group=self.service_group)
        self.create_timer(0.1, self._tick_locked, callback_group=self.logic_group)
        self.create_timer(0.05, self._publish_locked, callback_group=self.output_group)

    def _state_cb(self, msg):
        with self.lock:
            self.vehicle = msg
            self.last_state = time.monotonic()

    def _pose_cb(self, msg):
        with self.lock:
            self.pose = msg.pose
            self.last_pose = time.monotonic()

    def _velocity_cb(self, msg):
        with self.lock:
            self.velocity = msg.twist.linear

    def _enter(self, state):
        if self.state == state:
            return
        self.state = state
        self.entered = time.monotonic()
        self.arrived_since = None
        self.get_logger().info(f'state -> {state}')

    def _call(self, client, request, accepted, label):
        if self.pending or not client.service_is_ready():
            return False
        self.pending = label
        future = client.call_async(request)

        def finished(done):
            with self.lock:
                self.pending = None
                try:
                    response = done.result()
                    if not accepted(response):
                        self._fail(f'{label} rejected (result {getattr(response, "result", "?")})')
                except Exception as error:
                    self._fail(f'{label} failed: {error}')

        future.add_done_callback(finished)
        return True

    def _set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode
        return self._call(self.mode_client, request, lambda r: r.mode_sent, f'mode {mode}')

    def _arm(self):
        request = CommandBool.Request()
        request.value = True
        return self._call(self.arm_client, request, lambda r: r.success, 'arm')

    def _takeoff(self):
        request = CommandTOL.Request()
        request.altitude = float(self.takeoff_height)
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
        if self.velocity is None:
            return math.inf
        v = self.velocity
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    def ready(self):
        return True

    def start_mission(self):
        """Called once after position and link are ready."""
        self.launch_local = self.position()

    def after_takeoff(self):
        raise NotImplementedError

    def mission_tick(self):
        raise NotImplementedError

    def _reached(self, target, radius, max_speed=None, dwell=0.0):
        pos = self.position()
        if pos is None or target is None or math.dist(pos, target) > radius or (max_speed is not None and self.speed() > max_speed):
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
        if self.state not in ('SET_MODE', 'ARM', 'LAND') and self.vehicle.mode != 'GUIDED':
            self.get_logger().warn(f'pilot/mode takeover: {self.vehicle.mode}; stopping setpoints')
            self.desired = None
            self._enter('PASSIVE')
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
                if self.after_land == 'DONE':
                    self._enter('DONE')
                elif now - self.entered > 2.0:
                    self.desired = self.resume_target
                    self._enter('SET_MODE')
                return
            if not self.land_requested and self.pending is None:
                if self._set_mode('LAND'):
                    self.land_requested = True
            if now - self.entered > 60.0:
                self._fail('landing timeout')
            return
        if not self.vehicle.armed:
            self._fail('unexpected disarm')
            return
        if not self.ready():
            if self.navigation_bad_since is None:
                self.navigation_bad_since = now
                self.get_logger().warn('navigation sensors temporarily unavailable')
            elif now - self.navigation_bad_since > 5.0:
                self._fail('required navigation sensors lost for over 5 s')
            return
        self.navigation_bad_since = None
        self.mission_tick()

    def _publish_locked(self):
        with self.lock:
            self._publish()

    def _publish(self):
        if self.state in ('IDLE', 'SET_MODE', 'ARM', 'TAKEOFF', 'LAND', 'DONE', 'PASSIVE', 'ABORT') or self.desired is None:
            return
        if self.vehicle is None or not self.vehicle.connected or self.vehicle.mode != 'GUIDED':
            return
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
        target.position.x, target.position.y, target.position.z = self.desired
        current_yaw = self.yaw()
        target.yaw = float(unwrap_near(self.desired_yaw, current_yaw or 0.0))
        self.setpoint_pub.publish(target)


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
