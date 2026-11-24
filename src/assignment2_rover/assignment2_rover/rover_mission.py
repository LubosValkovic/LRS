# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Single-drone lawnmower search and own-rover position tracking."""

import math
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data

from assignment1_mission.part02.control import FlightNode, spin
from assignment2_rover.vision import stamp_ns


class RoverMission(FlightNode):
    """Search an unknown area, then stay above the fresh own-marker position.

    Search uses a configurable lawnmower pattern around the launch point. The
    rover's path, starting position and randomizer state are never inputs.
    Horizontal motion is commanded as bounded velocity; a bounded vertical
    velocity controller holds the search or tracking altitude.
    """

    def __init__(self, **node_kwargs):
        super().__init__('rover_mission', **node_kwargs)

        def parameter(name, default):
            return self.declare_parameter(name, default,
                ParameterDescriptor(read_only=True)).value

        self.sysid = parameter('sysid', 1)
        # Clear the 17.2 m collision envelope of the large tree in the supplied
        # world while retaining enough image resolution for marker detection.
        self.search_altitude = parameter('search_altitude', 19.0)
        self.track_altitude = parameter('track_altitude', 2.0)
        self.search_width = parameter('search_width', 30.0)
        self.search_length = parameter('search_length', 30.0)
        self.lane_spacing = parameter('lane_spacing', 5.0)
        self.search_speed = parameter('search_speed', 0.8)
        self.tracking_speed = parameter('tracking_speed', 0.6)
        self.target_timeout = parameter('target_timeout', 1.0)
        self.max_target_age = parameter('max_target_age', 0.75)
        self.velocity_gain = parameter('velocity_gain', 0.8)
        positive = (self.search_altitude, self.track_altitude, self.search_width,
                    self.search_length, self.lane_spacing, self.search_speed,
                    self.tracking_speed, self.target_timeout, self.max_target_age,
                    self.velocity_gain)
        if (not 1 <= self.sysid < 50 or not all(math.isfinite(x) and x > 0 for x in positive)
                or self.target_timeout < self.max_target_age
                or self.search_speed > 1.5 or self.tracking_speed > 1.5):
            raise ValueError('mission geometry/timeouts must be positive; speeds must be <= 1.5 m/s')

        self.search_origin = None
        self.search_points = []
        self.search_index = 0
        self.marker = None
        self.marker_frame = ''
        self.marker_stamp = 0
        self.marker_received = 0.0
        self.targets_rejected = 0
        self.target_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(PointStamped, 'marker_localizer/target', self.target_cb,
                                 qos_profile_sensor_data, callback_group=self.target_group)
        self.mission_status_pub = self.create_publisher(DiagnosticArray, '~/rover_status', 10)
        self.create_timer(0.2, self.publish_mission_status,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def start_mission(self):
        super().start_mission()
        self.search_origin = self.launch_local
        self.takeoff_height = self.search_altitude
        self.desired = (self.launch_local[0], self.launch_local[1],
                        self.launch_local[2] + self.search_altitude)
        self.desired_yaw = self.yaw() or 0.0
        self.get_logger().info(
            f'starting SYSID {self.sysid}: search area {self.search_width:.1f} x '
            f'{self.search_length:.1f} m, lane spacing {self.lane_spacing:.1f} m, '
            f'search altitude {self.search_altitude:.1f} m')

    def after_takeoff(self):
        self.search_points = self._make_lawnmower()
        self.search_index = 0
        self._set_search_goal()
        self._enter('SEARCH')

    def _make_lawnmower(self):
        """Build evenly spaced parallel sweeps around launch, without rover data."""
        intervals = max(1, math.ceil(self.search_length / self.lane_spacing))
        rows = [(-self.search_length / 2 + i * self.search_length / intervals)
                for i in range(intervals + 1)]
        points = []
        for index, north in enumerate(rows):
            west, east = -self.search_width / 2, self.search_width / 2
            start, end = (west, east) if index % 2 == 0 else (east, west)
            points.extend(((self.search_origin[0] + start, self.search_origin[1] + north),
                           (self.search_origin[0] + end, self.search_origin[1] + north)))
        return points

    def _set_search_goal(self):
        east, north = self.search_points[self.search_index]
        altitude = self.search_origin[2] + self.search_altitude
        self.desired = (east, north, altitude)
        position = self.position()
        if position is not None and math.hypot(east - position[0], north - position[1]) > 0.1:
            self.desired_yaw = math.atan2(north - position[1], east - position[0])
        self.velocity_setpoint = (0.0, 0.0)

    def target_cb(self, msg):
        """Accept only a fresh, finite point in the active MAVROS local frame."""
        with self.lock:
            stamp = stamp_ns(msg.header)
            now_ros = self.get_clock().now().nanoseconds
            if self.marker_stamp and now_ros < self.marker_stamp:
                self.marker = None
                self.marker_stamp = 0
                self.marker_received = 0.0
            point = (msg.point.x, msg.point.y, msg.point.z)
            age = (now_ros - stamp) * 1e-9
            if (not msg.header.frame_id or stamp <= 0 or now_ros <= 0
                    or not 0 <= age <= self.max_target_age
                    or not all(math.isfinite(x) for x in point)
                    or (self.pose_frame and msg.header.frame_id != self.pose_frame)
                    or stamp <= self.marker_stamp):
                self.targets_rejected += 1
                return
            self.marker = point
            self.marker_frame = msg.header.frame_id
            self.marker_stamp = stamp
            self.marker_received = time.monotonic()

    def _fresh_marker(self, now):
        if (self.marker is None or now - self.marker_received > self.target_timeout
                or (self.pose_frame and self.marker_frame != self.pose_frame)):
            return None
        return self.marker

    def _bounded_velocity(self, target, speed_limit):
        position = self.position()
        if position is None:
            return 0.0, 0.0
        east, north = target[0] - position[0], target[1] - position[1]
        distance = math.hypot(east, north)
        if distance <= 0.2:
            return 0.0, 0.0
        speed = min(speed_limit, self.velocity_gain * distance)
        return speed * east / distance, speed * north / distance

    def mission_tick(self):
        now = time.monotonic()
        marker = self._fresh_marker(now)
        if self.state == 'SEARCH' and marker is not None:
            self._enter('TRACKING')
            self.get_logger().info(f'own marker {self.sysid} found; tracking')
        elif self.state == 'TRACKING' and marker is None:
            self.marker = None
            self.velocity_setpoint = (0.0, 0.0)
            self._set_search_goal()
            self._enter('SEARCH')
            self.get_logger().warn('own marker lost; resuming the search pattern')

        if self.state == 'SEARCH':
            goal = self.search_points[self.search_index]
            self.desired = (goal[0], goal[1], self.search_origin[2] + self.search_altitude)
            self.velocity_setpoint = self._bounded_velocity(self.desired, self.search_speed)
            vx, vy = self.velocity_setpoint
            if math.hypot(vx, vy) > 0.05:
                self.desired_yaw = math.atan2(vy, vx)
            # ArduPilot's guided position hold may settle a few decimetres
            # from the requested point; use a tolerance outside that deadband
            # so the lawnmower cannot stall at a corner.
            if self._reached(self.desired, 0.8, 0.25, 0.25):
                self.search_index = (self.search_index + 1) % len(self.search_points)
                self.arrived_since = None
                self._set_search_goal()
        elif self.state == 'TRACKING' and marker is not None:
            self.desired = (marker[0], marker[1], marker[2] + self.track_altitude)
            self.velocity_setpoint = self._bounded_velocity(self.desired, self.tracking_speed)
            vx, vy = self.velocity_setpoint
            if math.hypot(vx, vy) > 0.05:
                self.desired_yaw = math.atan2(vy, vx)

    def publish_mission_status(self):
        with self.lock:
            now = time.monotonic()
            age = now - self.marker_received if self.marker is not None else math.inf
            status = DiagnosticStatus(
                name=self.get_fully_qualified_name() + '/search_track',
                hardware_id=str(self.sysid),
                level=DiagnosticStatus.OK if self.state == 'TRACKING' else DiagnosticStatus.WARN,
                message=self.state,
                values=[KeyValue(key='sysid', value=str(self.sysid)),
                        KeyValue(key='search_index', value=str(self.search_index)),
                        KeyValue(key='search_waypoints', value=str(len(self.search_points))),
                        KeyValue(key='target_age_s', value=str(age)),
                        KeyValue(key='rejected_targets', value=str(self.targets_rejected)),
                        KeyValue(key='target_frame', value=self.marker_frame),
                        KeyValue(key='desired_enu_m', value=str(self.desired)),
                        KeyValue(key='velocity_enu_mps', value=str(self.velocity_setpoint))])
        message = DiagnosticArray(status=[status])
        message.header.stamp = self.get_clock().now().to_msg()
        self.mission_status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    spin(RoverMission())


if __name__ == '__main__':
    main()
