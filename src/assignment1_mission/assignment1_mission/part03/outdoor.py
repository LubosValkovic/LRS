# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""GPS/compass square and timer driven continuous figure-eight mission."""

import math
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64, UInt32

from ..part02.control import FlightNode, spin
from ..part02.mission import wrap, yaw_from_quaternion
from .geo import enu_to_geodetic, geodetic_to_enu


class OutdoorMission(FlightNode):
    def __init__(self):
        super().__init__('assignment1_outdoor')
        self.poi_offset = (float(self._parameter('poi_east', 0.0)),
                           float(self._parameter('poi_north', 0.0)))
        self.omega = float(self._parameter('omega', 0.12))
        self.square_speed = float(self._parameter('square_speed', 0.30))
        self.side = float(self._parameter('square_side', 5.0))
        self.amplitude = float(self._parameter('figure_amplitude', 3.0))
        self.altitude = float(self._parameter('flight_altitude', 5.0))
        self.declination = math.radians(float(self._parameter('magnetic_declination_deg', 0.0)))
        max_radius = float(self._parameter('max_horizontal_radius', 10.0))
        max_altitude = float(self._parameter('max_altitude', 6.0))
        self.path_max_points = self._parameter('path_max_points', 2000)
        if (not all(math.isfinite(v) for v in (*self.poi_offset, self.omega, self.square_speed,
                                             self.side, self.amplitude, self.altitude,
                                             self.declination, max_radius, max_altitude))
                or min(self.side, self.amplitude, max_radius, max_altitude) <= 0
                or self.altitude < 0.5):
            raise ValueError('mission parameters must be finite; dimensions positive; altitude >= 0.5 m')
        if not isinstance(self.path_max_points, int) or self.path_max_points < 2:
            raise ValueError('path_max_points must be an integer >= 2')
        if self.omega <= 0 or self.omega * self.amplitude * math.sqrt(2) > 1.5:
            raise ValueError('omega must be positive and figure-8 maximum speed <= 1.5 m/s')
        if not 0 < self.square_speed <= 1.0:
            raise ValueError('square_speed must be in (0, 1] m/s')
        radius = math.hypot(*self.poi_offset) + max(self.side / math.sqrt(2), self.amplitude)
        if radius > max_radius or self.altitude > max_altitude:
            raise ValueError('mission exceeds configured geofence')
        self.get_logger().info(f'geofence: <= {radius:.2f} m from home, altitude {self.altitude:.1f} m; '
                               f'figure-8 max speed {self.omega * self.amplitude * math.sqrt(2):.2f} m/s')
        self.fix = self.satellites = self.compass = self.rel_alt = self.home = None
        self.fix_at = self.satellites_at = self.compass_at = self.rel_alt_at = 0.0
        self.geo_origin = self.local_origin = self.poi = None
        self.corners = []
        self.corner_index = 0
        self.leg_start = self.leg_end = None
        self.leg_started = self.leg_duration = self.eight_start = None
        self.status_at = 0.0
        self.path_tick = 0
        self.actual_path = Path()
        self.target_path = Path()
        self.actual_path.header.frame_id = self.target_path.header.frame_id = 'map'
        self.actual_poses = deque(maxlen=self.path_max_points)
        self.target_poses = deque(maxlen=self.path_max_points)
        self.actual_pub = self.create_publisher(Path, '~/actual_path', 10)
        self.target_pub = self.create_publisher(Path, '~/target_path', 10)
        self._subscribe_sensor(NavSatFix, '/mavros/global_position/global', 'fix', 'fix_at')
        self._subscribe_sensor(UInt32, '/mavros/global_position/raw/satellites', 'satellites', 'satellites_at', 'data')
        self.create_subscription(HomePosition, '/mavros/home_position/home', self._home_cb,
                                 qos_profile_sensor_data, callback_group=self.sensor_group)
        self._subscribe_sensor(Float64, '/mavros/global_position/compass_hdg', 'compass', 'compass_at', 'data')
        self._subscribe_sensor(Float64, '/mavros/global_position/rel_alt', 'rel_alt', 'rel_alt_at', 'data')

    def _home_cb(self, msg):
        with self.lock:
            if (self.geo_origin is None
                    and all(math.isfinite(v) for v in (msg.geo.latitude, msg.geo.longitude,
                                                       msg.position.x, msg.position.y))
                    and -90 < msg.geo.latitude < 90 and -180 <= msg.geo.longitude <= 180):
                self.home = msg

    def ready(self):
        now = time.monotonic()
        return (self.fix is not None and self.fix.status.status >= NavSatStatus.STATUS_FIX
                and all(math.isfinite(v) for v in (self.fix.latitude, self.fix.longitude, self.fix.altitude))
                and -90 < self.fix.latitude < 90 and -180 <= self.fix.longitude <= 180
                and self.satellites is not None and 8 <= self.satellites < 255
                and self.compass is not None and self.rel_alt is not None
                and math.isfinite(self.compass) and math.isfinite(self.rel_alt)
                and max(now - self.fix_at, now - self.satellites_at, now - self.compass_at,
                        now - self.rel_alt_at) < 3.0)

    def position(self):
        if self.geo_origin is None or self.fix is None or self.local_origin is None:
            return super().position()
        east, north = geodetic_to_enu(self.fix.latitude, self.fix.longitude, *self.geo_origin)
        return (self.local_origin[0] + east, self.local_origin[1] + north,
                self.local_origin[2] + self.rel_alt)

    def yaw(self):
        if self.compass is None:
            return super().yaw()
        return wrap(math.pi / 2 - math.radians(self.compass) - self.declination)

    def start_mission(self):
        pose = super().position()
        if self.home is not None:
            self.geo_origin = (self.home.geo.latitude, self.home.geo.longitude)
            self.local_origin = (self.home.position.x, self.home.position.y, pose[2] - self.rel_alt)
        else:
            self.geo_origin = (self.fix.latitude, self.fix.longitude)
            self.local_origin = (pose[0], pose[1], pose[2] - self.rel_alt)
        self.launch_local = self.position()
        self.poi = (self.local_origin[0] + self.poi_offset[0],
                    self.local_origin[1] + self.poi_offset[1])
        # Define the corners geodetically and convert back to local ENU.
        offsets = ((-1, 1), (1, 1), (1, -1), (-1, -1))
        self.corners = []
        for sx, sy in offsets:
            lat, lon = enu_to_geodetic(self.poi_offset[0] + sx * self.side / 2,
                                       self.poi_offset[1] + sy * self.side / 2,
                                       *self.geo_origin)
            east, north = geodetic_to_enu(lat, lon, *self.geo_origin)
            self.corners.append((self.local_origin[0] + east,
                                 self.local_origin[1] + north,
                                 self.launch_local[2] + self.altitude))
        self.desired = (*self.launch_local[:2], self.launch_local[2] + self.altitude)
        self.desired_yaw = self.yaw()
        self.takeoff_height = self.rel_alt + self.altitude
        self.get_logger().info(f'home {self.geo_origin}; launch ENU {self.launch_local}; '
                               f'POI offset {self.poi_offset}')

    def after_takeoff(self):
        self.corner_index = 0
        self.desired = (*self.launch_local[:2], self.launch_local[2] + self.altitude)
        self.desired_yaw = math.atan2(self.poi[1] - self.corners[0][1],
                                      self.poi[0] - self.corners[0][0])
        self._enter('SQUARE_ALIGN')

    def navigation_resumed(self, duration):
        if self.leg_started is not None:
            self.leg_started += duration
        if self.eight_start is not None:
            self.eight_start += duration

    def _start_square_leg(self, index):
        self.leg_start = self.desired if index == 0 else self.corners[index - 1]
        self.leg_end = self.corners[index]
        self.leg_started = time.monotonic()
        self.leg_duration = math.dist(self.leg_start[:2], self.leg_end[:2]) / self.square_speed
        self.arrived_since = None
        self._enter('OUT_SQUARE')

    def _update_poi_yaw(self):
        pos = self.position()
        dx, dy = self.poi[0] - pos[0], self.poi[1] - pos[1]
        if math.hypot(dx, dy) > 0.2:
            self.desired_yaw = math.atan2(dy, dx)

    def mission_tick(self):
        now = time.monotonic()
        if now - self.status_at > 1.0:
            self.status_at = now
            p = self.position()
            local = (p[0] - self.local_origin[0], p[1] - self.local_origin[1],
                     p[2] - self.local_origin[2])
            self.get_logger().info(f'GPS {self.fix.latitude:.8f},{self.fix.longitude:.8f},'
                                   f'{self.fix.altitude:.2f} GNSS altitude; ENU {local}; '
                                   f'compass {self.compass:.1f} deg, yaw ENU '
                                   f'{math.degrees(self.yaw()):.1f} deg, pose yaw '
                                   f'{math.degrees(yaw_from_quaternion(self.pose.orientation)):.1f} deg')
        if self.state == 'SQUARE_ALIGN':
            if abs(wrap(self.desired_yaw - self.yaw())) < math.radians(7):
                if self._reached(self.desired, 0.3, 0.3, 0.7):
                    self._start_square_leg(0)
            else:
                self.arrived_since = None
        elif self.state == 'OUT_SQUARE':
            self._update_poi_yaw()
            if now - self.leg_started >= self.leg_duration and self._reached(self.leg_end, 0.30, 0.35, 0.4):
                self.corner_index += 1
                if self.corner_index == len(self.corners):
                    self._start_square_leg(0)  # Close W4 -> W1 before entering the centre.
                elif self.corner_index > len(self.corners):
                    self.desired = (*self.poi, self.launch_local[2] + self.altitude)
                    self._enter('OUT_CENTER')
                else:
                    self._start_square_leg(self.corner_index)
        elif self.state == 'OUT_CENTER':
            p = self.position()
            self.desired_yaw = math.atan2(self.desired[1] - p[1], self.desired[0] - p[0])
            if self._reached(self.desired, 0.30, 0.25, 1.0):
                self.desired_yaw = math.pi / 4
                self._enter('EIGHT_ALIGN')
        elif self.state == 'EIGHT_ALIGN':
            if abs(wrap(self.desired_yaw - self.yaw())) < math.radians(10) and self._reached(self.desired, 0.3, 0.3, 0.6):
                self.eight_start = now
                self._enter('EIGHT')
            elif abs(wrap(self.desired_yaw - self.yaw())) >= math.radians(10):
                self.arrived_since = None
        elif self.state == 'EIGHT':
            if self.omega * (now - self.eight_start) >= 2 * math.pi:
                self.desired = (*self.launch_local[:2], self.launch_local[2] + self.altitude)
                self._enter('OUT_RETURN')
        elif self.state == 'OUT_RETURN':
            p = self.position()
            dx, dy = self.desired[0] - p[0], self.desired[1] - p[1]
            if math.hypot(dx, dy) > 0.2:
                self.desired_yaw = math.atan2(dy, dx)
            if self._reached(self.desired, 0.35, 0.25, 1.0):
                self.after_land = 'DONE'
                self.land_requested = False
                self._enter('LAND')

    def _publish(self):
        if not self._can_publish():
            return False
        if self.state == 'EIGHT':
            phase = min(2 * math.pi, self.omega * (time.monotonic() - self.eight_start))
            x = self.amplitude * math.sin(phase)
            y = self.amplitude * math.sin(phase) * math.cos(phase)
            self.desired = (self.poi[0] + x, self.poi[1] + y,
                            self.launch_local[2] + self.altitude)
            self.desired_yaw = math.atan2(math.cos(2 * phase), math.cos(phase))
        elif self.state == 'OUT_SQUARE':
            fraction = (min(1.0, (time.monotonic() - self.leg_started) / self.leg_duration)
                        if self.leg_duration > 0 else 1.0)
            self.desired = tuple(self.leg_start[i] + fraction * (self.leg_end[i] - self.leg_start[i])
                                 for i in range(3))
            self._update_poi_yaw()
        if not super()._publish():
            return False
        if self.state in ('OUT_SQUARE', 'OUT_CENTER', 'EIGHT', 'OUT_RETURN') and self.desired:
            self.path_tick += 1
            if self.path_tick % 5 == 0:
                stamp = self.get_clock().now().to_msg()
                for path, poses, point in ((self.actual_path, self.actual_poses, self.position()),
                                           (self.target_path, self.target_poses, self.desired)):
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = stamp
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
                    pose.pose.orientation.w = 1.0
                    poses.append(pose)
                    path.poses = list(poses)
                    path.header.stamp = stamp
                self.actual_pub.publish(self.actual_path)
                self.target_pub.publish(self.target_path)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = OutdoorMission()
    spin(node)


if __name__ == '__main__':
    main()
