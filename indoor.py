# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Indoor mission: CSV goals, planned routes and waypoint tasks."""

import math
import time

import rclpy

from .control import FlightNode, spin
from .mission import load_mission, wrap
from ..part01.planning import VoxelMap, plan_route


class IndoorMission(FlightNode):
    def __init__(self):
        super().__init__('assignment1_indoor')
        self.declare_parameter('mission_file', '')
        self.declare_parameter('map_file', '')
        self.declare_parameter('resolution', 0.25)
        self.declare_parameter('safety_radius', 0.45)
        self.declare_parameter('map_launch', [13.0, 7.0, 0.0])
        self.declare_parameter('map_to_local_yaw_deg', 90.0)
        mission_file = self.get_parameter('mission_file').value
        map_file = self.get_parameter('map_file').value
        if not mission_file or not map_file:
            raise ValueError('mission_file and map_file ROS parameters are required')
        self.map_launch = tuple(float(x) for x in self.get_parameter('map_launch').value)
        self.frame_yaw = math.radians(float(self.get_parameter('map_to_local_yaw_deg').value))
        self.frame_translation = (0.0, 0.0, 0.0)
        self.mission = load_mission(mission_file)
        grid = VoxelMap(map_file, self.get_parameter('resolution').value,
                        self.get_parameter('safety_radius').value)
        self.get_logger().info(f'map bounds {grid.minimum.tolist()} .. {grid.maximum.tolist()}, '
                               f'{int(grid.occupied.sum())} occupied voxels, '
                               f'{int(grid.blocked.sum())} inflated voxels')
        self.routes = []
        self.pre_route = None
        pre_start = (self.map_launch[0], self.map_launch[1], self.mission[0].z)
        if math.dist(pre_start, self.mission[0].point) > 0.2:
            self.pre_route = plan_route(grid, pre_start, self.mission[0].point).path
            self.get_logger().info(f'launch to first mission row: {len(self.pre_route)} path points')
        for a, b in zip(self.mission, self.mission[1:]):
            result = plan_route(grid, a.point, b.point)
            self.routes.append(result.path)
            self.get_logger().info(f'route {len(self.routes)}: {len(result.raw)} -> '
                                   f'{len(result.path)} points, {result.seconds:.3f} s')
        self.segment = 0
        self.point_index = 1
        self.pre_index = 1
        self.turn_position = None

    def local(self, map_point):
        x, y, z = map_point
        c, s = math.cos(self.frame_yaw), math.sin(self.frame_yaw)
        return (c * x - s * y + self.frame_translation[0],
                s * x + c * y + self.frame_translation[1],
                z + self.frame_translation[2])

    def start_mission(self):
        super().start_mission()
        lx, ly, lz = self.launch_local
        x, y, z = self.map_launch
        c, s = math.cos(self.frame_yaw), math.sin(self.frame_yaw)
        self.frame_translation = (lx - (c * x - s * y), ly - (s * x + c * y), lz - z)
        first = self.local(self.mission[0].point)
        self.desired = (self.launch_local[0], self.launch_local[1], first[2])
        self.desired_yaw = self.yaw() or 0.0
        self.takeoff_height = first[2] - self.launch_local[2]
        if self.takeoff_height < 0.5:
            self._fail('first takeoff altitude is too low')

    def after_takeoff(self):
        if self.pre_route is None:
            self._begin_segment(turn=True)
        else:
            self.pre_index = 1
            self._aim_pre()
            self._enter('PRE_NAV')

    def _aim_pre(self):
        a, b = self.pre_route[self.pre_index - 1:self.pre_index + 1]
        self.desired = self.local(b)
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-4:
            self.desired_yaw = math.atan2(b[1] - a[1], b[0] - a[0]) + self.frame_yaw

    def _begin_segment(self, turn):
        if self.segment >= len(self.routes):
            self._enter('DONE')
            return
        self.point_index = 1
        self._aim_at_route_point()
        if turn:
            self.turn_position = self.position()
            self.desired = self.turn_position
            self._enter('TURN')
        else:
            self._enter('NAVIGATE')

    def _aim_at_route_point(self):
        route = self.routes[self.segment]
        a = route[self.point_index - 1]
        b = route[self.point_index]
        self.desired = self.local(b)
        if math.hypot(b[0] - a[0], b[1] - a[1]) > 1e-4:
            self.desired_yaw = math.atan2(b[1] - a[1], b[0] - a[0]) + self.frame_yaw

    def _advance(self, turn=False):
        self.segment += 1
        self._begin_segment(turn)

    def mission_tick(self):
        if self.state == 'PRE_NAV':
            final = self.pre_index == len(self.pre_route) - 1
            radius = 0.6 if final else 0.35
            if self._reached(self.desired, radius, None, 0.15 if final else 0.0):
                if final:
                    self._begin_segment(turn=False)
                else:
                    self.pre_index += 1
                    self.arrived_since = None
                    self._aim_pre()
            return
        if self.state == 'TURN':
            self.desired = self.turn_position
            if abs(wrap(self.desired_yaw - self.yaw())) < math.radians(10):
                if self._reached(self.turn_position, 0.3, 0.3, 0.4):
                    self._aim_at_route_point()
                    self._enter('NAVIGATE')
            else:
                self.arrived_since = None
            return
        if self.state == 'NAVIGATE':
            route = self.routes[self.segment]
            final = self.point_index == len(route) - 1
            waypoint = self.mission[self.segment + 1]
            if final and waypoint.precision == 'hard':
                reached = self._reached(self.desired, 0.18, 0.20, 0.8)
            elif final:
                reached = self._reached(self.desired, 0.60, None, 0.15)
            else:
                reached = self._reached(self.desired, 0.35)
            if not reached:
                return
            if not final:
                self.point_index += 1
                self.arrived_since = None
                self._aim_at_route_point()
            else:
                self._enter('TASK')
            return
        if self.state != 'TASK':
            return
        waypoint = self.mission[self.segment + 1]
        if waypoint.task == '-':
            self._advance(turn=waypoint.precision == 'hard')
        elif waypoint.task.startswith('yaw'):
            self.desired = self.local(waypoint.point)
            self.desired_yaw = math.radians(float(waypoint.task[3:]))
            if abs(wrap(self.desired_yaw - self.yaw())) < math.radians(7):
                if self._reached(self.desired, 0.25, 0.2, 1.0):
                    self._advance(turn=True)
            else:
                self.arrived_since = None
        elif waypoint.task in ('land', 'landtakeoff'):
            self.after_land = 'DONE' if waypoint.task == 'land' else 'RESTART'
            self.land_requested = False
            self._enter('LAND')
            if waypoint.task == 'landtakeoff':
                self.segment += 1
                target = self.local(waypoint.point)
                self.resume_target = target
                self.takeoff_height = target[2] - self.launch_local[2]


def main(args=None):
    rclpy.init(args=args)
    node = IndoorMission()
    spin(node)


if __name__ == '__main__':
    main()
