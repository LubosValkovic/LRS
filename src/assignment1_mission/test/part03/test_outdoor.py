# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Outdoor trajectory, sensor recovery, origin and buffer regressions."""

import math
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from nav_msgs.msg import Path

from assignment1_mission.part02.control import FlightNode
from assignment1_mission.part03.outdoor import OutdoorMission


@pytest.fixture
def outdoor(flight_factory, clock):
    node = flight_factory(OutdoorMission)
    node.fix = SimpleNamespace(latitude=48., longitude=17., altitude=150.,
                               status=SimpleNamespace(status=0))
    node.satellites, node.compass, node.rel_alt = 12, 90., 0.
    node.fix_at = node.satellites_at = node.compass_at = node.rel_alt_at = clock[0]
    node.home = None
    node.geo_origin = node.local_origin = None
    node.poi = (0., 0.)
    node.poi_offset = (0., 0.)
    node.launch_local = (1., 2., 3.)
    node.altitude, node.side, node.amplitude = 5., 5., 3.
    node.declination, node.omega = 0., .12
    node.square_speed = .3
    node.leg_started, node.eight_start = 80., 90.
    node.path_tick = 0
    node.actual_path, node.target_path = Path(), Path()
    node.actual_poses, node.target_poses = deque(maxlen=2), deque(maxlen=2)
    node.actual_pub, node.target_pub = Mock(), Mock()
    return node


@pytest.mark.parametrize('sensor', ['gps', 'compass', 'altitude'])
def test_nonfinite_sensors_are_not_ready(outdoor, sensor):
    if sensor == 'gps':
        outdoor.fix.latitude = float('nan')
    elif sensor == 'compass':
        outdoor.compass = float('nan')
    else:
        outdoor.rel_alt = float('inf')
    assert not outdoor.ready()


def test_sensor_outage_pauses_and_resumes_figure_eight(outdoor, clock):
    outdoor.state = 'EIGHT'
    outdoor.ready = lambda: False
    before = outdoor.desired
    assert not outdoor._publish()
    assert outdoor.desired == before
    clock[0] = 102.
    outdoor.last_pose = outdoor.last_state = clock[0]
    assert not outdoor._publish()
    outdoor.ready = lambda: True
    assert outdoor._publish()
    assert outdoor.eight_start == 92.
    phase = .12 * (clock[0] - outdoor.eight_start)
    assert outdoor.desired[:2] == pytest.approx((3 * math.sin(phase),
                                              3 * math.sin(phase) * math.cos(phase)))


def test_long_sensor_outage_aborts(outdoor, clock):
    outdoor.ready = lambda: False
    assert not outdoor._publish()
    clock[0] = 106.
    outdoor.last_pose = outdoor.last_state = clock[0]
    assert not outdoor._publish()
    assert outdoor.state == 'ABORT'
    outdoor.setpoint_pub.publish.assert_not_called()


def test_home_at_equator_and_nonzero_launch_altitude(outdoor):
    outdoor.home = SimpleNamespace(geo=SimpleNamespace(latitude=0., longitude=0.),
                                   position=SimpleNamespace(x=1., y=2.))
    outdoor.fix.latitude, outdoor.fix.longitude = 0., 0.
    outdoor.rel_alt = 2.
    outdoor.pose.position.z = 12.
    outdoor.start_mission()
    assert outdoor.geo_origin == (0., 0.)
    assert outdoor.local_origin[2] == 10.
    assert outdoor.launch_local[2] == 12.
    assert outdoor.desired[2] == 17.
    assert outdoor.takeoff_height == 7.


def test_path_history_is_bounded(outdoor):
    outdoor.ready = lambda: True
    outdoor.state = 'OUT_RETURN'
    for _ in range(20):
        outdoor._publish()
    assert len(outdoor.actual_path.poses) == len(outdoor.target_path.poses) == 2
    assert len(outdoor.actual_poses) == len(outdoor.target_poses) == 2


def test_square_leg_already_at_corner_does_not_divide_by_zero(outdoor, clock):
    outdoor.state = 'OUT_SQUARE'
    outdoor.leg_start = outdoor.leg_end = outdoor.desired
    outdoor.leg_started, outdoor.leg_duration = clock[0], 0.
    assert outdoor._publish()
    assert outdoor.desired == outdoor.leg_end


def test_square_closes_fourth_edge_before_returning_to_centre(outdoor, clock):
    outdoor.corners = [(-2.5, 2.5, 8.), (2.5, 2.5, 8.),
                       (2.5, -2.5, 8.), (-2.5, -2.5, 8.)]
    outdoor.status_at = math.inf
    outdoor.corner_index = 0
    outdoor._reached = lambda *args: True
    outdoor._start_square_leg(0)
    visited = []
    while outdoor.state == 'OUT_SQUARE':
        visited.append(outdoor.leg_end)
        outdoor.desired = outdoor.leg_end
        clock[0] = outdoor.leg_started + outdoor.leg_duration + .1
        outdoor.mission_tick()
        assert len(visited) <= 5
    assert visited == [*outdoor.corners, outdoor.corners[0]]
    assert outdoor.state == 'OUT_CENTER'


@pytest.mark.parametrize('name,value', [('omega', float('nan')), ('poi_east', float('inf')),
                                      ('figure_amplitude', -3.), ('square_side', 0.),
                                      ('flight_altitude', -5.), ('path_max_points', 1)])
def test_invalid_parameters_are_rejected_before_flight(monkeypatch, name, value):
    def fake_init(node, _name):
        values = {name: value}
        node.declare_parameter = lambda key, default: values.setdefault(key, default)
        node.get_parameter = lambda key: SimpleNamespace(value=values[key])
        node.get_logger = Mock(return_value=Mock())
    monkeypatch.setattr(FlightNode, '__init__', fake_init)
    with pytest.raises(ValueError):
        OutdoorMission()
