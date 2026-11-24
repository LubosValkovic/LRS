# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Search, tracking and bounded MAVROS commands without arming a vehicle."""

import math
import time

from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from mavros_msgs.msg import PositionTarget, State
from rclpy.node import Node
from rclpy.parameter import Parameter

from assignment2_rover.rover_mission import RoverMission


def make_mission(ros, **overrides):
    params = dict(sysid=1, search_width=20., search_length=10., lane_spacing=3.,
                  search_altitude=8., track_altitude=2., search_speed=.8,
                  tracking_speed=.6, target_timeout=1., max_target_age=.75)
    params.update(overrides)
    return ros.make(RoverMission, namespace='/drone_1',
                    parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])


def target(node, east=1., north=2., up=.3165, frame='map'):
    msg = PointStamped()
    msg.header.frame_id = frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.point = Point(x=east, y=north, z=up)
    return msg


def test_lawnmower_covers_configured_rectangle_about_launch_only(ros):
    node = make_mission(ros)
    node.search_origin = (100., -20., 3.)
    points = node._make_lawnmower()
    assert len(points) == 10
    assert min(p[0] for p in points) == 90.
    assert max(p[0] for p in points) == 110.
    ys = sorted({p[1] for p in points})
    assert ys == [-25., -22.5, -20., -17.5, -15.]
    assert max(b - a for a, b in zip(ys, ys[1:])) <= node.lane_spacing


def test_target_requires_fresh_finite_point_in_local_pose_frame(ros):
    node = make_mission(ros)
    node.pose_frame = 'map'
    node.target_cb(target(node))
    assert node.marker == (1., 2., .3165)
    stamp = node.marker_stamp

    node.target_cb(target(node, frame='odom'))
    assert node.marker_stamp == stamp and node.targets_rejected == 1
    invalid = target(node, east=math.nan)
    node.target_cb(invalid)
    assert node.marker_stamp == stamp and node.targets_rejected == 2
    stale = target(node)
    stale.header.stamp.sec -= 2
    node.target_cb(stale)
    assert node.marker_stamp == stamp and node.targets_rejected == 3


def test_mission_switches_search_track_and_lost_target_resumes_search(ros, monkeypatch):
    node = make_mission(ros)
    node.search_origin = (0., 0., 0.)
    node.search_points = [(4., 0.), (4., 5.)]
    node.search_index = 0
    node.pose_frame = 'map'
    node.pose = Pose(position=Point(x=0., y=0., z=8.), orientation=Quaternion(w=1.))
    node.state = 'SEARCH'
    node.target_cb(target(node, east=1., north=2., up=.3165))

    node.mission_tick()
    assert node.state == 'TRACKING'
    assert node.desired == (1., 2., 2.3165)
    assert math.hypot(*node.velocity_setpoint) <= node.tracking_speed

    monkeypatch.setattr(time, 'monotonic', lambda: node.marker_received + node.target_timeout + .01)
    node.mission_tick()
    assert node.state == 'SEARCH'
    assert node.marker is None
    assert node.desired == (4., 0., 8.)
    assert math.hypot(*node.velocity_setpoint) <= node.search_speed


def test_search_advances_when_guided_position_hold_settles_within_deadband(ros):
    node = make_mission(ros)
    node.search_origin = (0., 0., 0.)
    node.search_points = [(4., 0.), (4., 5.)]
    node.search_index = 0
    node.state = 'SEARCH'
    node.pose = Pose(position=Point(x=3.9, y=.7, z=8.), orientation=Quaternion(w=1.))
    node.vehicle = State(connected=True, armed=True, mode='GUIDED')
    node.last_state = node.last_pose = time.monotonic()
    node.velocity = Point()
    node.last_velocity = time.monotonic()
    node.arrived_since = time.monotonic() - 1.0

    node.mission_tick()

    assert node.search_index == 1
    assert node.desired == (4., 5., 8.)


def test_velocity_target_caps_horizontal_motion_and_holds_target_altitude(ros):
    node = make_mission(ros)
    probe = ros.make(Node, 'velocity_probe')
    received = []
    probe.create_subscription(PositionTarget, '/drone_1/mavros/setpoint_raw/local',
                              received.append, 10)
    ros.until(lambda: node.setpoint_pub.get_subscription_count() > 0)
    node.vehicle = State(connected=True, armed=True, mode='GUIDED')
    node.pose = Pose(position=Point(z=8.), orientation=Quaternion(w=1.))
    node.last_state = node.last_pose = time.monotonic()
    node.state = 'TRACKING'
    node.desired = (1., 2., 10.)
    node.velocity_setpoint = (.6, -.2)

    assert node._publish()
    ros.until(lambda: len(received) == 1)
    message = received[0]
    assert message.type_mask & PositionTarget.IGNORE_PX
    assert message.type_mask & PositionTarget.IGNORE_PY
    assert not message.type_mask & PositionTarget.IGNORE_VX
    assert not message.type_mask & PositionTarget.IGNORE_VY
    assert message.type_mask & PositionTarget.IGNORE_PZ
    assert (message.velocity.x, message.velocity.y) == (.6, -.2)
    assert math.isclose(message.velocity.z, .5, abs_tol=1e-6)


def test_abort_stops_velocity_setpoints_and_does_not_enter_land(ros):
    node = make_mission(ros)
    node.state = 'TRACKING'
    node.desired = (1., 2., 3.)
    node.velocity_setpoint = (.2, 0.)
    result = node._abort_cb(None, type('Response', (), {})())
    assert result.success and node.state == 'ABORT'
    assert node.desired is None and node.velocity_setpoint is None
    assert node.state != 'LAND'
