# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Flight state and command safety regressions without a connected vehicle."""

from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


def test_stale_velocity_cannot_confirm_stable_arrival(flight_factory, clock):
    node = flight_factory()
    node.last_velocity = 90.0
    assert not node._reached(node.desired, .18, .2, .8)
    node.last_velocity = clock[0]
    assert not node._reached(node.desired, .18, .2, .8)
    clock[0] += .9
    assert node._reached(node.desired, .18, .2, .8)


@pytest.mark.parametrize('field', ['position', 'velocity'])
def test_nan_sensor_cannot_confirm_arrival(flight_factory, clock, field):
    node = flight_factory()
    if field == 'position':
        node.pose.position.x = float('nan')
    else:
        node.velocity.x = float('nan')
    assert not node._reached(node.desired, .18, .2)


def test_landtakeoff_waits_two_seconds_after_disarm(flight_factory, clock):
    node = flight_factory()
    node._enter('LAND')
    node.entered = 70.
    node.after_land = 'RESTART'
    node.resume_target = (1., 2., 3.)
    node.vehicle.mode = 'LAND'
    node.vehicle.armed = False
    node._tick()
    assert node.state == 'LAND'
    for t, expected in ((101., 'LAND'), (102., 'SET_MODE')):
        clock[0] = t
        node.last_state = node.last_pose = t
        node._tick()
        assert node.state == expected


def test_pilot_mode_change_during_arm_stops_mission(flight_factory, clock):
    node = flight_factory()
    node.state = 'ARM'
    node.vehicle.mode = 'LOITER'
    node.vehicle.armed = False
    node._arm = Mock()
    node._tick()
    assert node.state == 'PASSIVE'
    node._arm.assert_not_called()


def test_gps_loss_before_arm_prevents_arming(flight_factory, clock):
    node = flight_factory()
    node.state = 'ARM'
    node.vehicle.armed = False
    node.ready = lambda: False
    node._arm = Mock()
    node._tick()
    node._arm.assert_not_called()
    assert node.navigation_bad_since == clock[0]


def test_service_timeout_allows_abort_and_ignores_late_response(flight_factory, clock):
    node = flight_factory()
    future = Future()
    future.set_running_or_notify_cancel()
    client = Mock()
    client.call_async.return_value = future
    assert node._call(client, None, lambda r: r.success, 'takeoff')
    clock[0] = 106.
    node._set_mode = Mock(return_value=True)
    node._tick()
    assert node.state == 'ABORT'
    assert node.pending is None
    node._set_mode.assert_called_once_with('RTL')
    future.set_result(SimpleNamespace(success=False))
    assert node.state == 'ABORT'
    assert node.pending is None


def test_late_service_response_does_not_override_pilot_takeover(flight_factory, clock):
    node = flight_factory()
    future = Future()
    client = Mock()
    client.call_async.return_value = future
    node._call(client, None, lambda r: r.success, 'arm')
    node._enter('PASSIVE')
    future.set_result(SimpleNamespace(success=False))
    clock[0] += 10.
    node._tick()
    assert node.state == 'PASSIVE'
    assert node.pending is None


@pytest.mark.parametrize('condition', ['disarmed', 'stale_pose', 'stale_state', 'nan_target'])
def test_invalid_state_never_publishes_setpoints(flight_factory, clock, condition):
    node = flight_factory()
    if condition == 'disarmed':
        node.vehicle.armed = False
    elif condition == 'stale_pose':
        node.last_pose = 90.
    elif condition == 'stale_state':
        node.last_state = 90.
    else:
        node.desired = (float('nan'), 2., 3.)
    assert not node._publish()
    node.setpoint_pub.publish.assert_not_called()


def test_valid_setpoint_preserves_position_and_yaw(flight_factory, clock):
    node = flight_factory()
    assert node._publish()
    target = node.setpoint_pub.publish.call_args.args[0]
    assert (target.position.x, target.position.y, target.position.z) == node.desired
    assert target.yaw == node.desired_yaw
