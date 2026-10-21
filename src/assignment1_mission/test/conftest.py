# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Controller harness with real ROS messages and no live ROS connections."""

import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest


@pytest.fixture
def clock(monkeypatch):
    now = [100.0]
    monkeypatch.setattr(time, 'monotonic', lambda: now[0])
    return now


@pytest.fixture
def flight_factory():
    def make(node_class=None):
        from builtin_interfaces.msg import Time
        from assignment1_mission.part02.control import FlightNode

        node = (node_class or FlightNode).__new__(node_class or FlightNode)
        now = time.monotonic()
        values = dict(
            lock=threading.RLock(), state='NAVIGATE', entered=now,
            desired=(1.0, 2.0, 3.0), desired_yaw=0.0, arrived_since=None,
            navigation_bad_since=None, pending=None, pending_future=None,
            pending_since=0.0, mode_requested=0.0, arm_requested=0.0,
            takeoff_requested=False, takeoff_height=3.0, land_requested=False,
            after_land='DONE', landed_since=None, resume_target=None,
            last_state=now, last_pose=now, last_velocity=now,
            vehicle=SimpleNamespace(connected=True, armed=True, mode='GUIDED'),
            pose=SimpleNamespace(position=SimpleNamespace(x=1., y=2., z=3.),
                                 orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)),
            velocity=SimpleNamespace(x=0., y=0., z=0.),
        )
        for name, value in values.items():
            setattr(node, name, value)
        node.get_logger = Mock(return_value=Mock())
        node.get_clock = Mock()
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        node.setpoint_pub = Mock()
        node.mode_client = Mock()
        node.arm_client = Mock()
        node.takeoff_client = Mock()
        if node_class is None:
            node.mission_tick = Mock()
        return node
    return make
