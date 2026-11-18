# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""ROS checks isolated from the live simulation and from Assignment 1 tests."""

import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture
def ros():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=78)
    executor = SingleThreadedExecutor(context=context)
    nodes = []

    def make(node_class, *args, **kwargs):
        node = node_class(*args, context=context, use_global_arguments=False, **kwargs)
        nodes.append(node)
        for timer in node.timers:
            timer.cancel()
        executor.add_node(node)
        return node

    def until(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'ROS discovery or delivery timed out'

    try:
        yield SimpleNamespace(make=make, until=until)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown(context=context)
