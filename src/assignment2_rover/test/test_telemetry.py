# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

import time

from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rclpy.node import Node
from rclpy.task import Future

from assignment2_rover.telemetry import STREAMS, TelemetrySetup


def test_requests_stay_in_namespace_and_restart_after_disconnect(ros):
    nodes = [ros.make(TelemetrySetup, namespace=f'/drone_{i}') for i in (1, 2)]
    probe = ros.make(Node, 'interval_probe')
    calls = [[], []]
    for index in range(2):
        def interval(request, response, index=index):
            calls[index].append((request.message_id, request.message_rate))
            response.success = True
            return response
        probe.create_service(MessageInterval, f'/drone_{index + 1}/mavros/set_message_interval', interval)
    ros.until(lambda: all(n.interval_client.service_is_ready() for n in nodes))
    for n in nodes:
        n.tick()
    assert calls == [[], []]  # Never request streams before FCU connection.

    def configured(node):
        node.tick()
        return node.index == len(STREAMS)

    nodes[0].on_state(State(connected=True))
    ros.until(lambda: configured(nodes[0]))
    assert calls == [list(STREAMS), []]
    nodes[1].on_state(State(connected=True))
    ros.until(lambda: configured(nodes[1]))
    assert calls[1] == list(STREAMS)

    nodes[0].on_state(State(connected=False))
    nodes[0].tick()
    assert nodes[0].index == 0
    nodes[0].on_state(State(connected=True))
    ros.until(lambda: configured(nodes[0]))
    assert calls == [list(STREAMS) * 2, list(STREAMS)]


def test_rejected_and_missing_ack_do_not_advance_or_block_forever(ros):
    node = ros.make(TelemetrySetup, namespace='/drone_1')
    node.on_state(State(connected=True))
    rejected = Future()
    rejected.set_result(MessageInterval.Response(success=False))
    node.request_future = rejected
    node.tick()
    assert node.index == 0 and node.request_future is None
    assert node.next_request > time.monotonic()

    missing = Future()
    node.request_future = missing
    node.next_request = 0.0
    node.tick()
    assert missing.cancelled() and node.request_future is None and node.index == 0
    assert node.next_request > time.monotonic()

    node.index = len(STREAMS)
    node.last_state = time.monotonic() - 6.0
    node.tick()
    assert node.index == 0  # A vanished FCU also resets the setup without a disconnect message.
