# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Real ROS routing checks, in a separate domain with no autopilot or flight."""

import time
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import SetMode
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger

from assignment1_mission.part02.control import FlightNode
from assignment1_mission.part03.outdoor import OutdoorMission


@pytest.fixture
def ros():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=77)
    executor = SingleThreadedExecutor(context=context)
    nodes = []

    def make(node_class, *args, **kwargs):
        node = node_class(*args, context=context, use_global_arguments=False, **kwargs)
        nodes.append(node)
        # Exercise routing explicitly without running autonomous mission timers.
        for timer in node.timers:
            timer.cancel()
        executor.add_node(node)
        return node

    def until(predicate):
        deadline = time.monotonic() + 5.0
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert predicate(), 'ROS discovery or message delivery timed out'

    try:
        yield SimpleNamespace(make=make, until=until)
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        rclpy.shutdown(context=context)


def flight(ros, node_class=FlightNode, **kwargs):
    args = ('assignment1_indoor',) if node_class is FlightNode else ()
    return ros.make(node_class, *args, **kwargs)


def vehicle_endpoints(node):
    return ({p.topic_name for p in node.publishers if '/mavros/' in p.topic_name}
            | {s.topic_name for s in node.subscriptions if '/mavros/' in s.topic_name}
            | {c.service_name for c in node.clients}
            | {s.service_name for s in node.services if s.service_name.endswith('/abort')})


def assert_vehicle_names(node, prefix):
    subscriptions = {'state', 'local_position/pose', 'local_position/velocity_local'}
    if isinstance(node, OutdoorMission):
        subscriptions |= {'global_position/global', 'global_position/raw/satellites',
                          'home_position/home', 'global_position/compass_hdg',
                          'global_position/rel_alt'}
        assert node.actual_pub.topic_name == node.get_fully_qualified_name() + '/actual_path'
        assert node.target_pub.topic_name == node.get_fully_qualified_name() + '/target_path'
    expected = {f'{prefix}/mavros/{suffix}' for suffix in subscriptions}
    expected |= {f'{prefix}/mavros/{suffix}' for suffix in
                 ['setpoint_raw/local', 'set_mode', 'cmd/arming', 'cmd/takeoff']}
    expected.add(node.get_fully_qualified_name() + '/abort')
    assert vehicle_endpoints(node) == expected


@pytest.mark.parametrize('node_class', [FlightNode, OutdoorMission])
def test_root_namespace_keeps_assignment1_interfaces(ros, node_class):
    node = flight(ros, node_class)
    assert node.get_namespace() == '/'
    assert node.get_name() == ('assignment1_indoor' if node_class is FlightNode
                               else 'assignment1_outdoor')
    assert_vehicle_names(node, '')


@pytest.mark.parametrize('node_class', [FlightNode, OutdoorMission])
def test_two_namespaces_have_disjoint_vehicle_interfaces(ros, node_class):
    first = flight(ros, node_class, namespace='/drone_1')
    second = flight(ros, node_class, cli_args=['--ros-args', '-r', '__ns:=/drone_2'])
    assert_vehicle_names(first, '/drone_1')
    assert_vehicle_names(second, '/drone_2')
    assert vehicle_endpoints(first).isdisjoint(vehicle_endpoints(second))


def test_telemetry_and_setpoints_reach_only_their_own_vehicle(ros):
    drones = [flight(ros, namespace=f'/drone_{i}') for i in (1, 2)]
    probe = ros.make(Node, 'telemetry_probe')
    publishers, received = [], [[], []]
    for i in (1, 2):
        prefix = f'/drone_{i}/mavros'
        publishers.append([
            probe.create_publisher(State, prefix + '/state', 10),
            probe.create_publisher(PoseStamped, prefix + '/local_position/pose', qos_profile_sensor_data),
            probe.create_publisher(TwistStamped, prefix + '/local_position/velocity_local', qos_profile_sensor_data),
        ])
        probe.create_subscription(PositionTarget, prefix + '/setpoint_raw/local',
                                  received[i - 1].append, 10)
    ros.until(lambda: all(p.get_subscription_count() for group in publishers for p in group)
              and all(d.setpoint_pub.get_subscription_count() for d in drones))

    for index, (node, group) in enumerate(zip(drones, publishers)):
        pose = PoseStamped()
        pose.pose.position.x = float(index + 1)
        pose.pose.position.z = 2.0
        pose.pose.orientation.w = 1.0
        velocity = TwistStamped()
        velocity.twist.linear.x = float(index)
        group[0].publish(State(connected=True, armed=True, mode='GUIDED'))
        group[1].publish(pose)
        group[2].publish(velocity)
        ros.until(lambda: node.vehicle is not None and node.pose is not None
                  and node.velocity is not None)
        assert node.position() == (float(index + 1), 0.0, 2.0)
        assert node.velocity.x == float(index)
        if index == 0:
            assert drones[1].vehicle is drones[1].pose is drones[1].velocity is None

    for index, node in enumerate(drones):
        node.state = 'NAVIGATE'
        node.desired = (float(index + 3), 0.0, 2.0)
        assert node._publish()
        ros.until(lambda: len(received[index]) == 1)
        assert received[index][0].position.x == float(index + 3)
        if index == 0:
            assert received[1] == []
    assert [node.position()[0] for node in drones] == [1.0, 2.0]
    assert [len(messages) for messages in received] == [1, 1]


def test_abort_and_rtl_service_are_isolated_per_vehicle(ros):
    drones = [flight(ros, namespace=f'/drone_{i}') for i in (1, 2)]
    probe = ros.make(Node, 'service_probe')
    modes, abort_clients = [[], []], []
    for index, node in enumerate(drones):
        def set_mode(request, response, index=index):
            modes[index].append(request.custom_mode)
            response.mode_sent = True
            return response

        probe.create_service(SetMode, f'/drone_{index + 1}/mavros/set_mode', set_mode)
        abort_clients.append(probe.create_client(Trigger, node.get_fully_qualified_name() + '/abort'))
        node.vehicle = State(connected=True, armed=True, mode='GUIDED')
        node.state = 'NAVIGATE'
        node.desired = (1.0, 2.0, 3.0)
    ros.until(lambda: all(c.service_is_ready() for c in abort_clients)
              and all(d.mode_client.service_is_ready() for d in drones))

    for index, node in enumerate(drones):
        response = abort_clients[index].call_async(Trigger.Request())
        ros.until(response.done)
        assert response.result().success
        assert node.state == 'ABORT' and node.desired is None
        if index == 0:
            assert drones[1].state == 'NAVIGATE'
            assert drones[1].desired == (1.0, 2.0, 3.0)
        node._tick()
        ros.until(lambda: len(modes[index]) == 1 and node.pending is None)
        assert modes[index] == ['RTL']
        if index == 0:
            assert modes[1] == []
