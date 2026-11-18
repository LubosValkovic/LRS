# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Real OpenCV images and ROS routing; no autopilot, arming or flight."""

import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PolygonStamped, Pose, Point, Quaternion
from mavros_msgs.msg import PositionTarget, State
import numpy as np
import pytest
import rclpy
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from assignment1_mission.part02.control import FlightNode
from assignment2_rover.vision import MarkerDetectorNode, MarkerVision, OwnMarkerDetector


def image(ids=(1, 2)):
    gray = np.full((480, 640), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for index, marker_id in enumerate(ids):
        marker = (cv2.aruco.generateImageMarker(dictionary, marker_id, 144)
                  if hasattr(cv2.aruco, 'generateImageMarker')
                  else cv2.aruco.drawMarker(dictionary, marker_id, 144))
        x = 60 + index * 300
        gray[120:264, x:x + 144] = np.rot90(marker, index)
    return gray


def frame(node, ids=(1, 2)):
    msg = CvBridge().cv2_to_imgmsg(image(ids), encoding='mono8')
    msg.header.frame_id = 'down_camera'
    msg.header.stamp = node.get_clock().now().to_msg()
    return msg


def make_detector(ros, **parameters):
    return ros.make(MarkerDetectorNode, namespace='/drone_1',
                    parameter_overrides=[Parameter(k, value=v) for k, v in
                                         dict(sysid=1, **parameters).items()])


@pytest.mark.parametrize('sysid', [1, 2])
def test_real_aruco_filters_foreign_id_and_preserves_four_corners(sysid):
    points, visible = OwnMarkerDetector().detect(image(), sysid)
    assert set(visible) == {1, 2}
    assert len(points) == 4
    xs = [p[0] for p in points]
    assert (max(xs) < 300) if sysid == 1 else (min(xs) > 300)


def test_foreign_marker_and_blank_image_never_become_target():
    detector = OwnMarkerDetector()
    assert detector.detect(image((2,)), 1)[0] is None
    assert detector.detect(image(()), 1) == (None, ())


def test_ros_identity_check_and_consecutive_confirmation(ros):
    node = make_detector(ros)
    vision = node.vision
    probe = ros.make(Node, 'image_probe')
    target = [2]
    def parameters(request, response):
        response.values = [ParameterValue(type=2, integer_value=target[0])]
        return response
    probe.create_service(GetParameters, '/drone_1/mavros/get_parameters', parameters)
    publisher = probe.create_publisher(Image, '/drone_1/down_camera/image_raw', qos_profile_sensor_data)
    received = []
    probe.create_subscription(PolygonStamped, '/drone_1/marker_detector/corners', received.append, 10)
    ros.until(lambda: publisher.get_subscription_count() and vision.corners_pub.get_subscription_count()
              and vision.identity_client.service_is_ready())
    def identity(expected):
        vision.update_status()
        return vision.mavros_sysid == expected
    ros.until(lambda: identity(2))

    def deliver(ids=(1, 2)):
        msg = frame(node, ids)
        publisher.publish(msg)
        ros.until(lambda: vision.pending is not None)
        vision.process()
        return msg

    deliver()
    assert vision.snapshot() is None and vision.processed == 0
    target[0] = 1
    vision.identity_deadline = 0.
    ros.until(lambda: identity(1))
    deliver()
    assert vision.streak == 1 and vision.snapshot() is None
    deliver((2,))  # A different rover breaks the own-ID confirmation streak.
    assert vision.streak == 0 and vision.snapshot() is None
    for i in range(3):
        last = deliver()
        if i < 2:
            assert vision.snapshot() is None
    ros.until(lambda: len(received) == 1)
    assert vision.snapshot().sysid == 1
    assert received[0].header == last.header
    assert len(received[0].polygon.points) == 4
    assert max(p.x for p in received[0].polygon.points) < 300


@pytest.mark.parametrize('fault', ['old', 'future', 'repeat', 'truncated', 'no_frame'])
def test_invalid_or_replayed_images_cannot_confirm(ros, fault):
    node = make_detector(ros, confirm_frames=1)
    vision = node.vision
    vision.mavros_sysid = 1
    msg = frame(node)
    if fault == 'old':
        msg.header.stamp.sec -= 2
    elif fault == 'future':
        msg.header.stamp.sec += 2
    elif fault == 'repeat':
        vision.receive(msg)
    elif fault == 'truncated':
        msg.data = bytes(1)
    elif fault == 'no_frame':
        msg.header.frame_id = ''
    vision.receive(msg)
    vision.process()
    assert vision.snapshot() is None and vision.rejected == 1


def test_mailbox_keeps_only_latest_frame(ros):
    node = make_detector(ros, confirm_frames=1)
    vision = node.vision
    vision.mavros_sysid = 1
    first, latest = frame(node, (1,)), frame(node, (2,))
    vision.receive(first)
    vision.receive(latest)
    vision.process()
    assert vision.processed == 1 and vision.overwritten == 1
    assert vision.visible_ids == (2,) and vision.snapshot() is None


def test_late_result_and_changed_identity_are_discarded(ros, monkeypatch):
    node = make_detector(ros, confirm_frames=1, max_frame_age=0.05)
    vision = node.vision
    vision.mavros_sysid = 1
    detect = vision.detector.detect
    def slow(gray, sysid):
        time.sleep(.08)
        return detect(gray, sysid)
    monkeypatch.setattr(vision.detector, 'detect', slow)
    vision.receive(frame(node))
    vision.process()
    assert vision.snapshot() is None and 'during processing' in vision.last_rejection

    def changed(gray, sysid):
        vision.mavros_sysid = 2
        return detect(gray, sysid)
    monkeypatch.setattr(vision.detector, 'detect', changed)
    vision.receive(frame(node))
    vision.process()
    assert vision.snapshot() is None and 'identity changed' in vision.last_rejection


def test_snapshot_expires_when_camera_stops(ros, monkeypatch):
    node = make_detector(ros, confirm_frames=1)
    vision = node.vision
    vision.mavros_sysid = 1
    vision.receive(frame(node))
    vision.process()
    assert vision.snapshot() is not None
    monotonic = time.monotonic
    monkeypatch.setattr(time, 'monotonic', lambda: monotonic() + 1.)
    assert vision.snapshot() is None


def test_slow_vision_does_not_stop_flightnode_setpoint_timer():
    context = Context()
    rclpy.init(args=[], context=context, domain_id=79)
    executor = MultiThreadedExecutor(num_threads=4, context=context)
    node = FlightNode('vision_isolation', namespace='/drone_1', context=context,
                      use_global_arguments=False, parameter_overrides=[
                          Parameter('sysid', value=1), Parameter('max_frame_age', value=2.0)])
    # Cancel all autonomous flight timers. Only the synthetic setpoint publisher runs.
    for timer in node.timers:
        timer.cancel()
    node.vehicle = State(connected=True, armed=True, mode='GUIDED')
    node.pose = Pose(position=Point(z=2.), orientation=Quaternion(w=1.))
    node.last_state = node.last_pose = time.monotonic()
    node.state, node.desired = 'NAVIGATE', (0., 0., 2.)
    vision = MarkerVision(node)
    probe = Node('cadence_probe', context=context, use_global_arguments=False)
    def params(request, response):
        response.values = [ParameterValue(type=2, integer_value=1)]
        return response
    probe.create_service(GetParameters, '/drone_1/mavros/get_parameters', params)
    publisher = probe.create_publisher(Image, '/drone_1/down_camera/image_raw', qos_profile_sensor_data)
    received = []
    probe.create_subscription(PositionTarget, '/drone_1/mavros/setpoint_raw/local',
                              lambda msg: received.append(time.monotonic()), 10)
    node.create_timer(.05, node._publish_locked, callback_group=node.output_group)
    entered, release = threading.Event(), threading.Event()
    detect = vision.detector.detect
    def slow(gray, sysid):
        entered.set()
        assert release.wait(timeout=2.), 'test did not release vision callback'
        return detect(gray, sysid)
    vision.detector.detect = slow
    executor.add_node(node)
    executor.add_node(probe)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    def until(predicate):
        deadline = time.monotonic() + 3.
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.01)
        assert predicate()
    try:
        until(lambda: vision.mavros_sysid == 1 and publisher.get_subscription_count())
        publisher.publish(frame(node))
        assert entered.wait(timeout=2.)
        before = len(received)
        until(lambda: len(received) >= before + 3)
        assert not release.is_set()  # Actual ROS setpoints arrived while OpenCV was blocked.
        release.set()
        until(lambda: vision.processed == 1)
    finally:
        release.set()
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown(context=context)
