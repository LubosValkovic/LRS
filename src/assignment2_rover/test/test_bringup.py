# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

import json
import time

import pytest
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from assignment2_rover.check_bringup import BringupCheck, Stream, metadata


def sample(kind, stamp=1):
    msg = {'pose': PoseStamped, 'velocity': TwistStamped, 'state': State,
           'image': Image, 'camera_info': CameraInfo}[kind]()
    msg.header.stamp.sec = stamp
    msg.header.frame_id = 'camera' if kind in ('image', 'camera_info') else 'map'
    if kind == 'pose':
        msg.pose.orientation.w = 1.0
    elif kind == 'state':
        msg.connected = True
    elif kind in ('image', 'camera_info'):
        msg.width, msg.height = 2, 2
        if kind == 'image':
            msg.encoding, msg.step, msg.data = 'rgb8', 6, bytes(12)
        else:
            msg.k = [1., 0., 1., 0., 1., 1., 0., 0., 1.]
    return msg


def test_checker_requires_messages_and_correct_targets_not_just_topics(ros):
    check = ros.make(BringupCheck)
    probe = ros.make(Node, 'input_probe')
    publishers = {topic: probe.create_publisher(type(sample(s.kind)), topic,
                  10 if s.kind == 'state' else qos_profile_sensor_data)
                  for topic, s in check.streams.items()}
    targets = {1: 1, 2: 1}  # Deliberately wrong second FCU.
    for sysid in (1, 2):
        def params(request, response, sysid=sysid):
            assert request.names == ['target_system_id']
            response.values = [ParameterValue(type=2, integer_value=targets[sysid])]
            return response
        probe.create_service(GetParameters, f'/drone_{sysid}/mavros/get_parameters', params)
    ros.until(lambda: all(p.get_subscription_count() for p in publishers.values()))
    assert not check.ready(time.monotonic(), 5.0)
    assert not check.report(None, 0.1, 5.0)['passed']

    for stamp in (1, 2, 3):
        for topic, publisher in publishers.items():
            publisher.publish(sample(check.streams[topic].kind, stamp))
        ros.until(lambda: all(len(s.samples) == stamp for s in check.streams.values()))
    def targets_received():
        check.update_targets()
        return len(check.targets) == 2
    ros.until(targets_received)
    assert not check.ready(time.monotonic(), 5.0)
    assert not check.report(time.monotonic() - 1, 0.1, 5.0)['passed']

    targets[2] = 2
    check.targets.clear()
    check.target_futures.clear()
    ros.until(targets_received)
    assert check.ready(time.monotonic(), 5.0)
    assert check.report(time.monotonic() - 1, 0.1, 5.0)['passed']
    info = check.streams['/drone_2/down_camera/camera_info'].message
    info.width = 3
    assert not check.report(time.monotonic() - 1, 0.1, 5.0)['passed']


@pytest.mark.parametrize('fault', ['stale', 'frozen', 'no_publisher', 'disconnect', 'image', 'pose_nan'])
def test_live_check_rejects_bad_or_stopped_streams(fault):
    kind = {'disconnect': 'state', 'image': 'image', 'pose_nan': 'pose'}.get(fault, 'pose')
    stream = Stream(kind)
    for i in (1, 2, 3):
        msg = sample(kind, 1 if fault == 'frozen' else i)
        if fault == 'disconnect':
            msg.connected = False
        elif fault == 'image':
            msg.data = bytes(1)
        elif fault == 'pose_nan':
            msg.pose.position.x = float('nan')
        stream.receive(msg, float(i))
    report = stream.report(10. if fault == 'stale' else 3., 2.,
                           [] if fault == 'no_publisher' else ['probe'])
    assert report['problems']
    json.dumps(report, allow_nan=False)  # Invalid numeric input must still yield a usable report.


def test_invalid_calibration_is_rejected_without_breaking_json_report():
    stream = Stream('camera_info')
    msg = sample('camera_info')
    msg.k[0] = float('inf')
    stream.receive(msg, 1.)
    assert 'invalid camera calibration' in stream.report(1., 2., ['probe'])['problems']
    assert metadata('camera_info', msg)['k'][0] is None
