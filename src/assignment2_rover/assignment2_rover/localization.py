# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Confirmed image observations -> time-matched points in MAVROS local ENU."""

from dataclasses import dataclass
import math
import threading
import time

import cv2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PointStamped, PoseStamped
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

from .projection import CameraModel, PoseHistory, marker_translation, to_local_enu
from .vision import MarkerVision, run_node, stamp_ns


@dataclass(frozen=True)
class LocalTarget:
    observation: object
    frame: str
    position: np.ndarray
    optical: np.ndarray
    reprojection_error: float
    pose_span: float


class MarkerLocalizer:
    def __init__(self, node, vision):
        self.node, self.vision = node, vision
        def parameter(name, default):
            return node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value
        self.marker_size = parameter('marker_size', 0.6378986866791745)
        self.camera_translation = parameter('camera_translation_body', [0.0, 0.0, -0.03])
        self.camera_rotation = parameter(
            'camera_rotation_body_optical', [0.7071067811865476, -0.7071067811865476, 0.0, 0.0])
        self.max_gap = parameter('max_pose_gap', .15)
        self.max_reprojection = parameter('max_reprojection_error', 2.5)
        self.calibration_timeout = parameter('calibration_timeout', 2.)
        if any(not math.isfinite(v) or v <= 0 for v in
               (self.marker_size, self.max_gap, self.max_reprojection, self.calibration_timeout)):
            raise ValueError('projection sizes, limits and timeouts must be finite and positive')
        # Validate the configured mount once, before accepting any observations.
        from .projection import BodyPose
        to_local_enu([0., 0., 1.], BodyPose(1, 'map', np.zeros(3), np.array([0., 0., 0., 1.]), 0.),
                     self.camera_translation, self.camera_rotation)
        self.lock = threading.RLock()
        self.history = PoseHistory()
        self.camera = None
        self.camera_received = self.camera_stamp = 0
        self.pending = self.target = None
        self.last_stamp = self.last_clock = self.epoch = 0
        self.state, self.last_error = 'WAIT_DETECTION', ''
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.projection_group = MutuallyExclusiveCallbackGroup()
        node.create_subscription(CameraInfo, 'down_camera/camera_info', self.camera_cb,
                                 qos_profile_sensor_data, callback_group=self.input_group)
        node.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_cb,
                                 qos_profile_sensor_data, callback_group=self.input_group)
        self.target_pub = node.create_publisher(PointStamped, '~/target', 10)
        self.status_pub = node.create_publisher(DiagnosticArray, '~/projection_status', 10)
        node.create_timer(.05, self.process, clock=Clock(clock_type=ClockType.STEADY_TIME),
                          callback_group=self.projection_group)

    def camera_cb(self, msg):
        with self.lock:
            try:
                if (msg.distortion_model not in ('plumb_bob', 'rational_polynomial')
                        or msg.binning_x not in (0, 1) or msg.binning_y not in (0, 1)
                        or msg.roi.x_offset or msg.roi.y_offset
                        or msg.roi.width not in (0, msg.width) or msg.roi.height not in (0, msg.height)
                        or stamp_ns(msg.header) <= 0):
                    raise ValueError('unsupported camera distortion, ROI, binning or timestamp')
                camera = CameraModel(msg.width, msg.height, msg.header.frame_id,
                                     np.array(msg.k).reshape(3, 3), np.array(msg.d))
            except ValueError as error:
                self.camera = self.target = None
                self.epoch += 1
                self.last_error = str(error)
                return
            previous = self.camera
            if previous is None or (previous.frame, previous.width, previous.height) != (
                    camera.frame, camera.width, camera.height) or not np.array_equal(
                    previous.matrix, camera.matrix) or not np.array_equal(previous.distortion, camera.distortion):
                self.epoch += 1
                self.target = None
            self.camera, self.camera_received = camera, time.monotonic()
            self.camera_stamp = stamp_ns(msg.header)

    def pose_cb(self, msg):
        with self.lock:
            stamp = stamp_ns(msg.header)
            if abs(self.node.get_clock().now().nanoseconds - stamp) * 1e-9 > self.vision.max_age:
                self.history.samples.clear()
                self.target = None
                self.epoch += 1
                self.last_error = 'pose clock mismatch or old pose; check MAVROS use_sim_time'
                return
            p, q = msg.pose.position, msg.pose.orientation
            if self.history.samples and (stamp < self.history.samples[-1].stamp
                                         or msg.header.frame_id != self.history.samples[-1].frame):
                self.target = self.pending = None
                self.epoch += 1
            try:
                self.history.add(stamp, msg.header.frame_id, [p.x, p.y, p.z],
                                 [q.x, q.y, q.z, q.w], time.monotonic())
            except ValueError as error:
                self.target = None
                self.epoch += 1
                self.last_error = str(error)

    def snapshot(self):
        with self.lock:
            target = self.target
            if (target is None or self.vision.snapshot() is None
                    or not self.vision.fresh(target.observation, target.observation.received_at)
                    or time.monotonic() - self.camera_received > self.calibration_timeout
                    or not self.history.samples
                    or time.monotonic() - self.history.samples[-1].received_at > self.vision.max_age):
                return None
            return target

    def prepare(self):
        """Called under the data lock; hold one image until its pose bracket arrives."""
        clock = self.node.get_clock().now().nanoseconds
        if clock < self.last_clock:
            self.history.samples.clear()
            self.pending = self.target = None
            self.last_stamp = 0
            self.epoch += 1
        self.last_clock = clock
        if self.pending is None:
            observation = self.vision.snapshot()
            if observation is not None and stamp_ns(observation.header) > self.last_stamp:
                self.pending = observation
        observation = self.pending
        self.state = 'VALID' if self.snapshot() else 'WAIT_DETECTION'
        if observation is None:
            return None
        if not self.vision.fresh(observation, observation.received_at):
            self.last_stamp = stamp_ns(observation.header)
            self.pending = None
            self.last_error = 'observation expired while waiting for telemetry'
            return None
        if (self.camera is None or time.monotonic() - self.camera_received > self.calibration_timeout
                or abs(self.camera_stamp - stamp_ns(observation.header)) * 1e-9 > self.calibration_timeout):
            self.state = 'WAIT_CALIBRATION'
            return None
        if (self.camera.frame, self.camera.width, self.camera.height) != (
                observation.header.frame_id, observation.width, observation.height):
            self.state = 'WAIT_CALIBRATION'
            self.last_error = 'image and camera_info frame or dimensions differ'
            return None
        body = self.history.at(stamp_ns(observation.header), self.max_gap,
                               time.monotonic(), self.vision.max_age)
        if body is None:
            self.state = 'WAIT_POSE'
            return None
        self.pending = None
        self.last_stamp = stamp_ns(observation.header)
        return observation, body, self.camera, self.epoch

    def process(self):
        with self.lock:
            ready = self.prepare()
        if ready is not None:
            observation, body, camera, epoch = ready
            try:
                optical, error = marker_translation(observation.corners, camera, self.marker_size,
                                                     self.max_reprojection)
                position = to_local_enu(optical, body, self.camera_translation, self.camera_rotation)
                with self.lock:
                    if epoch == self.epoch and self.vision.snapshot() is not None and self.vision.fresh(
                            observation, observation.received_at):
                        self.target = LocalTarget(observation, body.frame, position, optical, error, body.bracket_span)
                        result = PointStamped(point=Point(x=float(position[0]), y=float(position[1]), z=float(position[2])))
                        result.header.frame_id, result.header.stamp = body.frame, observation.header.stamp
                        self.target_pub.publish(result)
                        self.state, self.last_error = 'VALID', ''
            except (ValueError, cv2.error) as error:
                with self.lock:
                    self.target = None
                    self.state, self.last_error = 'REJECTED', str(error)
        self.publish_status()

    def publish_status(self):
        with self.lock:
            target = self.snapshot()
            values = dict(sysid=self.vision.sysid, last_error=self.last_error)
            if target is not None:
                values.update(reprojection_error_px=target.reprojection_error,
                              optical_range_m=float(np.linalg.norm(target.optical)),
                              pose_bracket_span_s=target.pose_span,
                              target_enu_m=target.position.tolist())
            status = DiagnosticStatus(name=self.node.get_fully_qualified_name() + '/projection',
                hardware_id=str(self.vision.sysid), level=DiagnosticStatus.OK if target else DiagnosticStatus.WARN,
                message=self.state, values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        result = DiagnosticArray(status=[status])
        result.header.stamp = self.node.get_clock().now().to_msg()
        self.status_pub.publish(result)


class MarkerLocalizerNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__('marker_localizer', **node_kwargs)
        self.vision = MarkerVision(self)
        self.localizer = MarkerLocalizer(self, self.vision)


def main(args=None):
    run_node(MarkerLocalizerNode, args)
