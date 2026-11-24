# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Own-ID ArUco perception, composable with FlightNode without holding its lock."""

from dataclasses import dataclass
import math
import signal
import threading
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32, PolygonStamped
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class OwnMarkerDetector:
    """Return only the requested ID; preserve OpenCV's decoded corner ordering."""

    def __init__(self):
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'ArucoDetector')
                           else cv2.aruco.DetectorParameters_create())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = (cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
                         if hasattr(cv2.aruco, 'ArucoDetector') else None)

    def detect(self, gray, sysid):
        if self.detector is None:  # OpenCV 4.6, as installed with ROS Jazzy here.
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
        else:
            corners, ids, _ = self.detector.detectMarkers(gray)
        visible = tuple(int(i) for i in ids.flatten()) if ids is not None else ()
        matches = [c for c, i in zip(corners, visible) if i == sysid]
        if len(matches) != 1:
            return None, visible
        points = np.asarray(matches[0], dtype=np.float32).reshape(4, 2)
        height, width = gray.shape
        if (not np.isfinite(points).all() or not cv2.isContourConvex(points)
                or abs(cv2.contourArea(points)) < 16.0
                or np.any(points < 0) or np.any(points[:, 0] >= width)
                or np.any(points[:, 1] >= height)):
            return None, visible
        return tuple((float(x), float(y)) for x, y in points), visible


@dataclass(frozen=True)
class Observation:
    sysid: int
    header: object
    width: int
    height: int
    corners: tuple
    received_at: float


class MarkerVision:
    """Attach perception to a ROS node, including a future FlightNode subclass.

    The image callback only replaces a one-frame mailbox. OpenCV runs in its own
    callback group, outside both this short-lived data lock and FlightNode.lock.
    """

    def __init__(self, node):
        self.node = node
        def parameter(name, default):
            return node.declare_parameter(name, default, ParameterDescriptor(read_only=True)).value

        self.sysid = parameter('sysid', 0)
        self.confirm_frames = parameter('confirm_frames', 3)
        self.max_age = parameter('max_frame_age', 0.5)
        processing_hz = parameter('processing_hz', 20.0)
        if not 1 <= self.sysid < 50:
            raise ValueError('sysid must match the vehicle SYSID and be in 1..49 (DICT_4X4_50)')
        if self.confirm_frames < 1:
            raise ValueError('confirm_frames must be positive')
        if any(not math.isfinite(v) or v <= 0 for v in (self.max_age, processing_hz)):
            raise ValueError('max_frame_age and processing_hz must be finite and positive')
        self.lock = threading.RLock()
        self.bridge, self.detector = CvBridge(), OwnMarkerDetector()
        self.pending = self.observation = None
        self.last_stamp = self.last_clock = 0
        self.streak = 0
        self.epoch = 0
        self.last_own = 0.0
        self.visible_ids = ()
        self.processed = self.rejected = self.overwritten = 0
        self.last_rejection = ''
        self.processing_ms = 0.0
        self.mavros_sysid = None
        self.identity_future = None
        self.identity_deadline = 0.0
        self.identity_group = MutuallyExclusiveCallbackGroup()
        self.input_group = MutuallyExclusiveCallbackGroup()
        self.vision_group = MutuallyExclusiveCallbackGroup()
        self.status_group = MutuallyExclusiveCallbackGroup()
        self.identity_client = node.create_client(GetParameters, 'mavros/get_parameters',
                                                  callback_group=self.identity_group)
        self.corners_pub = node.create_publisher(PolygonStamped, '~/corners', 10)
        self.status_pub = node.create_publisher(DiagnosticArray, '~/status', 10)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(Image, 'down_camera/image_raw', self.receive, qos,
                                 callback_group=self.input_group)
        # Watchdogs continue to expire data even when the simulation clock stops.
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(1.0 / processing_hz, self.process, clock=steady,
                          callback_group=self.vision_group)
        node.create_timer(0.1, self.update_status, clock=steady, callback_group=self.status_group)

    def fresh(self, msg, received_at):
        age = (self.node.get_clock().now().nanoseconds - stamp_ns(msg.header)) * 1e-9
        return 0 <= age <= self.max_age and time.monotonic() - received_at <= self.max_age

    def reject(self, reason):
        self.rejected += 1
        self.last_rejection = reason
        self.streak = 0
        self.pending = None
        self.epoch += 1

    def receive(self, msg):
        now = time.monotonic()
        with self.lock:
            clock_now = self.node.get_clock().now().nanoseconds
            if clock_now < self.last_clock:  # New Gazebo clock epoch after a reset.
                self.last_stamp = self.streak = 0
                self.epoch += 1
                self.pending = self.observation = None
            self.last_clock = clock_now
            stamp = stamp_ns(msg.header)
            channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}.get(msg.encoding)
            if (not msg.header.frame_id or not channels or msg.width <= 0 or msg.height <= 0
                    or msg.step < msg.width * channels or len(msg.data) < msg.step * msg.height):
                self.reject('invalid image layout or frame')
            elif stamp <= 0 or stamp <= self.last_stamp or not self.fresh(msg, now):
                self.reject('old, repeated or future image stamp; check use_sim_time and /clock')
            else:
                self.last_stamp = stamp
                self.overwritten += self.pending is not None
                self.pending = (msg, now)

    def process(self):
        with self.lock:
            pending, self.pending = self.pending, None
            if pending is None:
                return
            msg, received_at = pending
            epoch = self.epoch
            if self.mavros_sysid != self.sysid:
                self.reject('waiting for matching MAVROS target_system_id')
                return
            if not self.fresh(msg, received_at):
                self.reject('image expired before processing')
                return
        started = time.monotonic()
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            corners, visible = self.detector.detect(gray, self.sysid)
        except (CvBridgeError, cv2.error, ValueError) as error:
            with self.lock:
                self.reject(f'image conversion/detection failed: {error}')
            return
        with self.lock:
            self.processing_ms = (time.monotonic() - started) * 1000
            self.processed += 1
            self.visible_ids = visible
            if epoch != self.epoch or self.mavros_sysid != self.sysid:
                self.reject('input or vehicle identity changed during processing')
                return
            if not self.fresh(msg, received_at):
                self.reject('image expired during processing')
                return
            if corners is None:
                self.streak = 0
                return
            self.streak = self.streak + 1 if received_at - self.last_own <= self.max_age else 1
            self.last_own = received_at
            if self.streak >= self.confirm_frames:
                self.observation = Observation(self.sysid, msg.header, msg.width, msg.height,
                                               corners, received_at)
                result = PolygonStamped(header=msg.header)
                result.polygon.points = [Point32(x=x, y=y, z=0.) for x, y in corners]
                self.corners_pub.publish(result)

    def snapshot(self):
        with self.lock:
            result = self.observation
            if (result is None or self.mavros_sysid != self.sysid
                    or not self.fresh(result, result.received_at)):
                return None
            return result

    def update_status(self):
        now = time.monotonic()
        with self.lock:
            if self.identity_future is not None:
                if self.identity_future.done():
                    try:
                        values = self.identity_future.result().values
                        self.mavros_sysid = (values[0].integer_value if len(values) == 1
                                             and values[0].type == ParameterType.PARAMETER_INTEGER else None)
                    except Exception:
                        self.mavros_sysid = None
                    self.identity_future = None
                elif now >= self.identity_deadline:
                    self.identity_future.cancel()
                    self.identity_future = None
                    self.mavros_sysid = None
            if now >= self.identity_deadline and self.identity_future is None:
                if self.identity_client.service_is_ready():
                    self.identity_future = self.identity_client.call_async(
                        GetParameters.Request(names=['target_system_id']))
                else:
                    self.mavros_sysid = None
                self.identity_deadline = now + 3.0
            if now - self.last_own > self.max_age:
                self.streak = 0
            observation = self.snapshot()
            state = ('WAIT_ID' if self.mavros_sysid != self.sysid else
                     'CONFIRMED' if observation else 'CANDIDATE' if self.streak else 'WAIT_IMAGE')
            values = dict(sysid=self.sysid, mavros_sysid=self.mavros_sysid,
                          visible_ids=list(self.visible_ids), candidate_frames=self.streak,
                          processed=self.processed, rejected=self.rejected, overwritten=self.overwritten,
                          processing_ms=round(self.processing_ms, 3), last_rejection=self.last_rejection)
        status = DiagnosticStatus(name=self.node.get_fully_qualified_name(), hardware_id=str(self.sysid),
                                  level=DiagnosticStatus.OK if observation else DiagnosticStatus.WARN,
                                  message=state, values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        result = DiagnosticArray(status=[status])
        result.header.stamp = self.node.get_clock().now().to_msg()
        self.status_pub.publish(result)


class MarkerDetectorNode(Node):
    def __init__(self, **node_kwargs):
        super().__init__('marker_detector', **node_kwargs)
        self.vision = MarkerVision(self)


def run_node(node_class, args=None):
    # Let callbacks finish before shutting down their clock and ROS handles.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = node_class()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


def main(args=None):
    run_node(MarkerDetectorNode, args)
