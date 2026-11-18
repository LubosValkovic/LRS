# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Read-only check of both MAVROS connections and both camera streams."""

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from .vehicles import VEHICLES


def invalid_sample(kind, msg):
    if msg is None:
        return 'no messages'
    if kind == 'state':
        return None if msg.connected else 'FCU disconnected'
    if not msg.header.frame_id:
        return 'empty frame_id'
    if kind == 'pose':
        p, q = msg.pose.position, msg.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(map(math.isfinite, values)) or sum(v * v for v in values[3:]) < 1e-12:
            return 'invalid pose'
    elif kind == 'velocity':
        v = msg.twist.linear
        if not all(map(math.isfinite, (v.x, v.y, v.z))):
            return 'invalid velocity'
    elif kind == 'image':
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(msg.encoding)
        if (not channels or msg.width <= 0 or msg.height <= 0
                or msg.step < msg.width * channels or len(msg.data) < msg.step * msg.height):
            return 'invalid image layout'
    elif kind == 'camera_info':
        if (msg.width <= 0 or msg.height <= 0 or not all(map(math.isfinite, (*msg.k, *msg.d)))
                or msg.k[0] <= 0 or msg.k[4] <= 0 or msg.k[8] != 1.0):
            return 'invalid camera calibration'
    return None


def metadata(kind, msg):
    if msg is None:
        return {}
    if kind == 'state':
        return {'connected': msg.connected, 'armed': msg.armed, 'mode': msg.mode}
    result = {'frame_id': msg.header.frame_id}
    if kind == 'pose':
        p, q = msg.pose.position, msg.pose.orientation
        result.update(position=finite_values([p.x, p.y, p.z]),
                      orientation_xyzw=finite_values([q.x, q.y, q.z, q.w]))
    elif kind == 'velocity':
        v = msg.twist.linear
        result['linear'] = finite_values([v.x, v.y, v.z])
    elif kind == 'image':
        result.update(width=msg.width, height=msg.height, encoding=msg.encoding,
                      step=msg.step, data_bytes=len(msg.data))
    elif kind == 'camera_info':
        result.update(width=msg.width, height=msg.height, k=finite_values(msg.k), d=finite_values(msg.d),
                      distortion_model=msg.distortion_model)
    return result


def finite_values(values):
    return [v if math.isfinite(v) else None for v in values]


@dataclass
class Stream:
    kind: str
    message: object = None
    last_seen: float = 0.0
    samples: list = field(default_factory=list)
    invalid_count: int = 0

    def receive(self, msg, now):
        self.message, self.last_seen = msg, now
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.samples.append((now, stamp))
        self.invalid_count += invalid_sample(self.kind, msg) is not None

    def report(self, now, max_silence, publishers):
        problems = []
        invalid = invalid_sample(self.kind, self.message)
        if invalid:
            problems.append(invalid)
        if len(self.samples) < 3:
            problems.append('fewer than 3 received samples')
        if not publishers:
            problems.append('no live ROS publisher')
        if now - self.last_seen > max_silence:
            problems.append('stream is stale')
        if self.invalid_count:
            problems.append(f'{self.invalid_count} invalid samples during measurement')
        wall_hz = source_hz = 0.0
        if len(self.samples) >= 2:
            wall_dt = self.samples[-1][0] - self.samples[0][0]
            source_dt = self.samples[-1][1] - self.samples[0][1]
            wall_hz = (len(self.samples) - 1) / wall_dt if wall_dt > 0 else 0.0
            source_hz = (len(self.samples) - 1) / source_dt if source_dt > 0 else 0.0
            if source_dt <= 0:
                problems.append('header stamp did not advance')
        return {'kind': self.kind, 'samples': len(self.samples), 'wall_hz': wall_hz,
                'source_hz': source_hz, 'publishers': publishers,
                'latest': metadata(self.kind, self.message), 'problems': problems}


class BringupCheck(Node):
    def __init__(self, **node_kwargs):
        super().__init__('rover_bringup_check', **node_kwargs)
        self.streams, self.target_clients, self.target_futures, self.targets = {}, {}, {}, {}
        channels = [('mavros/state', 'state', State, 10),
                    ('mavros/local_position/pose', 'pose', PoseStamped, qos_profile_sensor_data),
                    ('mavros/local_position/velocity_local', 'velocity', TwistStamped, qos_profile_sensor_data),
                    ('down_camera/image_raw', 'image', Image, qos_profile_sensor_data),
                    ('down_camera/camera_info', 'camera_info', CameraInfo, qos_profile_sensor_data)]
        for vehicle in VEHICLES:
            for suffix, kind, message_type, qos in channels:
                topic = vehicle.namespace + '/' + suffix
                self.streams[topic] = Stream(kind)
                self.create_subscription(message_type, topic,
                                         lambda msg, topic=topic: self.streams[topic].receive(msg, time.monotonic()), qos)
            self.target_clients[vehicle.sysid] = self.create_client(
                GetParameters, vehicle.namespace + '/mavros/get_parameters')

    def update_targets(self):
        for sysid, client in self.target_clients.items():
            if sysid not in self.target_futures and client.service_is_ready():
                self.target_futures[sysid] = client.call_async(GetParameters.Request(names=['target_system_id']))
            future = self.target_futures.get(sysid)
            if future is not None and future.done() and sysid not in self.targets:
                try:
                    values = future.result().values
                    self.targets[sysid] = (values[0].integer_value if len(values) == 1
                                           and values[0].type == ParameterType.PARAMETER_INTEGER else None)
                except Exception:
                    self.targets[sysid] = None

    def ready(self, now, max_silence):
        return (all(self.targets.get(v.sysid) == v.sysid for v in VEHICLES)
                and all(invalid_sample(s.kind, s.message) is None
                        and now - s.last_seen <= max_silence for s in self.streams.values()))

    def report(self, started, duration, max_silence):
        now = time.monotonic()
        complete = started is not None and now - started >= duration
        streams = {}
        for topic, stream in self.streams.items():
            publishers = [f'{p.node_namespace.rstrip("/")}/{p.node_name}'
                          for p in self.get_publishers_info_by_topic(topic)]
            streams[topic] = stream.report(now, max_silence, publishers)
        problems = [] if complete else ['startup timed out before a complete measurement']
        for vehicle in VEHICLES:
            if self.targets.get(vehicle.sysid) != vehicle.sysid:
                problems.append(f'{vehicle.namespace}: expected target_system_id {vehicle.sysid}')
            image = self.streams[vehicle.namespace + '/down_camera/image_raw'].message
            info = self.streams[vehicle.namespace + '/down_camera/camera_info'].message
            if image is not None and info is not None:
                if (image.width, image.height, image.header.frame_id) != (info.width, info.height, info.header.frame_id):
                    problems.append(f'{vehicle.namespace}: image/calibration dimensions or frames differ')
        passed = complete and not problems and all(not s['problems'] for s in streams.values())
        return {'passed': passed, 'measurement_wall_seconds': now - started if started else 0.0,
                'target_system_ids': self.targets, 'problems': problems, 'streams': streams}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=10.0, help='measurement window in wall seconds')
    parser.add_argument('--timeout', type=float, default=90.0, help='startup timeout in wall seconds')
    parser.add_argument('--max-silence', type=float, default=5.0)
    parser.add_argument('--output', type=Path, default=Path('log/rover_bringup.json'))
    args, ros_args = parser.parse_known_args(argv)
    if any(not math.isfinite(v) or v <= 0 for v in [args.duration, args.timeout, args.max_silence]):
        parser.error('duration, timeout and max-silence must be finite and positive')
    rclpy.init(args=ros_args)
    node = BringupCheck()
    started = None
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.update_targets()
            now = time.monotonic()
            if started is None and node.ready(now, args.max_silence):
                started = now
                for stream in node.streams.values():
                    stream.samples.clear()
                    stream.invalid_count = 0
                print('Both FCUs and both cameras ready; measuring live messages.', flush=True)
            if (started is None and now >= deadline) or (started is not None and now - started >= args.duration):
                break
        report = node.report(started, args.duration, args.max_silence)
    except KeyboardInterrupt:
        report = node.report(started, args.duration, args.max_silence)
        report['passed'] = False
        report['problems'].append('check interrupted')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    for topic, stream in report['streams'].items():
        print(f'{topic}: {stream["samples"]} samples, {stream["wall_hz"]:.2f} Hz wall, '
              f'{stream["source_hz"]:.2f} Hz source; {stream["problems"] or "OK"}')
    print(f'{"PASS" if report["passed"] else "FAIL"}: {args.output}')
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
