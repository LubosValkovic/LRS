# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Enable attitude and local position streams without version-specific SR parameters."""

import signal
import time

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import MessageInterval
from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


# MAVLink HEARTBEAT, ATTITUDE and LOCAL_POSITION_NED.
# More frequent heartbeats keep slow simulations within MAVROS wall-time limits.
# MAVROS needs both attitude and local position to publish poses.
STREAMS = ((0, 5.0), (30, 10.0), (32, 10.0))


class TelemetrySetup(Node):
    def __init__(self, **node_kwargs):
        super().__init__('telemetry_setup', **node_kwargs)
        self.connected = False
        self.last_state = 0.0
        self.index = 0
        self.request_future = None
        self.next_request = 0.0
        self.clock_future = None
        self.clock_ready = False
        self.clock_retry = 0.0
        self.clock_client = self.create_client(SetParameters, 'mavros/time/set_parameters')
        self.interval_client = self.create_client(MessageInterval, 'mavros/set_message_interval')
        self.create_subscription(State, 'mavros/state', self.on_state, 10)
        self.create_timer(0.25, self.tick)

    def reset(self):
        if self.request_future is not None:
            self.request_future.cancel()
        self.request_future = None
        self.index = 0
        self.next_request = 0.0

    def on_state(self, msg):
        if not msg.connected or not self.connected:
            self.reset()
        self.connected = msg.connected
        self.last_state = time.monotonic()

    def tick(self):
        now = time.monotonic()
        self.configure_clock(now)
        if not self.connected or now - self.last_state > 5.0:
            self.reset()
            return
        if self.request_future is not None:
            if self.request_future.done():
                try:
                    accepted = self.request_future.result().success
                except Exception:
                    accepted = False
                self.request_future = None
                if accepted:
                    self.index += 1
                    self.next_request = now
                    if self.index == len(STREAMS):
                        self.get_logger().info('Heartbeat at 5 Hz; attitude and local position at 10 Hz enabled.')
                else:
                    self.get_logger().warning('Telemetry request rejected; retrying in 2 s.')
                    self.next_request = now + 2.0
            elif now >= self.next_request:
                self.request_future.cancel()
                self.request_future = None
                self.next_request = now + 2.0
                self.get_logger().warning('Telemetry request timed out; retrying in 2 s.')
            return
        if self.index < len(STREAMS) and now >= self.next_request and self.interval_client.service_is_ready():
            message_id, rate = STREAMS[self.index]
            # One outstanding command per FCU: all interval requests share MAV_CMD 511.
            self.request_future = self.interval_client.call_async(
                MessageInterval.Request(message_id=message_id, message_rate=rate))
            self.next_request = now + 3.0

    def configure_clock(self, now):
        # Plugin nodes ignore global YAML overrides in MAVROS 2.15. Configure
        # the time plugin explicitly; UAS use_sim_time comes from mavros.yaml.
        if self.clock_ready and self.clock_client.service_is_ready():
            return
        self.clock_ready = False
        if self.clock_future is not None:
            if self.clock_future.done():
                try:
                    results = self.clock_future.result().results
                    self.clock_ready = len(results) == 3 and all(r.successful for r in results)
                except Exception:
                    self.clock_ready = False
                self.clock_future = None
                if self.clock_ready:
                    self.get_logger().info('Pose stamps use Gazebo ROS time; FCU boot-time sync disabled.')
            elif now >= self.clock_retry:
                self.clock_future.cancel()
                self.clock_future = None
            return
        if now >= self.clock_retry and self.clock_client.service_is_ready():
            self.clock_future = self.clock_client.call_async(SetParameters.Request(parameters=[
                ParameterMsg(name='use_sim_time', value=ParameterValue(type=1, bool_value=True)),
                ParameterMsg(name='timesync_mode', value=ParameterValue(type=4, string_value='NONE')),
                ParameterMsg(name='timesync_rate', value=ParameterValue(type=3, double_value=0.0)),
            ]))
            self.clock_retry = now + 3.0


def main(args=None):
    rclpy.init(args=args)
    node = TelemetrySetup()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Launch may forward SIGINT after the terminal already delivered it.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
