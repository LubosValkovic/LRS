#!/usr/bin/env python3
"""Drive the find-and-follow rovers with random, seeded velocity commands.

Usage:
  scripts/randomize_rovers.py [--seed N] [options]      (--help lists the options)

Run it after starting the rover world (scripts/run_gazebo.sh
fei_lrs_precision_landing.world). Every few seconds of simulation time each rover
gets a new random speed and turn rate. Commands are ramped, and the turn rate is
limited at speed, so the rover never tips over. A rover that strays more than
--radius metres from the centre of its original circle turns back toward it.
Ctrl+C stops both rovers.

The script talks to Gazebo directly over gz-transport, so no ROS bridge is needed.
The same seed gives the same sequence of commands.
"""

import argparse
import math
import random
import signal
import sys
import threading
import time
from dataclasses import dataclass

# Centres of the circles the rovers drive in worlds/fei_lrs_precision_landing.world,
# in Gazebo world coordinates (see scripts/generate_precision_landing.py).
ROVERS = {
    'aruco_rover_1': (-10.0, 2.0),
    'aruco_rover_2': (10.0, 2.0),
}


def wrap(angle):
    """Wrap an angle to [-pi, pi)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    """Return the yaw of an x, y, z, w quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def approach(current, target, max_step):
    """Move current toward target by at most max_step."""
    if target > current:
        return min(target, current + max_step)
    return max(target, current - max_step)


@dataclass
class Limits:
    """Tunable limits of the random motion; all in SI units."""

    speed_min: float = 0.15        # m/s
    speed_max: float = 0.40        # m/s
    yaw_rate_max: float = 0.20     # rad/s
    interval_min: float = 3.0      # s between new random commands
    interval_max: float = 6.0      # s
    radius: float = 4.0            # m from the circle centre before turning back
    accel: float = 0.2             # m/s^2, speed ramp
    yaw_accel: float = 0.2         # rad/s^2, turn-rate ramp
    lateral_accel: float = 0.1     # m/s^2, cap on speed x turn rate
    return_yaw_rate: float = 0.3   # rad/s while turning back


class RoverMotion:
    """Decide the next velocity command for one rover. No Gazebo dependency."""

    def __init__(self, home, limits, rng):
        """Start in random mode around home, using rng for every choice."""
        self.home = home
        self.limits = limits
        self.rng = rng
        self.reset()

    def reset(self):
        """Forget all state, e.g. after a world reset."""
        self.mode = 'random'
        self.last_t = None
        self.next_change = None
        self.target_v = 0.0
        self.target_w = 0.0
        self.v = None
        self.w = None
        self.events = []

    def _new_random_target(self, t):
        lim = self.limits
        self.target_v = self.rng.uniform(lim.speed_min, lim.speed_max)
        self.target_w = self.rng.uniform(-lim.yaw_rate_max, lim.yaw_rate_max)
        self.next_change = t + self.rng.uniform(lim.interval_min,
                                                lim.interval_max)
        self.events.append(
            f't={t:7.1f} s  new command {self.target_v:.2f} m/s, '
            f'{self.target_w:+.2f} rad/s')

    def step(self, t, x, y, yaw, measured_v, measured_w):
        """Return (speed, turn rate) to command at simulation time t."""
        lim = self.limits
        if self.last_t is not None and t < self.last_t:
            self.reset()                      # simulation time jumped back
        if self.v is None:                    # take over smoothly
            self.v, self.w = measured_v, measured_w
        dt = 0.0 if self.last_t is None else min(t - self.last_t, 0.5)
        self.last_t = t

        distance = math.hypot(x - self.home[0], y - self.home[1])
        if self.mode == 'random' and distance > lim.radius:
            self.mode = 'return'
            self.events.append(
                f't={t:7.1f} s  {distance:.1f} m from the centre, turning back')
        elif self.mode == 'return' and distance < lim.radius / 2.0:
            self.mode = 'random'
            self.next_change = None

        if self.mode == 'random':
            if self.next_change is None or t >= self.next_change:
                self._new_random_target(t)
        else:
            heading_error = wrap(math.atan2(self.home[1] - y,
                                            self.home[0] - x) - yaw)
            self.target_w = max(-lim.return_yaw_rate,
                                min(lim.return_yaw_rate, heading_error))
            self.target_v = (lim.speed_min if abs(heading_error) > 0.5
                             else 0.5 * (lim.speed_min + lim.speed_max))

        self.v = approach(self.v, self.target_v, lim.accel * dt)
        self.w = approach(self.w, self.target_w, lim.yaw_accel * dt)
        if abs(self.v * self.w) > lim.lateral_accel:
            self.w = math.copysign(lim.lateral_accel / max(abs(self.v), 1e-3),
                                   self.w)
        return self.v, self.w


def parse_args(argv):
    """Parse command-line options into (seed, Limits)."""
    defaults = Limits()
    parser = argparse.ArgumentParser(
        description='Drive the find-and-follow rovers with random, seeded '
                    'velocity commands. Ctrl+C stops both rovers.')
    parser.add_argument('--seed', type=int, default=42,
                        help='random seed (default 42)')
    for name, help_text in [
            ('speed_min', 'lowest random speed, m/s'),
            ('speed_max', 'highest random speed, m/s'),
            ('yaw_rate_max', 'highest random turn rate, rad/s'),
            ('interval_min', 'shortest time between new commands, s'),
            ('interval_max', 'longest time between new commands, s'),
            ('radius', 'distance from the circle centre before turning back, m'),
    ]:
        parser.add_argument('--' + name.replace('_', '-'), type=float,
                            default=getattr(defaults, name),
                            help=f'{help_text} (default {getattr(defaults, name)})')
    args = parser.parse_args(argv)
    limits = Limits(**{k: v for k, v in vars(args).items() if k != 'seed'})
    if not 0 < limits.speed_min <= limits.speed_max:
        parser.error('need 0 < --speed-min <= --speed-max')
    if not 0 < limits.interval_min <= limits.interval_max:
        parser.error('need 0 < --interval-min <= --interval-max')
    if limits.yaw_rate_max < 0 or limits.radius <= 0:
        parser.error('--yaw-rate-max must be >= 0 and --radius > 0')
    return args.seed, limits


def main(argv=None):
    """Connect to Gazebo and drive both rovers until Ctrl+C."""
    seed, limits = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        from gz.msgs10.odometry_pb2 import Odometry
        from gz.msgs10.twist_pb2 import Twist
        from gz.transport13 import Node
    except ImportError as error:
        sys.exit(f'Error: Gazebo Harmonic Python bindings not found ({error}). '
                 'They come with the gz-harmonic package from scripts/install.sh.')

    node = Node()
    stopping = threading.Event()
    seen = {name: threading.Event() for name in ROVERS}
    publishers = {}

    for index, (name, home) in enumerate(ROVERS.items()):
        publisher = node.advertise(f'/model/{name}/cmd_vel', Twist)
        motion = RoverMotion(home, limits, random.Random(seed * 1000 + index))
        lock = threading.Lock()
        publishers[name] = publisher

        def on_odometry(msg, name=name, publisher=publisher, motion=motion,
                        lock=lock):
            if stopping.is_set():
                return
            q = msg.pose.orientation
            with lock:
                v, w = motion.step(
                    msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9,
                    msg.pose.position.x, msg.pose.position.y,
                    yaw_from_quaternion(q.x, q.y, q.z, q.w),
                    msg.twist.linear.x, msg.twist.angular.z)
                events, motion.events = motion.events, []
            command = Twist()
            command.linear.x = v
            command.angular.z = w
            publisher.publish(command)
            seen[name].set()
            for event in events:
                print(f'[{name}] {event}', flush=True)

        if not node.subscribe(Odometry, f'/model/{name}/odometry', on_odometry):
            sys.exit(f'Error: could not subscribe to /model/{name}/odometry')

    def request_stop(*_):
        stopping.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    deadline = time.monotonic() + 15.0
    while not all(e.is_set() for e in seen.values()) and not stopping.is_set():
        if time.monotonic() > deadline:
            missing = ', '.join(n for n, e in seen.items() if not e.is_set())
            sys.exit(f'Error: no odometry from {missing}. Is '
                     'fei_lrs_precision_landing.world running (and unpaused)?')
        time.sleep(0.1)
    if not stopping.is_set():
        print(f'Driving {", ".join(ROVERS)} with seed {seed}. '
              'Ctrl+C stops both rovers.', flush=True)

    while not stopping.wait(0.5):
        pass

    stop = Twist()                          # zero velocity: the rovers halt
    for _ in range(10):
        for publisher in publishers.values():
            publisher.publish(stop)
        time.sleep(0.05)
    print('Stopped both rovers.', flush=True)


if __name__ == '__main__':
    main()
