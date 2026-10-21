# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Render the recorded outdoor target and GPS-derived tracks as a PNG."""

import argparse

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag2_py
from nav_msgs.msg import Path
from rclpy.serialization import deserialize_message


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', help='rosbag directory with outdoor Path topics')
    parser.add_argument('output', help='PNG output path')
    args = parser.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id='mcap'),
                rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                            output_serialization_format='cdr'))
    topics = {'/assignment1_outdoor/actual_path': None,
              '/assignment1_outdoor/target_path': None}
    while reader.has_next():
        name, payload, _ = reader.read_next()
        if name in topics:
            topics[name] = payload
    # Every Path is a history snapshot: only deserialize the latest one.
    topics = {name: deserialize_message(payload, Path) if payload is not None else None
              for name, payload in topics.items()}
    if any(path is None or not path.poses for path in topics.values()):
        raise ValueError('bag lacks recorded actual/target Path messages')
    fig, ax = plt.subplots(figsize=(7, 6))
    for name, color, label in (('/assignment1_outdoor/target_path', 'tab:blue', 'Commanded'),
                               ('/assignment1_outdoor/actual_path', 'tab:orange', 'GPS-derived')):
        points = topics[name].poses
        ax.plot([p.pose.position.x for p in points],
                [p.pose.position.y for p in points], color=color, label=label, linewidth=1.6)
    ax.scatter([0], [0], c='black', marker='*', s=70, label='POI / home')
    ax.set(xlabel='East in MAVROS ENU (m)', ylabel='North in MAVROS ENU (m)',
           title='Assignment 1 outdoor square and figure-eight')
    ax.grid(alpha=0.3)
    ax.set_aspect('equal', adjustable='box')
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output, dpi=170)
    plt.close(fig)
    print(f'wrote {args.output}; {len(topics["/assignment1_outdoor/actual_path"].poses)} samples')


if __name__ == '__main__':
    main()
