# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""MAVROS x2 and one-way camera bridges; SITL and Gazebo run separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from assignment2_rover.vehicles import VEHICLES


def generate_launch_description():
    config_dir = Path(get_package_share_directory('assignment2_rover')) / 'config'
    actions = []
    for vehicle in VEHICLES:
        frame_prefix = vehicle.namespace.lstrip('/')
        actions.append(Node(
            package='mavros', executable='mavros_node', namespace=vehicle.namespace,
            output='screen', parameters=[str(config_dir / 'mavros.yaml'), {
                'fcu_url': f'udp://0.0.0.0:{vehicle.mavlink_port}@',
                'gcs_url': '',
                'tgt_system': vehicle.sysid,
                'tgt_component': 1,
                'base_link_frame': frame_prefix + '/base_link',
                'odom_frame': frame_prefix + '/odom',
                'map_frame': frame_prefix + '/map',
            }],
        ))
        actions.append(Node(
            package='assignment2_rover', executable='telemetry_setup',
            namespace=vehicle.namespace, output='screen',
        ))
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='rover_cameras',
        output='screen', parameters=[{'config_file': str(config_dir / 'cameras.yaml')}],
    ))
    return LaunchDescription(actions)
