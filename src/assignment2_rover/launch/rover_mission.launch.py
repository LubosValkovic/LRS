# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Start camera localization and the single-drone search/follow controller."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from assignment2_rover.vehicles import VEHICLES


def nodes(context):
    sysid = int(LaunchConfiguration('vehicle').perform(context))
    vehicle = next(v for v in VEHICLES if v.sysid == sysid)
    projection = LaunchConfiguration('projection_config').perform(context)
    common = {
        'sysid': sysid,
        'use_sim_time': True,
        'confirm_frames': int(LaunchConfiguration('confirm_frames').perform(context)),
        'max_frame_age': float(LaunchConfiguration('max_frame_age').perform(context)),
        'processing_hz': float(LaunchConfiguration('processing_hz').perform(context)),
    }
    mission = {
        'sysid': sysid,
        'use_sim_time': True,
        'search_altitude': float(LaunchConfiguration('search_altitude').perform(context)),
        'track_altitude': float(LaunchConfiguration('track_altitude').perform(context)),
        'search_width': float(LaunchConfiguration('search_width').perform(context)),
        'search_length': float(LaunchConfiguration('search_length').perform(context)),
        'lane_spacing': float(LaunchConfiguration('lane_spacing').perform(context)),
        'search_speed': float(LaunchConfiguration('search_speed').perform(context)),
        'tracking_speed': float(LaunchConfiguration('tracking_speed').perform(context)),
        'target_timeout': float(LaunchConfiguration('target_timeout').perform(context)),
        'max_target_age': float(LaunchConfiguration('max_target_age').perform(context)),
        'velocity_gain': float(LaunchConfiguration('velocity_gain').perform(context)),
    }
    return [
        Node(package='assignment2_rover', executable='marker_localizer',
             namespace=vehicle.namespace, output='screen',
             parameters=[projection, common]),
        Node(package='assignment2_rover', executable='rover_mission',
             namespace=vehicle.namespace, output='screen', parameters=[mission]),
    ]


def generate_launch_description():
    config = Path(get_package_share_directory('assignment2_rover')) / 'config/projection.yaml'
    arguments = [
        DeclareLaunchArgument('vehicle', default_value='1', choices=['1', '2']),
        DeclareLaunchArgument('confirm_frames', default_value='3'),
        DeclareLaunchArgument('max_frame_age', default_value='0.5'),
        DeclareLaunchArgument('processing_hz', default_value='20.0'),
        DeclareLaunchArgument('projection_config', default_value=str(config)),
        DeclareLaunchArgument('search_altitude', default_value='19.0'),
        DeclareLaunchArgument('track_altitude', default_value='2.0'),
        DeclareLaunchArgument('search_width', default_value='30.0'),
        DeclareLaunchArgument('search_length', default_value='30.0'),
        DeclareLaunchArgument('lane_spacing', default_value='5.0'),
        DeclareLaunchArgument('search_speed', default_value='0.8'),
        DeclareLaunchArgument('tracking_speed', default_value='0.6'),
        DeclareLaunchArgument('target_timeout', default_value='1.0'),
        DeclareLaunchArgument('max_target_age', default_value='0.75'),
        DeclareLaunchArgument('velocity_gain', default_value='0.8'),
    ]
    return LaunchDescription([*arguments, OpaqueFunction(function=nodes)])
