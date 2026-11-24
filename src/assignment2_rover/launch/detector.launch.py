# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Run perception for one selected vehicle; no flight commands."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from assignment2_rover.vehicles import VEHICLES


def detector(context):
    sysid = int(LaunchConfiguration('vehicle').perform(context))
    vehicle = next(v for v in VEHICLES if v.sysid == sysid)
    estimate = LaunchConfiguration('estimate_position').perform(context) == 'true'
    parameters = [LaunchConfiguration('projection_config').perform(context)] if estimate else []
    return [Node(package='assignment2_rover', executable='marker_localizer' if estimate else 'marker_detector',
                 namespace=vehicle.namespace, output='screen',
                 parameters=parameters + [{'sysid': sysid, 'use_sim_time': True,
                              'confirm_frames': int(LaunchConfiguration('confirm_frames').perform(context)),
                              'max_frame_age': float(LaunchConfiguration('max_frame_age').perform(context)),
                              'processing_hz': float(LaunchConfiguration('processing_hz').perform(context))}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vehicle', default_value='1', choices=['1', '2'],
                              description='Vehicle SYSID and matching ROS namespace'),
        DeclareLaunchArgument('confirm_frames', default_value='3',
                              description='Consecutive own-ID frames required for confirmation'),
        DeclareLaunchArgument('max_frame_age', default_value='0.5',
                              description='Maximum image age in seconds'),
        DeclareLaunchArgument('processing_hz', default_value='20.0',
                              description='Maximum image processing frequency'),
        DeclareLaunchArgument('estimate_position', default_value='false', choices=['false', 'true'],
                              description='Also estimate the marker position in local ENU'),
        DeclareLaunchArgument('projection_config', default_value=str(Path(
            get_package_share_directory('assignment2_rover')) / 'config/projection.yaml'),
                              description='Marker size, camera mount and temporal limits'),
        OpaqueFunction(function=detector),
    ])
