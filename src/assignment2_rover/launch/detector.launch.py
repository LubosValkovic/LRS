# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Run perception for one selected vehicle; no flight commands."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from assignment2_rover.vehicles import VEHICLES


def detector(context):
    sysid = int(LaunchConfiguration('vehicle').perform(context))
    vehicle = next(v for v in VEHICLES if v.sysid == sysid)
    return [Node(package='assignment2_rover', executable='marker_detector',
                 namespace=vehicle.namespace, output='screen',
                 parameters=[{'sysid': sysid, 'use_sim_time': True,
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
        OpaqueFunction(function=detector),
    ])
