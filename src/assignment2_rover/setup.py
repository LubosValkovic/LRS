# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

from glob import glob

from setuptools import find_packages, setup


package_name = 'assignment2_rover'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='STU FEI URK',
    maintainer_email='lrs@fei.stuba.sk',
    description='Assignment 2 rover simulation, ArUco perception and input verification',
    license='MIT',
    entry_points={'console_scripts': [
        'sitl = assignment2_rover.sitl:main',
        'telemetry_setup = assignment2_rover.telemetry:main',
        'check_bringup = assignment2_rover.check_bringup:main',
        'marker_detector = assignment2_rover.vision:main',
        'marker_localizer = assignment2_rover.localization:main',
    ]},
)
