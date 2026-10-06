# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

from setuptools import find_packages, setup


package_name = 'assignment1_mission'
console_scripts = [
    'plan3d = assignment1_mission.part01.plan_cli:main',
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='STU FEI URK',
    maintainer_email='lrs@fei.stuba.sk',
    description='Assignment 1 Part 1: 3D voxel map and A* planner',
    license='MIT',
    entry_points={'console_scripts': console_scripts},
)
