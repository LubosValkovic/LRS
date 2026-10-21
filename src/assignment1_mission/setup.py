# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

from pathlib import Path

from setuptools import find_packages, setup


package_name = 'assignment1_mission'
source_dir = Path(__file__).parent / package_name
console_scripts = [
    'plan3d = assignment1_mission.part01.plan_cli:main',
]
if (source_dir / 'part02' / 'indoor.py').is_file():
    console_scripts.append('indoor = assignment1_mission.part02.indoor:main')
if (source_dir / 'part03' / 'outdoor.py').is_file():
    console_scripts.append('outdoor = assignment1_mission.part03.outdoor:main')

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
    description='Assignment 1 indoor and outdoor autonomous missions',
    license='MIT',
    entry_points={'console_scripts': console_scripts},
)
