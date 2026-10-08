# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Mission CSV parsing and shared ENU geometry."""

import csv
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    z: float
    precision: str
    task: str

    @property
    def point(self):
        return (self.x, self.y, self.z)


def load_mission(path):
    with open(path, newline='', encoding='utf-8') as stream:
        rows = csv.DictReader((line for line in stream if line.strip() and not line.lstrip().startswith('#')),
                              skipinitialspace=True)
        if rows.fieldnames is None or set(rows.fieldnames) != {'x', 'y', 'z', 'precision', 'task'}:
            raise ValueError('mission needs x,y,z,precision,task columns')
        mission = []
        for row in rows:
            try:
                values = (float(row[key]) for key in ('x', 'y', 'z'))
                x, y, z = values
                precision = row['precision'].strip().lower()
                task = row['task'].strip().lower()
            except (ValueError, TypeError, KeyError) as error:
                raise ValueError(f'bad mission row {row}: {error}') from error
            if not all(math.isfinite(v) for v in (x, y, z)) or z <= 0:
                raise ValueError(f'bad coordinates {row}')
            if precision not in ('hard', 'soft'):
                raise ValueError(f'bad precision {precision}')
            if task not in ('-', 'takeoff', 'land', 'landtakeoff'):
                if not task.startswith('yaw'):
                    raise ValueError(f'bad task {task}')
                try:
                    float(task[3:])
                except ValueError as error:
                    raise ValueError(f'bad yaw task {task}') from error
            mission.append(Waypoint(x, y, z, precision, task))
    if len(mission) < 2 or mission[0].task != 'takeoff' or mission[-1].task != 'land':
        raise ValueError('mission must start with takeoff and end with land')
    if any(w.task == 'takeoff' for w in mission[1:]) or any(w.task == 'land' for w in mission[:-1]):
        raise ValueError('takeoff/land only at first/last row; use landtakeoff in between')
    return mission


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def unwrap_near(angle, reference):
    return reference + wrap(angle - reference)


def yaw_from_quaternion(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))

