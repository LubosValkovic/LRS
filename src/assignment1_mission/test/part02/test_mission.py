# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""CSV validation and yaw wrapping regressions."""

import math

import pytest

from assignment1_mission.part02.mission import load_mission, unwrap_near


@pytest.mark.parametrize('row', ['1,2,3,hard,yawnan', '1,2,3,hard,yawinf',
                               '1,2,3,hard', '1,2,3,hard,-,extra', 'nan,2,3,hard,-'])
def test_invalid_mission_rows_are_rejected(tmp_path, row):
    path = tmp_path / 'mission.csv'
    path.write_text('x,y,z,precision,task\n1,2,1,soft,takeoff\n' + row + '\n1,2,1,hard,land\n')
    with pytest.raises(ValueError):
        load_mission(path)


def test_commented_mission_loads(tmp_path):
    path = tmp_path / 'mission.csv'
    path.write_text('# mission\nx,y,z,precision,task\n1,2,1,soft,takeoff\n'
                    '\n1,2,2,hard,yaw180\n1,2,1,hard,land\n')
    assert [w.task for w in load_mission(path)] == ['takeoff', 'yaw180', 'land']


def test_heading_uses_short_rotation_across_pi():
    actual, desired = math.radians(179), math.radians(-179)
    assert unwrap_near(desired, actual) - actual == pytest.approx(math.radians(2))
