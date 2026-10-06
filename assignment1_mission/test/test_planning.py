# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""End-to-end map checks using the supplied hangar and nontrivial routes."""

from pathlib import Path

import pytest

from assignment1_mission.part01.planning import VoxelMap, plan_route


MAP = Path(__file__).parents[2] / 'maps/FEI_LRS_PCD/map.pcd'


@pytest.fixture(scope='module')
def grid():
    return VoxelMap(MAP)


def test_shelf_volume_and_free_space(grid):
    assert not grid.free((2.695, 6.818, 1.5))
    assert not grid.free((5.945, 11.360, 2.5))
    assert grid.free((13.0, 7.0, 1.0))


def test_3d_route_and_shortcuts_remain_clear(grid):
    route = plan_route(grid, (4.84, 5.37, 2.0), (2.08, 9.74, 1.75))
    assert len(route.path) < len(route.raw)
    assert max(point[2] for point in route.path) > 3.76
    assert all(grid.clear_segment(a, b) for a, b in zip(route.path, route.path[1:]))


def test_blocked_goal_fails(grid):
    with pytest.raises(ValueError, match='blocked'):
        plan_route(grid, (13.0, 7.0, 1.0), (2.695, 6.818, 1.5))
