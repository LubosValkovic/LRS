# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""End-to-end map checks using the supplied hangar and nontrivial routes."""

from pathlib import Path
import heapq
import itertools
import math

import numpy as np
import pytest

from assignment1_mission.part01.planning import VoxelMap, plan_route


MAP = Path(__file__).resolve().parents[3] / 'maps/FEI_LRS_PCD/map.pcd'


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


def small_grid(shape=(7, 7, 3)):
    grid = VoxelMap.__new__(VoxelMap)
    grid.minimum = np.zeros(3)
    grid.resolution = 1.0
    grid.blocked = np.zeros(shape, dtype=bool)
    return grid


def intersects_box(start, end, low, high):
    """Independent closed-box/segment intersection using slab intervals."""
    entry, exit = 0.0, 1.0
    for p, q, a, b in zip(start, end, low, high):
        if p == q:
            if not a <= p <= b:
                return False
        else:
            first, last = sorted(((a - p) / (q - p), (b - p) / (q - p)))
            entry, exit = max(entry, first), min(exit, last)
            if entry > exit:
                return False
    return True


def reference_clear(grid, start, end):
    return not any(intersects_box(start, end, cell, cell + 1)
                   for cell in np.argwhere(grid.blocked))


def test_rays_match_independent_box_intersections():
    grid = small_grid((6, 6, 6))
    rng = np.random.default_rng(17)
    grid.blocked = rng.random(grid.blocked.shape) < 0.15
    rays = [(rng.uniform(0.1, 5.9, 3), rng.uniform(0.1, 5.9, 3)) for _ in range(100)]
    # Rays on voxel faces and edges must check both sides of each plane.
    rays += [(np.array([2., 2., .5]), np.array([2., 2., 5.5])),
             (np.array([1.5, 3., 1.5]), np.array([4.5, 3., 4.5])),
             (np.array([1.5, 1.5, 1.5]), np.array([4.5, 4.5 + 1e-7, 4.5]))]
    for start, end in rays:
        expected = reference_clear(grid, start, end)
        assert grid.clear_segment(start, end) == expected
        assert grid.clear_segment(end, start) == expected


def test_astar_matches_dijkstra_with_exact_endpoint_cost():
    grid = small_grid()
    grid.blocked[3, :6, :2] = True
    start, goal = (.65, .6, .55), (5.4, 5.4, 1.2)
    source = grid.index(start)
    queue = [(math.dist(start, grid.centre(source)), source)]
    costs = {}
    optimum = math.inf
    offsets = [v for v in itertools.product((-1, 0, 1), repeat=3) if any(v)]
    while queue:
        cost, cell = heapq.heappop(queue)
        if cell in costs:
            continue
        costs[cell] = cost
        centre = grid.centre(cell)
        distance = math.dist(centre, goal)
        if distance <= 1.25 and reference_clear(grid, centre, goal):
            optimum = min(optimum, cost + distance)
        for shift in offsets:
            nxt = tuple(a + b for a, b in zip(cell, shift))
            if grid.free_index(nxt) and nxt not in costs:
                nxt_centre = grid.centre(nxt)
                if reference_clear(grid, centre, nxt_centre):
                    heapq.heappush(queue, (cost + math.dist(centre, nxt_centre), nxt))
    route = plan_route(grid, start, goal)
    assert sum(math.dist(a, b) for a, b in zip(route.raw, route.raw[1:])) == pytest.approx(optimum)


def test_disconnected_grid_reports_no_path():
    grid = small_grid()
    grid.blocked[3, :, :] = True
    with pytest.raises(ValueError, match='no collision-free path'):
        plan_route(grid, (.5, .5, .5), (5.5, 5.5, .5))


def test_coarse_resolution_still_connects_exact_goal():
    grid = small_grid((6, 6, 3))
    grid.resolution = 4.
    grid.blocked[2, :3, :] = True
    goal = (17.9, 4.1, 6.1)
    route = plan_route(grid, (2., 2., 6.), goal)
    assert route.path[-1] == goal
    assert all(grid.clear_segment(a, b) for a, b in zip(route.path, route.path[1:]))


def test_single_point_pcd_and_racks_outside_map(tmp_path):
    pcd = tmp_path / 'single.pcd'
    pcd.write_text('# PCD\n\nFIELDS x y z\nPOINTS 1\nDATA ascii\n100 100 100\n')
    grid = VoxelMap(pcd, safety_radius=0)
    assert int(grid.occupied.sum()) == 1
    assert not grid.free((100, 100, 100))


@pytest.mark.parametrize('resolution,radius', [(float('nan'), .45), (.25, float('inf')), (0, .45)])
def test_invalid_grid_parameters_are_rejected(resolution, radius):
    with pytest.raises(ValueError, match='finite'):
        VoxelMap(MAP, resolution, radius)


@pytest.mark.parametrize('point', [(float('nan'), 1., 1.), (1., 1.), (1., 1., float('inf'))])
def test_invalid_planner_endpoints_are_rejected(grid, point):
    with pytest.raises(ValueError, match='three finite'):
        plan_route(grid, point, (13., 7., 1.))
