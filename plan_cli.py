# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Standalone planner command, independent of Gazebo and ROS graph."""

import argparse

from .planning import VoxelMap, plan_route


def main():
    parser = argparse.ArgumentParser(description='Plan a collision-free 3D route through the hangar PCD')
    parser.add_argument('--map', required=True, help='ASCII PCD map')
    parser.add_argument('--start', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--goal', nargs=3, type=float, required=True, metavar=('X', 'Y', 'Z'))
    parser.add_argument('--resolution', type=float, default=0.25)
    parser.add_argument('--safety-radius', type=float, default=0.45)
    args = parser.parse_args()
    grid = VoxelMap(args.map, args.resolution, args.safety_radius)
    print(f'bounds: {grid.minimum.tolist()} .. {grid.maximum.tolist()}')
    print(f'occupied: {int(grid.occupied.sum())}, inflated: {int(grid.blocked.sum())} voxels')
    print(f'start free: {grid.free(args.start)}, goal free: {grid.free(args.goal)}')
    route = plan_route(grid, args.start, args.goal)
    print(f'planning: {route.seconds:.3f} s; {len(route.raw)} raw -> {len(route.path)} simplified points')
    for point in route.path:
        print(','.join(f'{v:.3f}' for v in point))


if __name__ == '__main__':
    main()
