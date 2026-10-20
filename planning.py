# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""PCD voxel map, three dimensional A* and collision checked shortcuts."""

import heapq
import itertools
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation


# A centre-to-centre diagonal touches every nonempty subset of its axes.
# Precompute these cells once instead of ray tracing all 26 moves during A*.
NEIGHBOURS = []
for offset in itertools.product((-1, 0, 1), repeat=3):
    axes = [axis for axis, value in enumerate(offset) if value]
    if axes:
        touched = tuple(tuple(offset[axis] if axis in subset else 0 for axis in range(3))
                        for size in range(1, len(axes) + 1)
                        for subset in itertools.combinations(axes, size))
        NEIGHBOURS.append((offset, math.sqrt(len(axes)), touched))
NEIGHBOURS = tuple(NEIGHBOURS)


@dataclass
class Plan:
    raw: list
    path: list
    seconds: float


class VoxelMap:
    def __init__(self, pcd, resolution=0.25, safety_radius=0.45):
        if not math.isfinite(resolution) or not math.isfinite(safety_radius) or resolution <= 0 or safety_radius < 0:
            raise ValueError('resolution must be finite and positive; radius finite and nonnegative')
        self.resolution = float(resolution)
        self.safety_radius = float(safety_radius)
        points = self._read_pcd(pcd)
        self.minimum = np.floor(points.min(axis=0) / resolution) * resolution - resolution
        self.maximum = np.ceil(points.max(axis=0) / resolution) * resolution + resolution
        shape = np.ceil((self.maximum - self.minimum) / resolution).astype(int) + 1
        self.occupied = np.zeros(tuple(shape), dtype=bool)
        indices = np.floor((points - self.minimum) / resolution).astype(int)
        self.occupied[tuple(indices.T)] = True
        self._fill_racks()
        cells = int(math.ceil(safety_radius / resolution))
        offsets = np.indices((2 * cells + 1,) * 3).transpose(1, 2, 3, 0) - cells
        ball = np.sum((offsets * resolution) ** 2, axis=-1) <= (safety_radius + 1e-9) ** 2
        self.blocked = binary_dilation(self.occupied, structure=ball)

    @staticmethod
    def _read_pcd(path):
        path = Path(path)
        with path.open('rb') as stream:
            header = {}
            while True:
                raw = stream.readline()
                if not raw:
                    raise ValueError('incomplete PCD header')
                line = raw.decode('ascii').strip()
                if not line or line.startswith('#'):
                    continue
                key, *values = line.split()
                header[key] = values
                if key == 'DATA':
                    break
            if header.get('DATA') != ['ascii']:
                raise ValueError('this loader needs an ASCII PCD')
            try:
                xyz = [header['FIELDS'].index(axis) for axis in ('x', 'y', 'z')]
                count = int(header['POINTS'][0])
            except (KeyError, IndexError, ValueError) as error:
                raise ValueError('PCD needs XYZ fields and a valid POINTS count') from error
            if count <= 0:
                raise ValueError('empty PCD')
            cloud = np.loadtxt(stream, usecols=xyz, dtype=np.float64, ndmin=2)
        if len(cloud) != count:
            raise ValueError('PCD POINTS count does not match data')
        cloud = cloud[np.isfinite(cloud).all(axis=1)]
        if not len(cloud):
            raise ValueError('empty PCD')
        return cloud

    def _fill_racks(self):
        # The current Gazebo SDF has four racks. Their open visual meshes and
        # shelves are conservatively modelled as solid 3.09 x 1.05 x 3.76 m boxes.
        # This prevents a route through a shelf opening or between shelf levels.
        for x in (2.695, 5.945):
            for y in (6.818186, 11.3602):
                low = np.array([x - 1.545, y - 0.525, 0.0])
                high = np.array([x + 1.545, y + 0.525, 3.76])
                a = np.maximum(self.index(low), 0)
                b = np.minimum(self.index(high), np.asarray(self.occupied.shape) - 1)
                if np.any(a > b):
                    continue
                self.occupied[a[0]:b[0] + 1, a[1]:b[1] + 1, a[2]:b[2] + 1] = True

    def index(self, point):
        if len(point) != 3 or not all(math.isfinite(v) for v in point):
            raise ValueError('point must contain three finite coordinates')
        return tuple(math.floor((point[d] - self.minimum[d]) / self.resolution) for d in range(3))

    def centre(self, index):
        return tuple(float(self.minimum[d]) + (index[d] + 0.5) * self.resolution for d in range(3))

    def free_index(self, index):
        x, y, z = index
        nx, ny, nz = self.blocked.shape
        return 0 <= x < nx and 0 <= y < ny and 0 <= z < nz and not self.blocked[x, y, z]

    def free(self, point):
        return self.free_index(self.index(point))

    def clear_segment(self, start, end):
        """Conservative voxel supercover ray, including diagonal edge touches."""
        # Validate before converting NaN/Inf to voxel indices.
        cell = list(self.index(start))
        last = self.index(end)
        p = tuple((start[d] - self.minimum[d]) / self.resolution for d in range(3))
        q = tuple((end[d] - self.minimum[d]) / self.resolution for d in range(3))
        direction = tuple(q[d] - p[d] for d in range(3))

        def boundary_axes(point):
            return [d for d in range(3) if abs(point[d] - round(point[d])) <= 1e-12]

        def touching_free(index, axes):
            for bits in range(1 << len(axes)):
                adjacent = list(index)
                for bit, axis in enumerate(axes):
                    if bits & (1 << bit):
                        adjacent[axis] -= 1
                if not self.free_index(tuple(adjacent)):
                    return False
            return True

        if not touching_free(cell, boundary_axes(p)) or not touching_free(last, boundary_axes(q)):
            return False
        stationary = [d for d in boundary_axes(p) if direction[d] == 0]
        step = tuple(1 if v > 0 else -1 if v < 0 else 0 for v in direction)
        delta = [math.inf] * 3
        crossing = [math.inf] * 3
        for axis in range(3):
            if step[axis]:
                delta[axis] = abs(1.0 / direction[axis])
                boundary = cell[axis] + (1 if step[axis] > 0 else 0)
                crossing[axis] = (boundary - p[axis]) / direction[axis]
            if cell[axis] == last[axis]:
                crossing[axis] = math.inf
        while tuple(cell) != last:
            t = min(crossing)
            axes = [d for d in range(3) if math.isclose(crossing[d], t, rel_tol=0, abs_tol=1e-12)]
            # All cells touching a grid edge/corner must be free.
            for bits in range(1, 1 << len(axes)):
                candidate = list(cell)
                for bit, axis in enumerate(axes):
                    if bits & (1 << bit):
                        candidate[axis] += step[axis]
                if not touching_free(candidate, stationary):
                    return False
            for axis in axes:
                cell[axis] += step[axis]
                crossing[axis] += delta[axis]
                if cell[axis] == last[axis]:
                    crossing[axis] = math.inf
        return True


def plan_route(grid, start, goal):
    start = tuple(float(v) for v in start)
    goal = tuple(float(v) for v in goal)
    if not grid.free(start) or not grid.free(goal):
        raise ValueError(f'start or goal blocked: {start}, {goal}')
    begin = time.monotonic()
    source = grid.index(start)
    if grid.clear_segment(start, goal):
        raw = [start, goal]
    else:
        def heuristic(node):
            return math.dist(grid.centre(node), goal)

        source_centre = grid.centre(source)
        if not grid.clear_segment(start, source_centre):
            raise ValueError('start does not connect to its voxel centre')
        initial_cost = math.dist(start, source_centre)
        queue = [(initial_cost + heuristic(source), initial_cost, source)]
        cost = {source: initial_cost}
        parent = {}
        goal_cost = math.inf
        goal_parent = None
        goal_radius = max(1.25, math.sqrt(3) * grid.resolution / 2)
        while queue:
            estimate, current_cost, current = heapq.heappop(queue)
            if current_cost > cost[current] + 1e-9:
                continue
            if estimate >= goal_cost:
                break
            distance = heuristic(current)
            if distance <= goal_radius and grid.clear_segment(grid.centre(current), goal):
                candidate_cost = current_cost + distance
                if candidate_cost < goal_cost:
                    goal_cost, goal_parent = candidate_cost, current
            for offset, length, touched in NEIGHBOURS:
                nxt = tuple(current[d] + offset[d] for d in range(3))
                if not grid.free_index(nxt):
                    continue
                tentative = current_cost + length * grid.resolution
                if tentative + 1e-9 < cost.get(nxt, math.inf):
                    if any(not grid.free_index(tuple(current[d] + shift[d] for d in range(3)))
                           for shift in touched):
                        continue
                    cost[nxt] = tentative
                    parent[nxt] = current
                    heapq.heappush(queue, (tentative + heuristic(nxt), tentative, nxt))
        if goal_parent is None:
            raise ValueError(f'no collision-free path from {start} to {goal}')
        nodes = [goal_parent]
        while nodes[-1] != source:
            nodes.append(parent[nodes[-1]])
        nodes.reverse()
        raw = [start] + [grid.centre(n) for n in nodes] + [goal]
    # A centre-to-centre route may not reach the exact endpoint safely.
    if any(not grid.clear_segment(a, b) for a, b in zip(raw, raw[1:])):
        raise ValueError('grid route does not connect to exact endpoint')
    short = [raw[0]]
    index = 0
    while index < len(raw) - 1:
        farthest = len(raw) - 1
        while farthest > index + 1 and not grid.clear_segment(raw[index], raw[farthest]):
            farthest -= 1
        short.append(raw[farthest])
        index = farthest
    if any(not grid.clear_segment(a, b) for a, b in zip(short, short[1:])):
        raise ValueError('simplified route failed collision check')
    return Plan(raw, short, time.monotonic() - begin)
