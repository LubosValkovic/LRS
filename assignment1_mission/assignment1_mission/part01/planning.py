# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""PCD voxel map, three dimensional A* and collision checked shortcuts."""

import heapq
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation


@dataclass
class Plan:
    raw: list
    path: list
    seconds: float


class VoxelMap:
    def __init__(self, pcd, resolution=0.25, safety_radius=0.45):
        if resolution <= 0 or safety_radius < 0:
            raise ValueError('resolution must be positive and radius nonnegative')
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
                line = stream.readline().decode('ascii').strip()
                if not line:
                    raise ValueError('incomplete PCD header')
                key, *values = line.split()
                header[key] = values
                if key == 'DATA':
                    break
            if header['DATA'][0] != 'ascii':
                raise ValueError('this loader needs an ASCII PCD')
            fields = header['FIELDS']
            xyz = [fields.index(axis) for axis in ('x', 'y', 'z')]
            cloud = np.loadtxt(stream, usecols=xyz, dtype=np.float64)
        if len(cloud) != int(header['POINTS'][0]):
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
                a = self.index(low)
                b = self.index(high)
                self.occupied[a[0]:b[0] + 1, a[1]:b[1] + 1, a[2]:b[2] + 1] = True

    def index(self, point):
        return tuple(np.floor((np.asarray(point) - self.minimum) / self.resolution).astype(int))

    def centre(self, index):
        return tuple(self.minimum + (np.asarray(index) + 0.5) * self.resolution)

    def free_index(self, index):
        return all(0 <= index[d] < self.blocked.shape[d] for d in range(3)) and not self.blocked[index]

    def free(self, point):
        return self.free_index(self.index(point))

    def clear_segment(self, start, end):
        """Conservative voxel supercover ray, including diagonal edge touches."""
        p = (np.asarray(start) - self.minimum) / self.resolution
        q = (np.asarray(end) - self.minimum) / self.resolution
        direction = q - p
        cell = np.floor(p).astype(int)
        last = np.floor(q).astype(int)
        if not self.free_index(tuple(cell)) or not self.free_index(tuple(last)):
            return False
        step = np.sign(direction).astype(int)
        delta = np.full(3, np.inf)
        crossing = np.full(3, np.inf)
        for axis in range(3):
            if step[axis]:
                delta[axis] = abs(1.0 / direction[axis])
                boundary = cell[axis] + (1 if step[axis] > 0 else 0)
                crossing[axis] = (boundary - p[axis]) / direction[axis]
        while not np.array_equal(cell, last):
            t = crossing.min()
            axes = np.flatnonzero(np.isclose(crossing, t, atol=1e-10))
            # All cells touching a grid edge/corner must be free.
            for bits in range(1, 1 << len(axes)):
                candidate = cell.copy()
                for bit, axis in enumerate(axes):
                    if bits & (1 << bit):
                        candidate[axis] += step[axis]
                if not self.free_index(tuple(candidate)):
                    return False
            for axis in axes:
                cell[axis] += step[axis]
                crossing[axis] += delta[axis]
        return True


def plan_route(grid, start, goal):
    start = tuple(float(v) for v in start)
    goal = tuple(float(v) for v in goal)
    if not grid.free(start) or not grid.free(goal):
        raise ValueError(f'start or goal blocked: {start}, {goal}')
    begin = time.monotonic()
    source, target = grid.index(start), grid.index(goal)
    if grid.clear_segment(start, goal):
        raw = [start, goal]
    else:
        neighbours = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1)
                      for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)]
        heuristic = lambda node: math.dist(grid.centre(node), goal)
        queue = [(heuristic(source), 0.0, source)]
        cost = {source: 0.0}
        parent = {}
        while queue:
            _, current_cost, current = heapq.heappop(queue)
            if current_cost > cost[current] + 1e-9:
                continue
            if heuristic(current) <= 1.25 and grid.clear_segment(grid.centre(current), goal):
                break
            for offset in neighbours:
                nxt = tuple(current[d] + offset[d] for d in range(3))
                if not grid.free_index(nxt):
                    continue
                # Reject diagonals which cut through an occupied voxel corner.
                if not grid.clear_segment(grid.centre(current), grid.centre(nxt)):
                    continue
                tentative = current_cost + math.sqrt(sum(v * v for v in offset)) * grid.resolution
                if tentative + 1e-9 < cost.get(nxt, math.inf):
                    cost[nxt] = tentative
                    parent[nxt] = current
                    heapq.heappush(queue, (tentative + heuristic(nxt), tentative, nxt))
        else:
            raise ValueError(f'no collision-free path from {start} to {goal}')
        nodes = [current]
        while nodes[-1] != source:
            nodes.append(parent[nodes[-1]])
        nodes.reverse()
        raw = [start] + [grid.centre(n) for n in nodes[1:]] + [goal]
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
