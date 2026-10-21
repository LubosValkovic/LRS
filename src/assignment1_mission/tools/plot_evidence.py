# Copyright (c) 2026 STU FEI URK
# SPDX-License-Identifier: MIT

"""Generate A1.4 figures for exam_example.csv and the default 5 m square."""

import argparse
import json
import math
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import rosbag2_py
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from assignment1_mission.part01.planning import VoxelMap, plan_route
from assignment1_mission.part02.mission import load_mission, yaw_from_quaternion


ROOT = Path(__file__).resolve().parents[3]
POSE = '/mavros/local_position/pose'
TARGET = '/mavros/setpoint_raw/local'
COMPASS = '/mavros/global_position/compass_hdg'


def read_bag(directory, node):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(directory), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    needed = (POSE, TARGET, COMPASS, '/mavros/state', '/rosout')
    classes = {t: get_message(types[t]) for t in needed if t in types}
    data = dict(pose=[], target=[], compass=[], states=[], vehicle=[], paths={})
    latest = {}
    while reader.has_next():
        topic, payload, stamp = reader.read_next()
        if topic in (f'/{node}/actual_path', f'/{node}/target_path'):
            latest[topic.rsplit('/', 1)[-1]] = payload
        if topic not in classes:
            continue
        msg = deserialize_message(payload, classes[topic])
        t = stamp * 1e-9
        if topic == POSE:
            p = msg.pose.position
            data['pose'].append((t, p.x, p.y, p.z, yaw_from_quaternion(msg.pose.orientation)))
        elif topic == TARGET:
            p = msg.position
            data['target'].append((t, p.x, p.y, p.z, msg.yaw))
        elif topic == COMPASS:
            data['compass'].append((t, msg.data))
        elif topic == '/mavros/state':
            data['vehicle'].append((t, msg.armed, msg.connected, msg.mode))
        elif msg.name == node and msg.msg.startswith('state -> '):
            data['states'].append((t, msg.msg.split(' -> ')[1]))
    if not data['states'] or data['states'][-1][1] != 'DONE':
        raise ValueError(f'{directory}: no recorded successful mission completion')
    for key in ('pose', 'target', 'compass'):
        data[key] = np.asarray(data[key], dtype=float)
    path_class = get_message('nav_msgs/msg/Path')
    for key, payload in latest.items():
        msg = deserialize_message(payload, path_class)
        data['paths'][key] = np.asarray([
            (p.header.stamp.sec + p.header.stamp.nanosec * 1e-9,
             p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in msg.poses])
    return data


def stage_at(data, stamps):
    times, names = zip(*data['states'])
    indices = np.searchsorted(times, stamps, side='right') - 1
    return np.asarray(['IDLE', *names], dtype=object)[indices + 1]


def nearest(samples, stamps):
    right = np.searchsorted(samples[:, 0], stamps).clip(0, len(samples) - 1)
    left = (right - 1).clip(0)
    use_left = abs(samples[left, 0] - stamps) < abs(samples[right, 0] - stamps)
    return samples[np.where(use_left, left, right)]


def summary(values):
    values = np.asarray(values)
    if not len(values) or not np.isfinite(values).all():
        raise ValueError('measurement has no finite samples')
    return dict(samples=len(values), median=float(np.median(values)),
                p95=float(np.percentile(values, 95)), maximum=float(np.max(values)),
                rmse=float(np.sqrt(np.mean(values ** 2))))


def save(fig, output, name):
    fig.tight_layout()
    for suffix in ('png', 'pdf'):
        fig.savefig(output / f'{name}.{suffix}', dpi=180)
    plt.close(fig)


def racks(ax, three_d=False):
    for x in (2.695, 5.945):
        for y in (6.818186, 11.3602):
            x0, x1, y0, y1 = x - 1.545, x + 1.545, y - .525, y + .525
            if three_d:
                bottom = [(x0, y0, 0), (x1, y0, 0), (x1, y1, 0), (x0, y1, 0)]
                top = [(a, b, 3.76) for a, b, _ in bottom]
                faces = [bottom, top] + [[bottom[i], bottom[(i+1) % 4],
                                          top[(i+1) % 4], top[i]] for i in range(4)]
                ax.add_collection3d(Poly3DCollection(faces, facecolor='gray',
                                                    edgecolor='gray', alpha=.12))
            else:
                ax.add_patch(Rectangle((x0, y0), 3.09, 1.05, color='gray', alpha=.25))


def planning_figures(grid, mission, output):
    plans, rows = [], []
    for i, (a, b) in enumerate(zip(mission, mission[1:]), 1):
        runs = [plan_route(grid, a.point, b.point) for _ in range(3)]
        plan = runs[-1]
        plans.append(plan)
        rows.append(dict(leg=i, raw_points=len(plan.raw), simplified_points=len(plan.path),
                         median_seconds=float(np.median([p.seconds for p in runs])),
                         length_m=sum(math.dist(a, b) for a, b in zip(plan.path, plan.path[1:])),
                         collision_free=all(grid.clear_segment(a, b)
                                            for a, b in zip(plan.path, plan.path[1:]))))
    fig = plt.figure(figsize=(9, 6))
    ax = fig.add_subplot(111, projection='3d')
    cloud = grid._read_pcd(ROOT / 'maps/FEI_LRS_PCD/map.pcd')
    cloud = cloud[(cloud[:, 2] > .4) & (cloud[:, 2] < 4.8)][::12]
    ax.scatter(*cloud.T, s=.5, c='gray', alpha=.12, label='PCD (floor/roof omitted)')
    racks(ax, True)
    for i, plan in enumerate(plans):
        ax.plot(*np.asarray(plan.raw).T, color='tab:orange', lw=1, alpha=.65,
                label='Raw A*' if i == 0 else None)
        ax.plot(*np.asarray(plan.path).T, color='tab:blue', marker='o', ms=3,
                label='Collision-checked shortcuts' if i == 0 else None)
    ax.set(xlabel='Map X (m)', ylabel='Map Y (m)', zlabel='Map Z (m)',
           title='Indoor planning: 0.25 m voxels, 0.45 m inflation')
    ax.set_box_aspect((19, 15, 8))
    ax.view_init(elev=28, azim=-62)
    ax.legend(fontsize=8, loc='upper left')
    save(fig, output, 'planning_3d')
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    legs = np.arange(1, len(rows) + 1)
    axes[0].bar(legs - .18, [r['raw_points'] for r in rows], .36, label='Raw A*')
    axes[0].bar(legs + .18, [r['simplified_points'] for r in rows], .36, label='Simplified')
    axes[0].set(xlabel='Mission leg', ylabel='Number of path points', xticks=legs)
    axes[0].legend()
    axes[1].bar(legs, [1000*r['median_seconds'] for r in rows], color='tab:green')
    axes[1].set(xlabel='Mission leg', ylabel='Planning time (ms)', xticks=legs,
                title='Median of 3 runs; PCD loading excluded')
    for ax in axes:
        ax.grid(axis='y', alpha=.25)
        ax.set_axisbelow(True)
    save(fig, output, 'planning_stats')
    return plans, rows


def flight_figures(data, output, kind, mission=None, plans=None, grid=None):
    start, end = data['states'][0][0], data['states'][-1][0]
    pose = data['pose'][(data['pose'][:, 0] >= start) & (data['pose'][:, 0] <= end)].copy()
    target = data['target'].copy()
    if kind == 'indoor':
        launch = np.median(data['pose'][data['pose'][:, 0] < start + .2, 1:4], axis=0)
        for points in (pose, target):
            delta = points[:, 1:4] - launch
            points[:, 1:4] = np.column_stack((13 + delta[:, 1], 7 - delta[:, 0], delta[:, 2]))
        actual, commanded = pose[:, :4], target[:, :4]
        nav_mask = stage_at(data, pose[:, 0]) == 'NAVIGATE'
        nav = pose[nav_mask]
        blocked = np.asarray([not grid.free(p) for p in nav[:, 1:4]])
    else:
        actual, commanded = data['paths']['actual_path'], data['paths']['target_path']
        if actual.shape != commanded.shape or not np.allclose(actual[:, 0], commanded[:, 0], rtol=0, atol=1e-6):
            raise ValueError('outdoor path histories must have matching timestamps')
        launch = pose[0, 1:4]
        centre = data['target'][stage_at(data, data['target'][:, 0]) == 'EIGHT_ALIGN']
        if not len(centre):
            raise ValueError('outdoor recording lacks the figure-eight centre alignment')
        poi = np.median(centre[:, 1:3], axis=0)
    fig = plt.figure(figsize=(11, 4.8))
    xy = fig.add_subplot(121)
    xyz = fig.add_subplot(122, projection='3d')
    if kind == 'indoor':
        racks(xy)
        racks(xyz, True)
        for i, plan in enumerate(plans):
            route = np.asarray(plan.path)
            xy.plot(*route[:, :2].T, '--', c='gray', lw=1, label='Planned route' if i == 0 else None)
            xyz.plot(*route.T, '--', c='gray', lw=1)
        for i, wp in enumerate(mission):
            xy.plot(wp.x, wp.y, 'ko', ms=3)
            xy.annotate(f'W{i}', (wp.x, wp.y), xytext=(4, 4), textcoords='offset points', fontsize=8)
    else:
        xy.plot(*poi, 'k*', ms=10, label='POI')
    for values, color, label in ((commanded, 'tab:orange', 'Commanded'),
                                  (actual, 'tab:blue', 'Measured')):
        xy.plot(values[:, 1], values[:, 2], c=color, lw=1.2, label=label)
        xyz.plot(*values[:, 1:4].T, c=color, lw=1.2)
    if kind == 'indoor' and blocked.any():
        xy.scatter(nav[blocked, 1], nav[blocked, 2], c='tab:red', marker='x', s=24,
                   label='Inside inflated cells', zorder=5)
    frame = 'Map' if kind == 'indoor' else 'MAVROS ENU'
    xy.set(xlabel=f'{frame} X (m)', ylabel=f'{frame} Y (m)', title=f'{kind.title()} ground track')
    xy.set_aspect('equal', adjustable='box')
    xy.grid(alpha=.25)
    xy.legend(fontsize=8)
    xyz.set(xlabel='X (m)', ylabel='Y (m)', zlabel='Z (m)', title='3D trajectory')
    if kind == 'outdoor':
        xyz.set_zlim(0, 6)
    xyz.set_box_aspect((1, 1, .65))
    save(fig, output, f'{kind}_track')

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(pose[:, 0] - start, pose[:, 3], label='Local pose', lw=1.2)
    axes[0].scatter(commanded[:, 0] - start, commanded[:, 3], s=2, c='tab:orange', label='Setpoint')
    axes[0].set_ylabel('Altitude (m)')
    axes[0].legend(fontsize=8)
    reference = nearest(commanded, actual[:, 0])
    errors = np.linalg.norm(actual[:, 1:4] - reference[:, 1:4], axis=1)
    valid = abs(actual[:, 0] - reference[:, 0]) < .15
    metrics = dict(duration_s=end-start, final_state='DONE',
                   final_armed=bool(data['vehicle'][-1][1]),
                   states=[dict(time_s=t-start, state=s) for t, s in data['states']],
                   setpoint_distance_m=summary(errors[valid]))
    if kind == 'outdoor':
        axes[1].plot(actual[valid, 0] - start, errors[valid], lw=1)
        axes[1].set_ylabel('Distance to active\nsetpoint (m)')
        heading = nearest(data['compass'], actual[:, 0])
        measured_yaw = np.pi/2 - np.radians(heading[:, 1])
        states = stage_at(data, actual[:, 0])
        desired_yaw = nearest(data['target'], actual[:, 0])[:, 4]
        # Points near the POI have undefined bearing.
        bearing = np.arctan2(poi[1] - actual[:, 2], poi[0] - actual[:, 1])
        square = (states == 'OUT_SQUARE') & (np.linalg.norm(actual[:, 1:3] - poi, axis=1) > .5)
        eight = states == 'EIGHT'
        fresh = abs(heading[:, 0] - actual[:, 0]) < .3
        for name, mask, wanted in (('square', square & fresh, bearing),
                                    ('figure_eight', eight & fresh, desired_yaw)):
            error = np.degrees(np.arctan2(np.sin(measured_yaw - wanted), np.cos(measured_yaw - wanted)))
            axes[2].plot(actual[mask, 0] - start, error[mask], '.', ms=2, label=name.replace('_', ' '))
            metrics[f'{name}_yaw_error_deg'] = summary(abs(error[mask]))
            metrics[f'{name}_tracking_m'] = summary(errors[mask & valid])
        metrics['maximum_radius_from_poi_m'] = float(np.linalg.norm(actual[:, 1:3] - poi, axis=1).max())
        metrics['figure_eight_duration_s'] = next(t for t, s in data['states'] if s == 'OUT_RETURN') - next(
            t for t, s in data['states'] if s == 'EIGHT')
        corners = poi + np.asarray([[-2.5, 2.5], [2.5, 2.5], [2.5, -2.5], [-2.5, -2.5]])
        visits = []
        for point in commanded[states == 'OUT_SQUARE', 1:3]:
            distances = np.linalg.norm(corners - point, axis=1)
            corner = int(np.argmin(distances)) + 1
            if distances[corner-1] < .1 and (not visits or corner != visits[-1]):
                visits.append(corner)
        metrics['square_corner_order'] = visits
        if visits != [1, 2, 3, 4, 1]:
            raise ValueError(f'square did not close all four edges: {visits}')
        axes[2].set_ylabel('Compass heading\nerror (deg)')
        axes[2].legend(fontsize=8)
    else:
        segments = np.asarray([(a, b) for plan in plans for a, b in zip(plan.path, plan.path[1:])])
        offsets = nav[:, None, 1:4] - segments[None, :, 0, :]
        vectors = segments[:, 1, :] - segments[:, 0, :]
        fraction = (np.sum(offsets * vectors, axis=2) / np.sum(vectors*vectors, axis=1)).clip(0, 1)
        distances = np.linalg.norm(offsets - fraction[:, :, None]*vectors, axis=2).min(axis=1)
        series = np.full(len(pose), np.nan)
        series[nav_mask] = distances
        axes[1].plot(pose[:, 0] - start, series, lw=1)
        axes[1].set_ylabel('Distance to nearest\nplanned segment (m)')
        metrics['navigation_path_distance_m'] = summary(distances)
        metrics['navigation_clearance'] = dict(samples=len(nav),
            inside_inflated_voxels=int(blocked.sum()),
            inside_occupied_voxels=sum(bool(grid.occupied[grid.index(p)]) for p in nav[:, 1:4]))
        pose[:, 4] = np.unwrap(pose[:, 4])
        axes[2].plot(pose[:, 0] - start, np.degrees(pose[:, 4]), label='Measured', lw=1)
        wanted = nearest(pose, target[:, 0])[:, 4]
        wanted += np.arctan2(np.sin(target[:, 4] - wanted), np.cos(target[:, 4] - wanted))
        axes[2].scatter(target[:, 0] - start, np.degrees(wanted), s=2, label='Setpoint', c='tab:orange')
        axes[2].set_ylabel('ENU yaw (deg)')
        axes[2].legend(fontsize=8)
        metrics['waypoint_arrivals'] = []
        for (t, _), wp in zip((s for s in data['states'] if s[1] == 'TASK'), mission[1:]):
            at = nearest(pose, np.asarray([t]))[0]
            metrics['waypoint_arrivals'].append(dict(task=wp.task, precision=wp.precision,
                error_m=float(np.linalg.norm(at[1:4] - wp.point))))
    for ax in axes:
        ax.grid(alpha=.25)
        ax.set_xlim(0, end-start)
    for t, state in data['states']:
        if state in ('TAKEOFF', 'LAND', 'OUT_SQUARE', 'EIGHT', 'OUT_RETURN'):
            axes[0].axvline(t-start, color='gray', lw=.6, alpha=.5)
            axes[0].text(t-start, 1.01, state, rotation=90, fontsize=6,
                         transform=axes[0].get_xaxis_transform(), va='bottom')
    axes[2].set_xlabel('Time since mission start (s)')
    fig.suptitle(f'{kind.title()} flight telemetry (recorded simulation)')
    save(fig, output, f'{kind}_tracking')
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--indoor', type=Path, required=True)
    parser.add_argument('--outdoor', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({'font.size': 9, 'axes.spines.top': False, 'axes.spines.right': False})
    mission = load_mission(ROOT / 'missions/exam_example.csv')
    grid = VoxelMap(ROOT / 'maps/FEI_LRS_PCD/map.pcd', .25, .45)
    plans, rows = planning_figures(grid, mission, args.output)
    indoor = read_bag(args.indoor, 'assignment1_indoor')
    outdoor = read_bag(args.outdoor, 'assignment1_outdoor')
    metrics = dict(planning=rows,
                   indoor=flight_figures(indoor, args.output, 'indoor', mission, plans, grid),
                   outdoor=flight_figures(outdoor, args.output, 'outdoor'))
    (args.output / 'metrics.json').write_text(json.dumps(metrics, indent=2) + '\n')
    print(json.dumps(metrics, indent=2))


if __name__ == '__main__':
    main()
