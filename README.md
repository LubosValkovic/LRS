# Assignment 1 — Part 1: 3D map and path planning

This branch contains only A1.1: loading the hangar point cloud into a 3D voxel
grid, obstacle inflation, 3D A* planning, and collision-checked path
shortening.

## Method

- Input: `maps/FEI_LRS_PCD/map.pcd` (ASCII PCD).
- Map: dense Boolean 3D voxel grid, default resolution `0.25 m`.
- Racks: four known shelf volumes are conservatively filled as solid boxes.
- Clearance: spherical 3D dilation with default safety radius `0.45 m`.
- Planner: 3D A* with 26 neighbours, Euclidean edge cost and heuristic.
- Post-processing: greedy farthest-visible shortcuts checked by a voxel
  supercover ray.

## Build

From the repository root:

```bash
sudo apt install python3-numpy python3-scipy
colcon build --packages-select assignment1_mission
source install/setup.bash
```

## Run

Gazebo, SITL, and MAVROS are not needed for Part 1.

```bash
ros2 run assignment1_mission plan3d \
  --map maps/FEI_LRS_PCD/map.pcd \
  --start 4.84 5.37 2.0 \
  --goal 2.08 9.74 1.75 \
  --resolution 0.25 \
  --safety-radius 0.45
```

The command prints map bounds, occupied and inflated voxel counts, planning
time, raw and simplified point counts, and the final path.

## Test

```bash
pytest -q assignment1_mission/test/test_planning.py
```

The tests verify filled rack occupancy, a genuinely 3D collision-free route,
safe shortened segments, and rejection of a blocked goal.

## Source layout

```text
assignment1_mission/
├── assignment1_mission/part01/
│   ├── planning.py   # PCD, voxel grid, inflation, A*, shortcuts
│   └── plan_cli.py   # standalone command
├── test/test_planning.py
├── package.xml
└── setup.py
```
