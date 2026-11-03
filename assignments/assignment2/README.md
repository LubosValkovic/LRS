# Assignment 2 — Pick one topic (10 points)

**Deadline: end of week 12** (the last exercise). See [`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md) for the
semester rules, the late-submission penalty and the 56 % threshold.

## How it works

There are **three topics**. You choose **exactly one** of them and build it. All three are worth
the same **10 points** and only the topic you picked is scored — doing a second one earns
nothing extra.

All three build on the control node you wrote for [Assignment 1](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/README.md) —
you are extending your own work, not starting over. Assignment 2 is deliberately smaller than
Assignment 1: it is one focused capability added to a system that already flies, not a second
full project.

Tell your exercise teacher which topic you picked **in week 7**. Changing your mind later is
possible but the deadline does not move.

## Topics

Each topic has its own file with the full specification, the point breakdown and the acceptance
criteria.

| Topic | What you build | Points | File |
|---|---|---|---|
| **Obstacle avoidance** | Detect an unknown obstacle from the depth camera's point cloud and fly around it on a straight A → B flight | **10** | [`obstacle-avoidance.md`](obstacle-avoidance.md) |
| **Drone localisation** | Fuse noisy GPS, drifting visual odometry and a biased gyro in your own eight-state Kalman filter, then fly a mission on that estimate instead of MAVROS's position | **10** | [`localization.md`](localization.md) |
| **Find-and-follow a moving rover** | Two drones, each searching for its own ArUco-marked rover with the down camera and following it as it moves | **10** | [`rover-find-and-follow.md`](rover-find-and-follow.md) |

What each topic runs on, to help you choose:

| | Obstacle avoidance | Drone localisation | Find-and-follow |
|---|---|---|---|
| **World** | `fei_lrs_gazebo_depth.world` | `fei_lrs_gazebo_localization.world` | `fei_lrs_precision_landing.world` |
| **Vehicles** | 1 drone | 1 drone | 2 drones + 2 rovers |
| **Bring-up** | 1 × SITL, 1 × MAVROS | 1 × SITL, 1 × MAVROS + the supplied [`localization_sensor_sim`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/localization_sensor_sim/README.md) package | 2 × SITL, 2 × MAVROS in separate namespaces + the supplied `scripts/randomize_rovers.py` |
| **New input** | Depth camera point cloud, via `ros_gz_bridge` | GPS, VO and gyro on `/localization/*` | Down cameras, via `ros_gz_bridge` |
| **The new part is** | Perception and replanning | State estimation | Multi-vehicle, computer vision and tracking |

The localisation topic has one rule of its own worth knowing before you pick it: your estimator
and controller may **not** use ground truth or any MAVROS position/attitude output as feedback —
see [`localization.md`](localization.md#inputs-you-may-and-may-not-use).

## Point breakdown

### Obstacle avoidance — 10 points

| Section | Points |
|---|---|
| §1 Point cloud subscriber | 2.0 |
| §2 Obstacle detection — stable, correctly positioned | 3.0 |
| §3 Path replanning in 3D — a real avoidance algorithm, around or over | 2.5 |
| §4 Flight with replanning — autonomous A → B around the obstacle | 2.5 |

### Drone localisation — 10 points

| Section | Points |
|---|---|
| §1 Sensor acquisition and characterisation | 1.5 |
| §2 Simplified eight-state KF/EKF | 4.5 |
| §3 Missing and bad measurements — GPS/VO outages, outlier gating | 2.0 |
| §4 Flight using estimated localisation | 2.0 |

### Find-and-follow a moving rover — 10 points

| Section | Points |
|---|---|
| §1 Two vehicles, bridged into ROS 2 | 2.0 |
| §2 Search and marker identification | 3.0 |
| §3 From a detection to a world-frame position | 2.0 |
| §4 Tracking, and a lost/found cycle | 3.0 |

### Staging

In every topic the four sections are **cumulative milestones** — a section only pays out if all
the sections before it are demonstrated:

| Demonstrated | Obstacle avoidance | Drone localisation | Find-and-follow |
|---|---|---|---|
| §1 only | 2.0 | 1.5 | 2.0 |
| §1 + §2 | 5.0 | 6.0 | 5.0 |
| §1 + §2 + §3 | 7.5 | 8.0 | 7.0 |
| All four | **10.0** | **10.0** | **10.0** |

## Rules common to all topics

These apply whichever topic you pick:

1. **One topic, 10 points**, split across that topic's four sections — the exact split is in
   the table above and in the topic's own file.
2. **Assignment 2 counts toward the same combined threshold as Assignment 1** — 56 % of Assignment 1 + Assignment 2
   together (≥ 16.8 / 30) to be allowed to sit the final test. There is no separate per-assignment
   minimum for Assignment 2 — see [`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md#grading).
3. **Points are awarded for what demonstrably works** during the defence, on a clean checkout
   of your repository.
4. **Later sections require the earlier ones.** You cannot score the advanced part of a topic
   without the foundation it stands on — see [Staging](#staging).
5. **Documentation is required** — same expectations as
   [A1.4](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/04_documentation_and_defence.md): what you chose, why, pros and cons, plus a
   solution diagram. Each topic's file lists what its documentation has to cover on top of that.
6. **You must be able to explain everything you submit.** Team work is allowed; the defence is
   individual.
7. **A badly organized git history can cost you points on top of the above.** No credit for
   git hygiene itself, but a repository nobody can follow — one dumped commit, no sign of who
   on a team did what — gets a deduction. See
   [`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md#git-version-control).

## Deliverables

Common to all topics — each topic's file adds its own specifics:

- Git repository with your ROS 2 package(s), buildable with `colcon build`, extending your
  Assignment 1 control node.
- `README.md`: build instructions, how to bring up and launch everything the topic needs
  (bridges, extra SITL/MAVROS instances, the sensor package), dependencies.
- Documentation per rule 5 above (in the repository — Markdown or PDF).
- Rosbag or screen recording of the complete, successful run.

## Where to start

1. **Read all three topic files before week 7** — at least *What you are building* and the
   point staging table — and pick the one whose new part you want to learn.
2. **Make sure your Assignment 1 control node actually flies.** All three topics extend it; a shaky
   position controller will cost you time in any of them.
3. **Bring up the topic's world and look at the raw data before writing code** — the point
   cloud in RViz, the `/localization/*` topics with `ros2 topic hz` / `ros2 topic echo`, the
   rover world's cameras with `gz topic -l`. Each topic file tells you how.
4. Keep perception or estimation and flight control in **separate callback groups**
   ([`mt_executor_demo`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/mt_executor_demo/README.md)) from the start — heavy processing
   must never stall your setpoint publisher.

## Links

- Setup, MAVROS/MAVLink references and tooling: [`../assignment1/useful_links.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/useful_links.md)
- ROS 2 workspace, topics and QoS: [`../../tutorial/ros2_cheatsheet.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/tutorial/ros2_cheatsheet.md)
- Callback groups and the multithreaded executor: [`../../mt_executor_demo/README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/mt_executor_demo/README.md)
- [`ros_gz` bridge](https://github.com/gazebosim/ros_gz) — needed for obstacle avoidance's point cloud and the rover topic's cameras
- [`localization_sensor_sim`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/localization_sensor_sim/README.md) — the sensor package for the localisation topic

Note that the **10-point test on the lecture material** is a separate thing from Assignment 2, written
in the second half of the semester — see [`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md#overview).
