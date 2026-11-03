# Drone localisation with a simplified Kalman filter (10 points)

An [Assignment 2](README.md) topic. **Deadline: end of week 12.** See
[`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md) for the semester rules and [`README.md`](README.md) for the
rules shared by all topics (threshold, documentation, git).

## What you are building

Your drone has three deliberately imperfect localisation sensors: noisy GPS, drifting visual
odometry (VO), and a yaw-rate gyro with a static bias. Build a small Kalman filter that combines
them, keeps estimating through missing or bad measurements, and then use that estimate to fly a
mission.

This extends the control node from [Assignment 1](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/README.md). MAVROS still arms the
drone and receives setpoints, but its local position is no longer the answer fed back to your
controller. The new data path is:

```text
GPS position ───────────┐
VO pose / body velocity ├─> your KF or EKF ─> estimated x, y, z, yaw ─> controller ─> MAVROS
IMU yaw rate ───────────┘
```

This is intentionally a **simplified localisation filter**, not a full inertial navigation
system. You estimate eight states, use only the gyro's Z axis, and do not integrate the
accelerometer.

## Starting the sensor simulation

Use the dedicated world, `worlds/fei_lrs_gazebo_localization.world`. It is separate from the
normal hangar world so this assignment's sensors do not change any other exercise. Start Gazebo
from the repository root with an explicit seed:

```bash
GZ_SIM_RESOURCE_PATH="$PWD/models:${GZ_SIM_RESOURCE_PATH:-}" \
  gz sim -r -v 4 --seed 42 "$PWD/worlds/fei_lrs_gazebo_localization.world"
```

Start ArduPilot SITL and MAVROS in their own terminals as usual:

```bash
scripts/run_sitl.sh
scripts/run_mavros.sh
```

Build and source the supplied sensor package, then start its bridges and corruption node with
the **same public seed**:

```bash
colcon build --packages-select localization_sensor_sim
source install/setup.bash
ros2 launch localization_sensor_sim localization_sensors.launch.py seed:=42
```

The default rates, covariance, noise, drift and outage settings are in
`localization_sensor_sim/config/sensors.yaml`. Your teacher may use a different seed at the
defence, so do not tune against one noise trace or one hidden bias.

### Sensor contract

| Topic | Type | Nominal rate | Frame and meaning |
|---|---|---:|---|
| `/localization/gps` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 5 Hz | Position in MAVROS-compatible ENU, `map`; orientation is invalid |
| `/localization/vo` | `nav_msgs/msg/Odometry` | 30 Hz | Drifting pose/yaw in `vo_odom`; linear velocity is in the `base_link` body frame |
| `/localization/imu` | `sensor_msgs/msg/Imu` | 100 Hz | Only `angular_velocity.z` and its variance are valid |
| `/evaluation/ground_truth` | `nav_msgs/msg/Odometry` | 100 Hz | Transformed truth for a separate evaluator or plotting process only |

Gazebo starts the drone at `(13, 7, 0)` with yaw `0`. The supplied simulator rotates that into
the same ENU convention used here and by MAVROS:

```text
x_ENU = -y_Gazebo     y_ENU = x_Gazebo     z_ENU = z_Gazebo
yaw_ENU = wrap(yaw_Gazebo + pi/2)
```

The initial ENU pose is therefore approximately `(-7, 13, 0)` with yaw `pi/2`. GPS is noisy;
do not hard-code this pose into your filter.

These teacher controls are available during testing:

```bash
# Disable and restore GPS (the assessed outage lasts 15 seconds).
ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool "{data: true}"

# Disable and restore VO for a short tracking loss.
ros2 service call /localization/set_vo_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /localization/set_vo_enabled std_srvs/srv/SetBool "{data: true}"

# Corrupt the next enabled GPS sample with one large outlier.
ros2 service call /localization/inject_gps_outlier std_srvs/srv/Trigger "{}"
```

## Inputs you may and may not use

Your **estimator** may subscribe only to the three `/localization/` sensor outputs above. Your
controller may still use `/mavros/state`, MAVROS services, and MAVROS setpoint outputs. A separate
plotting or evaluation node may subscribe to `/evaluation/ground_truth`, but it must not send
anything back to the estimator or controller.

The following are forbidden as estimator or controller feedback:

- `/evaluation/ground_truth`, `/localization/_ground_truth_raw`, and Gazebo pose topics such as
  `/world/fei_lrs_world/pose/info` or `/world/fei_lrs_world/dynamic_pose/info`;
- MAVROS local/global position, odometry, attitude, heading, and pose outputs, including
  `/mavros/local_position/*`, `/mavros/global_position/*`, `/mavros/odometry/*`, and
  `/mavros/imu/data` or `/mavros/imu/data_raw`;
- IMU orientation or linear acceleration, including MAVROS IMU orientation and accelerometer
  integration — only `/localization/imu.angular_velocity.z` is an estimator input;
- `robot_localization`, ready-made Kalman-filter/localisation packages, or copied filter
  implementations.

Eigen in C++ and NumPy in Python are allowed. They are matrix libraries, not finished estimators.
You may inspect truth offline or in a separate plot to understand errors, but the teacher will
check your ROS graph and source code for a truth path into the submitted system.

## Specification

### 1. Sensor acquisition and characterisation — 1.5 points

- Subscribe to GPS, VO, and IMU with sensor-data-compatible QoS. Preserve and use each message's
  simulation timestamp; arrival time or a fixed callback period is not a substitute.
- Check `header.frame_id` and the VO `child_frame_id`. In particular, VO twist is body-frame
  velocity and cannot be treated as ENU velocity without a rotation.
- Make a short stationary and moving recording. Measure the actual topic rates, show GPS noise,
  show that VO drifts relative to truth, and show the gyro Z bias while the drone is not turning.
- Run once with a second seed and demonstrate that the hidden bias/noise changes. Repeating one
  seed must reproduce the same corruption.
- Ground truth may appear in a separate plotting process for this characterisation. It must never
  be subscribed to by your estimator.

**Accepted when:** all three callbacks receive correctly timestamped data with the expected
frames and rates, and your recording or plots clearly show GPS noise, VO drift, and gyro bias.

### 2. Simplified eight-state KF/EKF — 4.5 points

The mandatory state, in this order, is:

```text
x = [x, y, z, vx, vy, vz, yaw, gyro_bias]^T
```

- Predict position with a constant-velocity model and propagate yaw with the measured gyro rate
  minus the estimated bias. Use the elapsed time from message stamps on every prediction.
- Maintain the covariance `P` and meaningful, configurable process and measurement covariances
  `Q` and `R`. The covariance must be predicted and corrected with the state.
- Apply asynchronous measurement updates when each sensor arrives; do not wait for GPS, VO, and
  IMU to have matching timestamps. Ignore or handle duplicate/out-of-order timestamps safely.
- Wrap yaw innovations and the corrected yaw state consistently to `[-pi, pi)` (or an equivalent
  continuous convention). Estimate the gyro bias rather than assuming the configured value.
- Fuse GPS position and a documented selection of VO pose, yaw, and/or body velocity fields. You
  do not have to use every VO field, but your choice must constrain all required flight states.
- For VO body velocity, either implement the genuine nonlinear measurement and its EKF Jacobian,
  or rotate it with the current yaw estimate and document that linear-KF approximation. Both are
  accepted if implemented and explained correctly.
- Initialise deliberately. For example, use the first GPS/VO positions and the known initial ENU
  yaw, with suitably uncertain velocity and gyro bias — not the forbidden truth topic.

You may publish your estimate as `nav_msgs/msg/Odometry`, a pose message, or your own message. It
must expose `x`, `y`, `z`, and `yaw`, your README must name the topic and frame, and your controller
must actually consume that output.

**Accepted when:** the filter runs from launch without invalid values, its state and covariance
remain finite, the estimate follows the complete flight, yaw wraps correctly, and you can explain
your prediction, each measurement update, and how gyro bias becomes observable. Assessment is on
correct behaviour and your explanation, not one brittle error threshold.

### 3. Missing and bad measurements — 2.0 points

- Continue predicting through a **15-second GPS outage** without resetting, freezing the whole
  filter, or producing `NaN`/infinite state or covariance.
- Tolerate a short VO tracking loss in the same way. GPS or the motion model should carry the
  estimate until VO messages return.
- Reject the injected GPS outlier with a configurable innovation-based gate. Gate the innovation
  using its expected covariance — a fixed “distance from the current estimate” rule is not enough.
- Resume updates smoothly when either sensor returns. Do not snap to one raw measurement, reset
  the mission, or restart the estimator.

Log rejected measurements and sensor timeout/recovery transitions so the behaviour is visible at
the defence. Do not print at the sensor rate.

**Accepted when:** your running estimator survives the teacher's GPS and VO service tests, rejects
the one-shot GPS outlier for the stated statistical reason, and recovers without a state reset or
an invalid/discontinuous output.

### 4. Flight using estimated localisation — 2.0 points

Remove `/mavros/local_position/pose` (and every other forbidden pose source above) from controller
feedback. Capture the first valid estimated position as the mission origin, take off to 2 m, and
fly two laps through these **relative ENU offsets**:

```text
(0, 0) -> (0, -4) -> (2.5, -4) -> (2.5, 0) -> (0, 0)
```

- Hold approximately 2 m altitude and use roughly 0.5 m/s horizontal speed.
- Complete both laps, return to the relative origin, and land. A sensible implementation takes
  about 60–90 seconds.
- The teacher disables GPS for 15 seconds during a middle leg. Your flight and estimator must
  continue; no manual input is allowed after mission start.
- MAVROS state, arming/mode services, and position/velocity setpoint outputs remain allowed. Only
  using MAVROS localisation as feedback is forbidden.

**Accepted when:** on a teacher-selected seed, the complete takeoff, two-lap route, outage,
return, and landing succeed unattended, with the controller demonstrably driven by your estimator.

## Point staging

The four sections are cumulative milestones — each one only pays out if the ones before it are
demonstrated, per the [Assignment 2 rules](README.md#rules-common-to-all-topics):

| Demonstrated | Points | % |
|---|---:|---:|
| §1 only | 1.5 | 15 % |
| §1 + §2 (working eight-state filter) | 6.0 | 60 % |
| §1 + §2 + §3 (outages and outlier) | 8.0 | 80 % |
| All four (flown autonomously) | 10.0 | 100 % |

There is **no separate Assignment 2 pass mark**. The gate to sit the final test is Assignment 1
+ Assignment 2 combined: 56 % of 30, at least **16.8 / 30** — see
[`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md#grading). Your available total therefore depends on what you already
demonstrated in Assignment 1.

## Hints

- **Characterise before filtering.** Use `ros2 topic hz`, `ros2 topic echo --field header`, and a
  small rosbag while hovering. It is much easier to choose `Q` and `R` after seeing the sensors.
- Start with prediction plus GPS position, then add yaw/gyro bias, then VO. Plot after every step.
  A filter with one correct update is a better debugging base than three simultaneous mystery
  updates.
- The IMU is faster than VO and GPS. A practical design predicts on each IMU message and brings
  the state to a slower measurement's timestamp before correcting it.
- For a position observation, the innovation covariance has the form `S = H P H^T + R`. The
  quadratic innovation `r^T S^-1 r` is the useful quantity for a statistical gate. Make the
  threshold a parameter and be ready to justify it.
- Never compute a matrix inverse just to multiply by it. Solve the linear system with Eigen or
  NumPy instead, and use a numerically stable covariance update such as Joseph form if round-off
  makes `P` lose symmetry.
- Keep estimator and flight-control responsibilities separate. During development, replay the
  same sensor bag into the estimator without starting ArduPilot, then test control at a static
  hover before attempting the full route.

## Deliverables

- ROS 2 package(s), in C++ or Python, building with `colcon build` on ROS 2 Humble and Jazzy.
- Your eight-state estimator, its launch/config files, and the controller integration used for
  the final flight.
- A README naming your estimator output topic and frame, listing `Q`, `R`, initial covariance and
  gate parameters, and summarising the prediction/update equations and body-velocity treatment.
- Documentation per [`README.md`](README.md#rules-common-to-all-topics): choices, pros/cons, and a
  diagram of sensor → estimator → controller → MAVROS data flow.
- A short rosbag or screen recording showing sensor characterisation, one outlier rejection, the
  15-second GPS outage, and the complete autonomous flight.

## Links

- [`useful_links.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/useful_links.md) — MAVROS/MAVLink references and general tooling
- [`tutorial/ros2_cheatsheet.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/tutorial/ros2_cheatsheet.md) — subscribers, QoS, callback groups
- [`ros_gz` bridge](https://github.com/gazebosim/ros_gz) — Gazebo/ROS topic bridging
- [ROS 2 `nav_msgs/msg/Odometry`](https://docs.ros.org/en/humble/p/nav_msgs/msg/Odometry.html)
- [ROS 2 `sensor_msgs/msg/Imu`](https://docs.ros.org/en/humble/p/sensor_msgs/msg/Imu.html)
