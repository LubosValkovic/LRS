# Find-and-follow a moving rover (10 points)

One of the three [Assignment 2](README.md) topics. **Deadline: end of week 12.** See
[`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md) for the semester rules and [`README.md`](README.md) for the
rules shared by all topics (threshold, documentation, git).

## What you are building

Two drones, each finding and then following its own moving ground rover using only its down
camera — never a known coordinate. This extends the control node from
[Assignment 1](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/README.md): same node, same MAVROS setpoints, same position
controller, same `MultiThreadedExecutor`/callback-group pattern. What's new is a second vehicle
running the same architecture side by side, and a perception input (the down camera) feeding a
decision (search vs. track) back into the thing you already built.

Fly it in `worlds/fei_lrs_precision_landing.world`:

```bash
scripts/run_gazebo.sh fei_lrs_precision_landing.world
```

That world already has, built in:

- **Two drones**, `drone_1` and `drone_2`, each with a single down-facing (nadir) camera —
  topic `drone_<N>/down_camera/image_raw`, 640x480, 60° horizontal FOV, 30 Hz — replacing the
  outdoor world's stereo pair.
- **Two rovers**, `aruco_rover_1` and `aruco_rover_2`, each driving a fixed circle out of the
  box (no ROS 2 node needed for that part to move) and each carrying an ArUco marker
  (`DICT_4X4_50`) on a white pad on top.
- **The pairing rule: marker id == the matching drone's `SYSID_THISMAV`.** `drone_1`
  (`SYSID_THISMAV` 1) is paired with marker id 1, `drone_2` (`SYSID_THISMAV` 2) with marker id
  2. Keep that rule — it is how a drone tells its own rover apart from the other one if both
  markers are ever visible.
- **Two independent ArduPilot SITL FDM ports already wired into the world**: `drone_1` is
  instance 0 (`fdm_port_in` 9002), `drone_2` is instance 1 (`fdm_port_in` 9012). Bring both up
  with `sim_vehicle.py` the same way [Assignment 1](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/README.md) has you bring up
  one, just twice, with `--instance`/`--sysid` set to match and each in its own working
  directory (two `sim_vehicle.py` processes building the same ArduCopter binary at the same time
  can race on the shared `ardupilot/build` directory — build once, or serialise the two builds,
  before running both):

  ```bash
  # terminal 1 — drone_1
  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --add-param-file=$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm \
    --instance 0 --sysid 1 --use-dir /tmp/ardupilot-drone1 \
    -l 48.15135451,17.07361560,150,0

  # terminal 2 — drone_2
  sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON \
    --add-param-file=$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm \
    --instance 1 --sysid 2 --use-dir /tmp/ardupilot-drone2 \
    -l 48.15135451,17.07388440,150,0
  ```

  The `-l` home locations above are each drone's actual spawn point (`(-10,-10)` and
  `(10,-10)` local ENU metres) reprojected from the world's own geodetic origin — they matter:
  ArduPilot's SITL JSON backend has no idea where in the Gazebo world its model was placed, so
  `-l` has to already account for it, or your local-frame math will be off by exactly that
  offset. `--instance 0`/`1` automatically pick FDM ports 9002/9012 to match the world (base
  port + 10 × instance) and give each vehicle its own non-colliding MAVLink UDP port
  (14550/14560) for MAVROS to connect to later — that part of "get two vehicles running without
  their ports colliding" is handled for you; getting two *MAVROS* instances talking to the right
  one each, in the right ROS namespace, is part of the assignment.

Everything else — MAVROS (x2), bridging the cameras into ROS 2, the ArUco detector and the
search/track controller — is what you build. The rovers' random motion comes from
`scripts/randomize_rovers.py`, supplied with the repository. See the specification below.

## Specification

### 1. Two vehicles, bridged into ROS 2 — 2.0 points

- Two independent MAVROS instances, one per drone, each in its own ROS 2 namespace, each
  connected to the matching SITL instance above.
- Bridge both cameras' `image_raw` and `camera_info` topics with `ros_gz_bridge` (get the exact
  `camera_info` topic name from `gz topic -l` on the running world — gz-sim's camera sensor
  publishes it automatically, it is not declared anywhere you can just read).
- Verify independently that both vehicles' telemetry is actually flowing before writing any
  mission logic — `ros2 topic info <topic> --verbose` showing a real publisher, not just the
  topic name appearing in `ros2 topic list`, which it will even with nothing publishing to it if
  something else has already subscribed.

**Accepted when:** `ros2 node list`/`ros2 topic list` show both drones' full MAVROS telemetry
and both cameras' bridged image topics, simultaneously, from a single `gz sim` + two SITL + two
MAVROS bring-up.

### 2. Search and marker identification — 3.0 points

- Each drone arms, takes off to a search altitude (your parameter), and searches for its own
  marker with the down camera — `cv2.aruco`/`cv::aruco`, `DICT_4X4_50`, plus `cv_bridge` to get
  from `sensor_msgs/msg/Image` to an OpenCV `Mat`.
- **A real search, not a fly-to-a-known-coordinate.** You are not given the rover's position;
  design a search pattern that covers the area it could plausibly be in and confirm the
  requirement is met by actually detecting the marker, not by computing where the rover must be
  from the circle it starts on.
- **Confirm the marker id matches this drone's own `SYSID_THISMAV`** before treating a detection
  as a target. A marker with the wrong id — the *other* rover's — must be ignored, not chased;
  demonstrate this with both drones running at once.

**Accepted when:** both drones, run together, each find their own marker and never react to the
other one, from a search pattern that does not assume where the rover starts.

### 3. From a detection to a world-frame position — 2.0 points

- Turn a detected marker into a target position in the drone's own local ENU frame: camera
  intrinsics (from `camera_info`, not hardcoded), the marker's physical size, and the camera's
  mounting offset and orientation relative to the drone body.
- **The down camera's mount pose is not simply "pointing straight down along -Z" by inspection.**
  Work out the actual optical-frame convention and verify it empirically — log a detection's
  estimated position against something you can check by eye in the simulator (e.g. the drone's
  own reported altitude while hovering directly over a rover) before trusting any sign or axis
  in your pose math. This is exactly the kind of mistake
  [`03_outdoor_gps_mission.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/03_outdoor_gps_mission.md)'s ENU/NED warnings are
  about.
- Check what pose-estimation API your installed OpenCV actually has before using it — the old
  `estimatePoseSingleMarkers` path is deprecated/removed in newer OpenCV, replaced by a
  `solvePnP`-based approach.
- **The marker's physical size is not automatically obvious from the model.** The rover carries
  a 1 m white pad with a smaller printed marker on it — measure or derive what
  `cv2.aruco`/`cv::aruco` actually reports corners for, not the pad size, and not necessarily the
  marker's nominal printed size either if the texture has any extra margin baked in. Getting this
  wrong silently scales every distance you compute; verify it the same way as the mount pose,
  empirically.

**Accepted when:** a detection's computed world position, logged against the drone's own known
altitude while roughly overhead, is accurate to within about the size of the marker itself, not
metres off.

### 4. Tracking, and a lost/found cycle — 3.0 points

- Once a marker is confirmed, hold a **configurable** altitude above the rover (a parameter, not
  a hardcoded number) and continuously reposition to stay above it as it moves.
- **Run the supplied rover randomizer** — out of the box each rover drives a fixed,
  precomputable circle, which a controller could solve once and never actually react to.
  `scripts/randomize_rovers.py` gives each rover a new random speed and yaw rate every few
  seconds instead, and keeps it within about 5 m of its circle's centre:

  ```bash
  scripts/randomize_rovers.py --seed 42      # Ctrl+C stops both rovers; --help lists the limits
  ```

  Your teacher picks the seed at the defence, so develop against more than one.
- If the marker is lost from view for longer than a short, **configurable** timeout, drop back
  into search instead of continuing toward a stale last-known position.
- **Never land.** There is no landing state in this exercise; have a working abort path
  (service or topic, matching whatever `~/abort` convention you already used in Assignment 1)
  that brings the vehicle to a safe hold/RTL instead.

**Accepted when:** with both rovers moving under randomized commands, both drones independently
track their own rover, and you can demonstrate at least one genuine lost-then-reacquired cycle
per drone — not staged, an actual gap long enough to trip your timeout.

## Point staging

The four sections above are also staged milestones — each one only pays out if the ones before
it are demonstrated, per the [Assignment 2 rules](README.md#rules-common-to-all-topics):

| Demonstrated | Points | % |
|---|---|---|
| §1 only | 2.0 | 20 % |
| §1 + §2 (both drones find their own marker) | 5.0 | 50 % |
| §1 + §2 + §3 (accurate world-frame position, not yet flown to) | 7.0 | 70 % |
| All four (tracking, randomized motion, a real lost/found cycle) | 10.0 | 100 % |

There is **no separate Assignment 2 pass mark** — none of the milestones above is a minimum.
The only gate is the one to sit the final test: **Assignment 1 + Assignment 2 combined**, 56 % of
30, at least **16.8 / 30** — see [`../README.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/README.md#grading). How many points you need
here therefore depends on what you scored on Assignment 1.

## Hints

- **Get the search/detect pipeline working with one drone at a time before running both.**
  Confirm one drone finds its own marker and ignores the other rover's, then bring the second
  one up. Debugging two SITL/MAVROS stacks and your mission logic simultaneously is much harder
  than either alone.
- **`gz topic -l` and `gz topic -i -t <topic>` are your friends** for every "what's the exact
  topic name and message type" question in this assignment — the down camera's `camera_info`
  and everything else. Don't guess from documentation for a different
  Gazebo version; check the running world.
- **A fast search leg plus a slow vision pipeline is a bias, not just noise.** If your node reads
  "current" telemetry at the moment it finishes processing a frame rather than at the moment the
  frame was captured, and the drone is moving quickly, the position you compute for a detection
  will be off by roughly (vehicle speed) × (pipeline latency) — consistently in the direction of
  travel. Capping your search/track speed is a simpler fix than trying to time-synchronise two
  topics that may not even share a clock (`ros_gz_bridge` topics carry Gazebo's own
  simulation-clock timestamps; MAVROS's do not, unless you also bridge `/clock` and run every
  node with `use_sim_time`).
- **The rover model's wheels are cosmetic** — motion comes entirely from
  `gz-sim-velocity-control-system` applying a body-frame twist, not from the wheel joints, so
  nothing on the rover actually rolls. If you ever see a rover tip onto its side (check its
  orientation on `model/aruco_rover_<N>/pose` — that ground-truth topic exists for exactly
  this kind of debugging, not for your follow controller to consume) its marker stops facing the
  sky and no down-facing camera at any altitude will find it. The collision friction is already
  tuned low enough (`mu = 1.0`, roughly rubber-on-concrete) that the baked-in circle alone runs
  indefinitely without this happening, and the randomizer ramps its commands so it does not tip
  the rovers either — if you still hit it after raising the randomizer's limits (`--speed-max`,
  `--yaw-rate-max`), it is almost certainly that speed/yaw-rate combination asking the box to turn
  faster than it can slip at that friction, not a pre-existing issue with the model.
- Keep vision processing and flight control in **separate callback groups** (see
  [`mt_executor_demo`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/mt_executor_demo/README.md)) — ArUco detection takes real
  processing time per frame, and it must never stall your setpoint publisher.
- **MAVProxy needs a real, interactive terminal.** If you ever script SITL bring-up for
  automated testing rather than running it by hand, a backgrounded MAVProxy process with no
  controlling TTY exits almost immediately (it reads commands from stdin) — this looks exactly
  like "SITL never sends a heartbeat" and is not.

## Deliverables

- ROS 2 package(s) building with `colcon build`, extending your Assignment 1 control node to run
  two independent instances (one per drone).
- Your multi-vehicle bring-up (SITL ×2, MAVROS ×2, bridge) as scripts or a launch file, not just
  commands you ran once and remember.
- Documentation per [`README.md`](README.md#rules-common-to-all-topics): your camera
  intrinsics/extrinsics/marker-size derivation and how you verified it, your
  altitude-above-rover and detection-timeout parameters and their defaults, pros/cons, and a
  diagram of the perception → decision → control data flow for one drone.
- Rosbag or screen recording of both drones searching, tracking, and going through at least one
  lost/found cycle each, at the same time.

## Links

- [`useful_links.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/assignments/assignment1/useful_links.md) — MAVROS/MAVLink references, general tooling
- [`tutorial/ros2_cheatsheet.md`](https://github.com/KocurMaros/LRS-URK/blob/99602ce0684c4451a1c40591716016e18fa05ee7/tutorial/ros2_cheatsheet.md) — subscribers, QoS, callback groups
- [`ros_gz` bridge](https://github.com/gazebosim/ros_gz) — `parameter_bridge` usage and the full gz ↔ ROS type table
- [ArduPilot SITL — multiple vehicles](https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html) — `sim_vehicle.py` instance/sysid conventions
- [OpenCV ArUco detection](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
