# spot_sim_slam

**Dockerized Gazebo simulation of Boston Dynamics Spot with a simulated
RealSense D435i, Isaac ROS Visual SLAM, Isaac ROS nvblox, and Nav2** — for
testing the full `spot_vslam_nav` autonomy stack on a laptop before deploying
to the Jetson Orin Nano + real Spot.

On top of teleop and point-to-point navigation it runs an **autonomous search
mission**: tell Spot how many people are hidden in the world and it explores
on its own until it has found them, dropping a pin on the 2D map for each one.

```
┌─────────────────────────── your laptop (any distro, Jazzy host untouched) ───────────────────────────┐
│                                                                                                      │
│  ┌────────── sim (Humble) ──────────┐   ┌───── vslam (Humble+CUDA) ─────┐  ┌───── nav (Humble) ────┐ │
│  │ Gazebo Sim world                 │   │ Isaac ROS Visual SLAM         │  │ Nav2 (MPPI, costmaps) │ │
│  │ Spot model + champ gait ctrl     │   │ (cuVSLAM, NVIDIA GPU)         │  │ cmd_vel gate          │ │
│  │ simulated D435i:                 │──►│ /camera/infra1..2 + /imu      │  │ goal_cli              │ │
│  │  stereo IR + depth + RGB + IMU   │   │ publishes map->odom           │  │                       │ │
│  │ ros_gz bridge (RealSense topics) │   └───────────────────────────────┘  └───────────────────────┘ │
│  │ champ publishes odom->base_link  │   ┌──── nvblox (Humble+CUDA) ─────┐  ┌──── search (Humble) ──┐ │
│  └──────────────────────────────────┘   │ Isaac ROS nvblox              │  │ frontier exploration  │ │
│                                         │ TSDF + ESDF from depth        │  │ scan/explore mission  │ │
│  ┌────── perception (Humble) ───────┐   │ 2D slice ──► /map             │  │ drives Nav2 actions   │ │
│  │ YOLO person detector (CPU)       │   └───────────────────────────────┘  └───────────────────────┘ │
│  │ depth reprojection ──► map frame │                                                                │
│  │ person_map ──► pins + JSON       │        --profile cpu swaps in RTAB-Map (no GPU needed)         │
│  └──────────────────────────────────┘                                                                │
│                                  host network / DDS  +  /clock (sim time)                            │
└──────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

## The search mission in one paragraph

nvblox integrates the D435i depth stream into a TSDF on the GPU and derives a
Euclidean signed distance field from it. A horizontal slice of that ESDF
becomes `/map`, an ordinary `nav_msgs/OccupancyGrid`, which does three jobs at
once: it is Nav2's static layer, it is where the explorer finds *frontiers*
(free cells touching unknown cells), and it is the canvas the person pins are
drawn on. The mission controller alternates two Nav2 behaviours — spin in
place to sweep the camera's 69° field of view across the full circle, then
navigate to the best-scoring frontier — while YOLO watches the RGB stream.
Each detection is reprojected through the depth image into the map frame, and
a track that has been seen four times is promoted to a confirmed pin. When the
pin count reaches the number you asked for, the mission stops.

## Why Docker, and why Humble inside the containers?

Your laptop runs ROS 2 Jazzy — but your **Jetson Orin Nano runs Humble**
(Isaac ROS 3.x on JetPack 6; the Jazzy-based Isaac ROS releases target x86 and
Jetson Thor, not Orin Nano). Testing natively on Jazzy would mean testing a
*different* stack than you deploy. Containers solve this perfectly:

* Everything inside is **Humble — identical to the Jetson**, including the very
  same `spot_vslam_nav` package (bundled in this repo) with its Nav2 params,
  cmd_vel gate, and health monitor.
* Your host Jazzy install is untouched and not even used.
* The simulated D435i publishes the **same topic names and frames** as the real
  RealSense driver, so the VSLAM/Nav2 configs are byte-for-byte transferable.

## What's simulated, honestly

* **Spot's body and gait** via the open-source [CHAMP](https://github.com/chvmp/champ)
  quadruped controller in the [`spot_gazebo_ros2`](https://github.com/g1y5x3/spot_gazebo_ros2)
  package (there is **no official Boston Dynamics Gazebo model**). It walks,
  accepts `cmd_vel`, publishes `odom -> base_link` — good enough to exercise
  SLAM + navigation end to end. It does *not* reproduce Spot's real gait
  dynamics, stair climbing, or self-righting.
* **D435i** as two grayscale cameras (50 mm baseline, 640×360@30), a depth
  camera, and a 200 Hz IMU with realistic noise, injected into Spot's URDF at
  image build time (`sim/scripts/inject_camera.py`).
* **cuVSLAM** runs the real GPU code — same package you'll run on the Jetson.

## Requirements

| Service | Needs |
|---|---|
| `sim`, `nav`, `rviz`, `search` | Docker, X11 (`xhost +local:docker`) |
| `vslam`, `nvblox` | NVIDIA GPU + driver + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) |
| `perception` | nothing special — CPU-only PyTorch |
| `slam_cpu` (fallback) | nothing special — pure CPU |

No NVIDIA GPU in the laptop? Use `--profile cpu`: RTAB-Map consumes the same
stereo/IMU topics and publishes the same `map->odom` TF, so Nav2, the safety
gate, and all integration logic are still fully testable. Just remember it's a
stand-in — final SLAM-quality validation happens with cuVSLAM (GPU container
or the Jetson itself).

See [USAGE.md](USAGE.md) for a step-by-step operating guide, including how to
record a cuVSLAM map and reuse it in a later session.

## Quick start

```bash
xhost +local:docker
docker compose build            # first build ~30–45 min (clones + compiles sim)

docker compose up -d sim vslam nvblox nav search perception rviz
```

Gazebo and RViz open as windows on your screen. Gazebo boots slowly on a
software-rendered GPU fallback — wait for the world to unpause before
sending commands:

```bash
docker logs -f spot_sim | grep "data: true"
```

### Autonomous search

```bash
# explore only, no person target (just builds the map)
docker exec spot_search bash -lc \
  "source /ws/install/setup.bash && ros2 run spot_search search_cli 0"

# search for N people (stops once found, otherwise explores until exhausted)
docker exec spot_search bash -lc \
  "source /ws/install/setup.bash && ros2 run spot_search search_cli 2"
```

Add `--no-follow` to either command to send the request and return
immediately instead of streaming status in your terminal.

```bash
# stop a running mission
docker exec spot_search bash -lc \
  "source /ws/install/setup.bash && ros2 service call /search/stop std_srvs/srv/Trigger"

# check mission status
docker exec spot_search bash -lc \
  "source /ws/install/setup.bash && ros2 topic echo /search/status"
```

### Manual driving (optional)

```bash
# teleop sanity check
docker exec -it spot_sim bash -lc \
  "source /ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# single point-to-point goal (same CLI as on the robot)
docker exec -it spot_nav bash -lc \
  "source /ws/install/setup.bash && ros2 run spot_vslam_nav goal_cli -- 3.0 1.0 0"
```

### Shutting down

```bash
docker compose down
```

### What to verify (your pre-Jetson checklist)

1. `ros2 run tf2_tools view_frames` inside any container → exactly one chain
   `map → odom → base_link`, with `map→odom` owned by `visual_slam` (or
   `rtabmap`) and `odom→base_link` owned by champ. This is the same contract
   as on the real robot (`body` instead of `base_link`).
2. `/visual_slam/tracking/odometry` tracks the Gazebo ground truth while you
   teleop in circles; drive a loop and watch `map→odom` snap on loop closure.
3. Send a Nav2 goal behind an obstacle → Spot plans around it; block its path
   with a Gazebo box mid-run → costmap updates, MPPI replans.
4. Kill the `vslam` container mid-navigation → `cmd_vel_gate` must stop the
   robot within ~1.5 s (`/vslam_ok` goes false). This is your safety-path test.
5. `docker stop spot_nav` mid-walk → champ receives no cmd_vel and the robot
   stops (watchdog behavior mirrors the real driver's `cmd_duration`).

## Mapping sim ⇄ real robot

| | Simulation | Real robot (Jetson) |
|---|---|---|
| Base frame | `base_link` (champ) | `body` (spot_ros2) |
| Kinematic odom | champ, `/odom` | spot_ros2, `/odometry` |
| `map→odom` | cuVSLAM (identical config) | cuVSLAM |
| cmd_vel consumer | champ gait controller | spot_ros2 driver |
| Launch | `docker compose up` | `ros2 launch spot_vslam_nav spot_slam_nav.launch.py` |

The only launch-arg differences are the frame/topic names above — already
parameterized in `spot_vslam_nav`'s launch files.

## Known rough edges

* `inject_camera.py` auto-detects Spot's URDF and base link in the upstream
  repo at build time and prints what it chose — check the build log once. If
  upstream restructures, point `--search-root`/parent link manually.
* The upstream sim repo is Humble + Gazebo Sim; if its bringup launch file name
  changes, adjust `find_upstream_bringup()` in `sim/launch/sim.launch.py`.
* Isaac ROS apt packaging on x86 is picky about driver/CUDA combos. If the
  `vslam` image misbehaves, use NVIDIA's own `isaac_ros_common` `run_dev.sh`
  container instead and launch the mounted `sim/launch/vslam_sim.launch.py`
  from inside it — the rest of the stack doesn't care where cuVSLAM runs.
* Sim time: everything runs with `use_sim_time:=true`; the `/clock` bridge is
  in `sim/config/gz_bridge.yaml`. If TF complains about extrapolation, verify
  `/clock` is actually publishing.
* Gazebo's own depth camera point-cloud output is bugged in this version (every
  point comes out with x >= 0 regardless of source pixel); `sim.launch.py` runs
  a small `depth_to_points` node instead, generating `/camera/depth/color/points`
  from the depth image + camera_info directly. Nav2's *local* costmap consumes
  that point cloud straight (`depth_cloud` observation source) rather than going
  through `pointcloud_to_laserscan` — its `/scan` never reliably delivered data.
  The *global* costmap now takes nvblox's `/map` as a static layer instead.
* **nvblox's ESDF slice heights are in the global frame, not relative to the
  floor.** The `map`/`odom` origin sits at Spot's spawn pose, roughly 0.7 m up,
  so the slice defaults in `nvblox_sim.launch.py` are negative numbers. If
  `/map` comes out empty or solid, this is the first thing to check — confirm
  the floor's height with `ros2 run tf2_ros tf2_echo map base_link` and move
  `slice_min_height`/`slice_max_height` to bracket knee-to-chest height above it.
* **nvblox reconstructs in `map`, which cuVSLAM corrects on loop closure, and
  nvblox cannot deform an already-integrated map.** So a loop closure leaves a
  visible seam. The alternative — reconstructing in `odom` — has no seams but
  inherits champ's kinematic drift, which over a ~100 m tunnel is far worse
  than a seam. Switch with `global_frame:=odom` if your run is short.
* **Person detection is the least realistic part of the stack.** The sim's
  victims are "Rescue Randy" rescue-training mannequins rendered at 640×480,
  which YOLO scores lower than it would a real person — hence the 0.35
  confidence floor, well below a sane hardware default. `person_map`'s
  `min_hits` is what actually rejects false positives; tighten `confidence`
  and relax `min_hits` when moving to the real robot.

## Layout

```
spot_sim_slam/
├── docker-compose.yml
├── docker/
│   ├── sim.Dockerfile        # Humble + Gazebo + champ Spot + our packages
│   ├── vslam.Dockerfile      # CUDA + Humble + isaac_ros_visual_slam + nvblox
│   └── perception.Dockerfile # Humble + YOLO (CPU torch), weights baked in
├── sim/                      # ROS package: spot_sim_slam
│   ├── urdf/d435i_sim.urdf.xacro     # simulated D435i (stereo IR+depth+RGB+IMU)
│   ├── scripts/inject_camera.py      # welds the camera onto Spot's URDF
│   ├── config/gz_bridge.yaml         # gz<->ROS topics (RealSense-compatible)
│   ├── config/nav2_sim_overrides.yaml
│   └── launch/{sim,vslam_sim,rtabmap_fallback}.launch.py
├── spot_search/              # autonomous person search (also deploys to Jetson)
│   ├── map_slice_to_occupancy.py  # nvblox ESDF slice -> /map OccupancyGrid
│   ├── person_detector.py         # YOLO + depth reprojection -> map-frame 3D
│   ├── person_map.py              # dedupe/confirm -> pins, markers, JSON
│   ├── frontier.py                # frontier extraction (pure numpy)
│   ├── search_mission.py          # scan/explore state machine over Nav2
│   ├── search_cli.py              # "find N people" + live status
│   ├── save_search_map.py         # render map+pins to PNG
│   └── launch/{nvblox_sim,perception,search}.launch.py
└── spot_vslam_nav/           # the UNCHANGED robot package (also deploys to Jetson)
```

## Which container runs what

| Service | Image | Role |
|---|---|---|
| `sim` | sim | Gazebo, Spot, D435i, gz↔ROS bridge |
| `vslam` | vslam | cuVSLAM → `map→odom` |
| `nvblox` | vslam | nvblox reconstruction + `/map` |
| `nav` | sim | Nav2 + cmd_vel safety gate |
| `perception` | perception | YOLO detector + person pins |
| `search` | sim | mission controller + frontier explorer |
| `rviz` | sim | visualization |

`spot_search` is installed into three of these images and each runs only the
nodes whose dependencies it has — `nvblox_msgs` exists only in the vslam
image, `ultralytics`/`cv_bridge` only in the perception image.
