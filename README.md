# spot_sim_slam

**Dockerized Gazebo simulation of Boston Dynamics Spot with a simulated
RealSense D435i, Isaac ROS Visual SLAM, and Nav2** — for testing the full
`spot_vslam_nav` autonomy stack on a laptop before deploying to the
Jetson Orin Nano + real Spot.

```
┌─────────────────────────── your laptop (any distro, Jazzy host untouched) ───────────────────────────┐
│                                                                                                      │
│  ┌────────── sim (Humble) ──────────┐   ┌───── vslam (Humble+CUDA) ─────┐  ┌───── nav (Humble) ────┐ │
│  │ Gazebo Sim world                 │   │ Isaac ROS Visual SLAM         │  │ Nav2 (MPPI, costmaps) │ │
│  │ Spot model + champ gait ctrl     │   │ (cuVSLAM, NVIDIA GPU)         │  │ cmd_vel gate          │ │
│  │ simulated D435i:                 │──►│ /camera/infra1..2 + /imu      │  │ goal_cli              │ │
│  │  stereo IR + depth + IMU         │   │ publishes map->odom           │  │                       │ │
│  │ ros_gz bridge (RealSense topics) │   └───────────────────────────────┘  └───────────────────────┘ │
│  │ champ publishes odom->base_link  │        --profile cpu swaps in RTAB-Map (no GPU needed)         │
│  └──────────────────────────────────┘                                                                │
│                                  host network / DDS  +  /clock (sim time)                            │
└──────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

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
| `sim`, `nav`, `rviz` | Docker, X11 (`xhost +local:docker`) |
| `vslam` | NVIDIA GPU + driver + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) |
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

# GPU laptop (full-fidelity):
docker compose --profile gpu up -d

# CPU-only laptop:
docker compose --profile cpu up sim slam_cpu nav rviz
```

Then drive it:

```bash
# teleop sanity check
docker exec -it spot_sim bash -lc \
  "source /ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"

# autonomous goal (same CLI as on the robot)
docker exec -it spot_nav bash -lc \
  "source /ws/install/setup.bash && ros2 run spot_vslam_nav goal_cli -- 3.0 1.0 0"
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
  from the depth image + camera_info directly. Nav2's costmaps consume that
  point cloud straight (`depth_cloud` observation source) rather than going
  through `pointcloud_to_laserscan` — its `/scan` never reliably delivered data.

## Layout

```
spot_sim_slam/
├── docker-compose.yml
├── docker/
│   ├── sim.Dockerfile        # Humble + Gazebo + champ Spot + our packages
│   └── vslam.Dockerfile      # CUDA + Humble + isaac_ros_visual_slam (apt)
├── sim/                      # ROS package: spot_sim_slam
│   ├── urdf/d435i_sim.urdf.xacro     # simulated D435i (stereo IR+depth+IMU)
│   ├── scripts/inject_camera.py      # welds the camera onto Spot's URDF
│   ├── config/gz_bridge.yaml         # gz<->ROS topics (RealSense-compatible)
│   ├── config/nav2_sim_overrides.yaml
│   └── launch/{sim,vslam_sim,rtabmap_fallback}.launch.py
└── spot_vslam_nav/           # the UNCHANGED robot package (also deploys to Jetson)
```
