# spot_vslam_nav

**GPU-accelerated Visual SLAM + autonomous navigation for Boston Dynamics Spot**,
built from [Isaac ROS Visual SLAM (cuVSLAM)](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam),
[Nav2](https://docs.nav2.org), and the [spot_ros2 driver](https://github.com/rai-opensource/spot_ros2),
running entirely on an **NVIDIA Jetson Orin Nano** mounted as a Spot payload.

```
                        ┌────────────────── Jetson Orin Nano (payload) ──────────────────┐
                        │                                                                │
  RealSense D435i/D455  │  ┌──────────────┐  map->odom TF   ┌──────────────────────────┐ │
  (stereo IR + IMU) ───────►  Isaac ROS   ├─────────────────►                          │ │
                        │  │  Visual SLAM │                 │           Nav2           │ │
                        │  │  (cuVSLAM,   │  /vslam_ok      │  planner ─ MPPI ─ smoother│ │
                        │  │   GPU)       ├──────┐          │  rolling global costmap  │ │
                        │  └──────────────┘      │          └────────────┬─────────────┘ │
                        │                        ▼                       │ cmd_vel_nav   │
  depth cloud ─────► pointcloud_to_laserscan ─► /scan ─► costmaps        ▼               │
                        │                            ┌──────────────────────────┐        │
                        │                            │       cmd_vel_gate       │        │
                        │                            │ (watchdog + VSLAM E-stop)│        │
                        │                            └────────────┬─────────────┘        │
                        │                                         │ /cmd_vel             │
                        │  ┌──────────────────────────────────────▼─────────────┐        │
   Spot (payload  ◄────────┤              spot_ros2 driver                      │        │
   Ethernet,            │  │   odom->body TF, /odometry, claim/power/stand      │        │
   192.168.50.3)        │  └────────────────────────────────────────────────────┘        │
                        └────────────────────────────────────────────────────────────────┘
```

## Why an external stereo camera?

Spot's five body cameras are wide-angle fisheyes on rolling shutters that are not
calibrated as stereo pairs in a cuVSLAM-friendly way. cuVSLAM wants a rigid,
synchronized, rectified stereo pair — exactly what a RealSense D435i/D455 provides,
and it's the sensor NVIDIA validates cuVSLAM against. Mount the RealSense rigidly
on the payload rail next to the Jetson (vibration isolation helps on a walking robot).

**Division of labor:**

| Function | Provided by |
|---|---|
| `odom -> body` TF, kinematic odometry | Spot driver (leg kinematics, very smooth) |
| `map -> odom` TF, drift correction, loop closure | cuVSLAM |
| Obstacle sensing | RealSense depth → virtual laser scan |
| Planning + control | Nav2 (MPPI, omnidirectional model) |
| Gait, balance, stairs, low-level safety | Spot's onboard computer (always in charge) |

Nav2 only ever sends body-velocity `Twist` commands. Spot's own controller retains
final authority over foot placement and self-preservation.

## Hardware / software requirements

* Boston Dynamics Spot (robot software supporting Spot SDK 5.x to match spot_ros2)
* NVIDIA Jetson Orin Nano 8 GB, **JetPack 6.x** (Ubuntu 22.04) — Orin Nano Super OK
* Intel RealSense D435i or D455 (must have an IMU), USB 3 cable to the Jetson
* Payload mounting + power (Spot payload port: 24 V; use a regulator for the Jetson)
* ROS 2 **Humble**, Isaac ROS **3.x**, Nav2 ≥ 1.1.9 (for the MPPI controller)

> **Orin Nano note:** it's the smallest Isaac ROS-capable Jetson. cuVSLAM at
> 640×360@60 + Nav2 fits, but enable MAXN power mode (`sudo nvpmodel -m 0`,
> `sudo jetson_clocks`) and don't run RViz on the robot — visualize from a
> laptop on the same ROS_DOMAIN_ID.

## Installation

On the Jetson:

```bash
mkdir -p ~/spot_ws/src && cd ~/spot_ws/src
# copy/clone this package here, then:
bash spot_vslam_nav/scripts/install_jetson.sh
```

The script installs ROS 2 Humble, Isaac ROS Visual SLAM (NVIDIA apt repo), Nav2,
RealSense, builds `spot_ros2` from source (ARM64), and builds this package.
If the packaged Isaac ROS install doesn't match your JetPack version, use
NVIDIA's [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
Docker workflow and run this package inside that container — everything here is
plain ROS 2 and works unchanged.

### Configure

1. **Spot credentials** — edit `config/spot_ros2_config.yaml`
   (hostname `192.168.50.3` when connected via payload Ethernet; set username/password).
2. **Camera mount** — edit `CAMERA_MOUNT_XYZ` / `CAMERA_MOUNT_RPY` at the top of
   `launch/vslam.launch.py` to match where the RealSense sits relative to Spot's
   `body` frame. Get this right to ~1 cm / ~1°; a bad extrinsic is the #1 cause
   of poor SLAM on legged robots.
3. **Network** — give the Jetson a static IP on Spot's payload subnet
   (e.g. `192.168.50.5/24`).

## Running

Everything at once (driver + VSLAM + Nav2 + auto claim/power/stand):

```bash
ros2 launch spot_vslam_nav spot_slam_nav.launch.py \
    spot_config:=$HOME/spot_ws/src/spot_vslam_nav/config/spot_ros2_config.yaml
```

With a named/prefixed Spot:

```bash
ros2 launch spot_vslam_nav spot_slam_nav.launch.py spot_name:=MySpot spot_config:=...
```

Piecewise (useful for debugging):

```bash
ros2 launch spot_driver spot_driver.launch.py config_file:=...      # 1. driver
ros2 launch spot_vslam_nav vslam.launch.py                          # 2. SLAM
ros2 launch spot_vslam_nav nav2.launch.py                           # 3. Nav2
ros2 run spot_vslam_nav spot_auto_bringup                           # 4. stand up
```

### Sending goals

From RViz (on your laptop): use the **Nav2 Goal** tool with `rviz/spot_nav.rviz`.

From the CLI:

```bash
ros2 run spot_vslam_nav goal_cli -- 3.0 1.5 90     # x[m] y[m] yaw[deg] in map frame
ros2 action cancel /navigate_to_pose                # abort
```

Spot walks to the goal, avoiding obstacles seen by the depth camera, with cuVSLAM
keeping the map frame drift-free (loop closures included).

### Verifying the TF tree

```bash
ros2 run tf2_tools view_frames
```

You should see exactly one chain: `map → odom → body → ... → camera_link`.
`map→odom` from `visual_slam`, `odom→body` from the Spot driver. If two nodes
fight over `odom→body`, you forgot `publish_odom_to_base_tf: false` (already set
in `launch/vslam.launch.py`).

## Safety design

* **`cmd_vel_gate`** sits between Nav2 and the driver: clamps velocities, sends a
  zero-twist if Nav2 goes silent for 0.5 s, and hard-blocks motion the moment
  `vslam_health_monitor` reports tracking loss (`/vslam_ok = false`).
* The Spot driver's `cmd_duration: 0.25` means each Twist is a 250 ms motion
  authorization — if the Jetson dies mid-plan, Spot stops within a quarter second.
* Spot's onboard obstacle avoidance and E-stop remain active; keep the tablet or
  an external E-stop endpoint handy per Boston Dynamics' safety requirements.
* Keep first tests at low speed (`vx_max` in `nav2_params.yaml` starts at 0.8 m/s;
  drop to 0.4 for initial runs).

## Tuning notes

**cuVSLAM on a walking robot.** Gait impacts cause motion blur and IMU spikes.
The config enables IMU fusion and image denoising; if you see frequent tracking
loss on gravel/stairs, drop `infra_fps` to 30 and raise `img_jitter_threshold_ms`
to 68 in `config/vslam.yaml`, or slow Spot down.

**IR emitter tradeoff.** The emitter is disabled (`emitter_enabled: 0`) because the
dot pattern corrupts feature tracking. Depth becomes passive-stereo and degrades on
textureless walls. If your environment is feature-poor, a D455 (larger baseline)
or an added texture/light source helps.

**Costmap heights.** `pointcloud_to_laserscan` slices obstacles between 0.10 m and
0.60 m above `body`. Note `body` is ~0.5 m off the ground when standing, so adjust
`min_height`/`max_height` in `launch/nav2.launch.py` if you need to see low
obstacles — or feed the full cloud into a `VoxelLayer` if you have CPU headroom.

**Stairs and rough terrain.** This stack treats the world as 2.5D. Spot can climb
stairs, but Nav2's costmap will mark them as obstacles. For stair missions, use
Spot's GraphNav/Autowalk instead, or add a traversability layer.

**Orin Nano headroom.** If CPU/GPU saturate: lower MPPI `batch_size` (1200 → 800),
drop `controller_frequency` to 10 Hz, depth to 480×270, and keep
`enable_slam_visualization: false` in production.

## Package layout

```
spot_vslam_nav/
├── launch/
│   ├── spot_slam_nav.launch.py   # full stack bringup
│   ├── vslam.launch.py           # RealSense + cuVSLAM + mount TF + health monitor
│   └── nav2.launch.py            # Nav2 + depth->scan + cmd_vel gate
├── config/
│   ├── vslam.yaml                # cuVSLAM params (IMU fusion, denoising)
│   ├── realsense.yaml            # stereo IR @60fps, emitter off, IMU united
│   ├── nav2_params.yaml          # MPPI omni controller, Spot footprint
│   └── spot_ros2_config.yaml     # driver credentials/rates template
├── spot_vslam_nav/
│   ├── spot_auto_bringup.py      # claim -> power_on -> stand
│   ├── cmd_vel_gate.py           # watchdog + VSLAM-loss E-stop + clamp
│   ├── vslam_health_monitor.py   # publishes /vslam_ok
│   └── goal_cli.py               # send NavigateToPose goals
├── rviz/spot_nav.rviz
└── scripts/install_jetson.sh
```

## Known limitations

* Rolling global costmap means no persistent occupancy map across reboots.
  cuVSLAM can save/load its landmark map via the
  `/visual_slam/save_map` and `/visual_slam/load_map_and_localize` services for
  repeatable localization; persistent costmaps would need e.g. `nvblox` or
  `slam_toolbox` layered on top.
* Isaac ROS parameter/topic names occasionally shift between major releases;
  this package targets Isaac ROS 3.x (`num_cameras` / `visual_slam/image_N`
  interface). For Isaac ROS 2.x, remap to `stereo_camera/left/image` etc.
* Verify exact spot_ros2 topic/service names against the version you build
  (`ros2 topic list`, `ros2 service list`) — the repo moves quickly.
* This has to be treated as a starting point: **always test with the tablet
  E-stop ready, in an open area, at low speed first.**

## License

MIT (this package). Isaac ROS, Nav2, and spot_ros2 carry their own licenses.
