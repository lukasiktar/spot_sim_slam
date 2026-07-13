# Using the simulator

Step-by-step operating guide for `spot_sim_slam`: starting the stack, driving
the robot, watching it in RViz, and — the part most people actually come here
for — recording a cuVSLAM map and reusing it later. See [README.md](README.md)
for the architecture overview and what's simulated.

## 1. One-time setup

```bash
xhost +local:docker        # needed after every X session restart, not just once
docker compose build       # ~30-45 min the first time
```

## 2. Start the stack

Bring services up one at a time (recommended the first few times, so you can
watch each one's logs) or all at once.

```bash
docker compose up sim -d
```

`sim` takes a while to become useful — budget **up to ~2 minutes** the first
time. Two things happen back to back:

1. Gazebo boots **paused** (the upstream launch file hardcodes this) and
   `sim.launch.py` polls `ign service .../control` in a retry loop until it
   confirms the unpause actually landed (watch for `data: true` in
   `docker logs spot_sim`).
2. If you don't have GPU passthrough working for the `sim` container, camera
   rendering falls back to software rasterization (`libGL error: ... iris`
   messages are expected and harmless) — this is what makes early startup slow.

Once it's up:

```bash
docker compose up vslam -d   # needs an NVIDIA GPU; use slam_cpu (--profile cpu) otherwise
docker compose up nav -d
docker compose up rviz -d
```

Or everything at once: `docker compose --profile gpu up -d` (GPU) /
`docker compose --profile cpu up sim slam_cpu nav rviz` (no GPU).

Sanity check the TF tree is complete before doing anything else:

```bash
docker exec spot_sim bash -lc \
  "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && \
   ros2 run tf2_ros tf2_echo map base_link"
```

You want a real transform back, not "waiting for" spam. If `map` doesn't
exist yet, cuVSLAM is still initializing (loading GXF extensions, warming up
the GPU, creating the tracker — a few seconds) or the `vslam` container isn't
up yet.

## 3. Watch it in RViz

`docker compose up rviz -d` opens a window on your host X display using
`spot_vslam_nav/rviz/spot_nav.rviz`. You should see:

- **RobotModel** — the actual Spot mesh, standing on the ground, driven by
  TF (not the SDF `robot_description` topic — RViz can't parse that; see the
  comment in the `.rviz` file if you're curious why a separate URDF file is
  used).
- **TF** — coordinate frame axes.
- **VSLAM Odometry** / **VSLAM Landmarks** — cuVSLAM's live tracking output.
- **Local Costmap** / **Global Costmap** — Nav2's rolling-window obstacle
  layers, built from the D435i depth point cloud.
- **Global Plan** / **Local Plan** — appear once you send a nav goal.

The Orbit camera's Target Frame is `base_link`, so the view follows the robot
instead of orbiting wherever `map`'s origin happens to be (which is nowhere
near where Spot spawns).

## 4. Drive the robot

**Manual teleop** (works with just `sim` up, no `nav` needed):

```bash
docker exec -it spot_sim bash -lc \
  "source /ws/install/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard"
```

**Autonomous navigation** (needs `sim` + `vslam` + `nav`):

```bash
docker exec -it spot_nav bash -lc \
  "source /ws/install/setup.bash && ros2 run spot_vslam_nav goal_cli -- 3.0 1.0 0"
# x[m] y[m] yaw[deg], in the map frame
```

Or click "2D Goal Pose" in RViz's Navigation 2 panel.

Either way, driving Spot around is what builds up the local/global costmaps
and grows cuVSLAM's internal map — there's no separate "mapping mode" to
turn on. If you just want SLAM+mapping without commanding paths, teleop is
enough; the costmap fills in as you go regardless of what's driving.

## 5. Record (save) the map for later

cuVSLAM (`enable_localization_n_mapping: true` in `vslam_sim.launch.py`)
keeps a map database internally the whole time it runs, and exposes it
through plain services — no separate "map saver" tool needed:

```
/visual_slam/save_map          (isaac_ros_visual_slam_interfaces/srv/FilePath)
/visual_slam/load_map           (isaac_ros_visual_slam_interfaces/srv/FilePath)
/visual_slam/localize_in_map    (isaac_ros_visual_slam_interfaces/srv/LocalizeInMap)
/visual_slam/reset
/visual_slam/set_slam_pose
/visual_slam/get_all_poses
```

Save whenever you've driven around enough of the environment to be useful
(more coverage → better relocalization later):

```bash
docker exec spot_vslam bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 service call /visual_slam/save_map isaac_ros_visual_slam_interfaces/srv/FilePath \"{file_path: '/maps/simple_tunnel'}\""
```

`/maps` inside the `vslam` container is bind-mounted from `./maps` in this
repo (see `docker-compose.yml`) specifically so the map **survives**
`docker compose down` — without that mount, everything you saved would be
deleted the moment the container is removed, since `file_path` is just a
path inside the container's own (ephemeral) filesystem. A response of
`success: True` means the map database (a small `data.mdb` file plus
metadata) landed under `maps/simple_tunnel/` on your host.

Pick a name that describes the world/run, e.g. `maps/simple_tunnel`,
`maps/simple_tunnel_2026-07-13`. Nothing under `maps/` other than
`.gitkeep` is tracked by git (see `.gitignore`) — copy out any map you want
to keep permanently.

## 6. Reuse a saved map in a later session

Two options, depending on whether you want cuVSLAM to also localize you into
the map immediately:

**Load only** (map becomes available for future localization, no pose set):

```bash
docker exec spot_vslam bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 service call /visual_slam/load_map isaac_ros_visual_slam_interfaces/srv/FilePath \"{file_path: '/maps/simple_tunnel'}\""
```

**Load + localize** (also give cuVSLAM a rough starting pose hint so it can
relocalize faster — use this if you're restarting near where you left off):

```bash
docker exec spot_vslam bash -lc \
  "source /opt/ros/humble/setup.bash && \
   ros2 service call /visual_slam/localize_in_map isaac_ros_visual_slam_interfaces/srv/LocalizeInMap \
   \"{map_folder_path: '/maps/simple_tunnel', pose_hint: {position: {x: 80.6, y: 33.6, z: 0.48}, orientation: {w: 1.0}}}\""
```

Call either of these right after `vslam` comes up, before you start driving —
cuVSLAM builds on top of whatever's loaded rather than starting from a blank
map. Watch `/visual_slam/tracking/odometry` and the `map` TF in RViz: once
relocalized, new landmarks should register against the previously-mapped
geometry instead of drifting off into a fresh coordinate frame.

## 7. Shut down

```bash
docker compose down
```

This removes the containers but **not** anything you saved under `./maps` —
that directory lives on the host, independent of container lifecycle.

## Troubleshooting quick reference

| Symptom | Likely cause |
|---|---|
| rviz window doesn't appear / X11 errors | Run `xhost +local:docker` again — needed after every X session restart |
| `map` frame doesn't exist yet | cuVSLAM still initializing (~5s) or `vslam` container not up |
| Robot floating/misplaced in RViz | Make sure you're on a rebuilt image with the current `spot_nav.rviz` / `model.urdf` mesh-path fix — a stale image predates it |
| Local costmap only shows one side | Confirm `depth_to_points` node is actually running: `docker exec spot_sim ... ros2 node list \| grep depth_to_points` |
| Robot falls over / physics jitter | CPU contention — running `sim`+`vslam`+`nav` together is heavy without full GPU passthrough; close other apps or check `nvidia-container-toolkit` graphics capability |
