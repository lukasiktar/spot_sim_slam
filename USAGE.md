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
docker compose up vslam -d       # cuVSLAM: map->odom. NVIDIA GPU required
docker compose up nvblox -d      # nvblox: 3D reconstruction -> /map. GPU required
docker compose up nav -d
docker compose up perception -d  # YOLO person detector + map pins (CPU)
docker compose up search -d      # search mission controller
docker compose up rviz -d
```

Or everything at once: `docker compose --profile gpu up -d` (GPU) /
`docker compose --profile cpu up sim slam_cpu nav rviz` (no GPU).

`perception` is slow to become useful the first time only in the sense that
it loads PyTorch; the YOLO weights are baked into the image, so it never
reaches for the network mid-mission.

Before starting a search, confirm nvblox is actually producing a map —
everything downstream (Nav2's static layer, frontiers, pins) hangs off it:

```bash
docker exec spot_nvblox bash -lc \
  "source /ws/install/setup.bash && ros2 topic hz /map"
```

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
- **Local Costmap** — Nav2's rolling-window obstacle layer, built live from
  the D435i depth point cloud.
- **Global Costmap** — no longer rolling: it now sizes itself to nvblox's map.
- **nvblox 2D Map** — the ESDF slice as an occupancy grid. This is the "2D
  map" the person pins land on, and the surface the explorer reads frontiers
  from. Grey is unknown, and shrinking grey is the mission making progress.
- **nvblox ESDF** — the same data as a 3D point cloud. Off by default; useful
  when the 2D slice looks wrong and you want to see what height it was cut at.
- **Person Pins** — a red pole and labelled head per confirmed person.
- **Frontiers** — the candidate goals the explorer is choosing between.
- **Person Detections** — the detector's annotated camera view.
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

## 5. Send Spot to find people

This is the autonomous search: you say how many people are out there, Spot
works out where to look.

The bundled `simple_tunnel` world already contains two "Rescue Randy" victim
models — one sitting at `(41.3, 32.4)` and one lying at `(69.6, 38.1)`, with
Spot spawning at `(80.6, 33.6)`. So the natural first mission is two:

```bash
docker exec -it spot_search bash -lc \
  "source /ws/install/setup.bash && ros2 run spot_search search_cli 2"
```

The CLI publishes the request and then follows along, printing the mission
state and the pin count until it finishes:

```
Mission sent: find 2 person(s).
  [SCANNING] scanning for people (0/2 found)
  [EXPLORING] exploring towards (72.4, 35.1) -- 0/2 found
  pins on the map: 1
  [EXPLORING] exploring towards (58.9, 33.8) -- 1/2 found
  pins on the map: 2
  [DONE] complete: found 2/2
```

What it is doing between those lines:

1. **Scan** — a full turn in place, in two halves, via Nav2's `spin`
   behaviour. The colour camera only sees 69°, so without this Spot would
   walk straight past anyone standing off to the side.
2. **Explore** — read `/map`, find the frontiers (free cells that touch
   unknown cells), score them by `size × 0.15 − distance`, and send the winner
   to Nav2 as a `navigate_to_pose` goal.
3. **Scan again** on arrival, and repeat.

It stops when the pin count reaches your target, or when no frontier is left —
in which case it says so rather than pretending to have succeeded:

```
[EXHAUSTED] exhausted: found 1/2, nothing left to explore
```

You can watch the same thing from outside the CLI:

```bash
ros2 topic echo /search/status          # mission state
ros2 topic echo /found_persons/poses    # confirmed pin coordinates
ros2 topic echo /found_persons/count
```

and stop it early with:

```bash
ros2 service call /search/stop std_srvs/srv/Trigger
```

Pins persist across missions, and the mission counts *all* confirmed pins
against its target — so asking for 2 again right after a successful run
completes instantly. Clear them first to re-run the search properly:

```bash
ros2 service call /found_persons/reset std_srvs/srv/Trigger
```

### Where the pins show up

In **RViz**, the `Person Pins` display draws a red pole and labelled head at
each confirmed person, on top of the `nvblox 2D Map` display. `Frontiers`
shows the candidate goals the explorer is choosing between, and
`Person Detections` is the detector's annotated camera view.

On **disk**, `person_map` keeps `maps/found_persons.json` current as pins are
confirmed:

```json
{
  "frame_id": "map",
  "count": 2,
  "persons": [
    {"id": 1, "x": 69.51, "y": 38.22, "z": 0.31, "sightings": 11, "score": 0.72}
  ]
}
```

And you can render the map-with-pins as an image once the run is over:

```bash
docker exec spot_perception bash -lc \
  "source /ws/install/setup.bash && \
   ros2 run spot_search save_search_map --output /maps/search_result.png"
```

### Tuning the search

| Symptom | Knob |
|---|---|
| Walks past people | raise `scan_segments` (more, shorter turns) on `search_mission` |
| False pins on debris | raise `min_hits` or `confidence` on the perception launch |
| Two pins for one person | raise `association_radius` on `person_map` |
| Explores timidly / revisits | lower `frontier_size_weight` (prefers near over large) |
| Gives up too early | raise `goal_timeout`, or lower `min_frontier_cells` |

## 6. Record (save) the map for later

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

### This is a separate map from nvblox's

cuVSLAM's save above persists its own *sparse* map — visual feature
landmarks, used for relocalization. It has nothing to do with nvblox's
*dense* volumetric reconstruction (the TSDF/ESDF behind `/map` and the 2D
slice you see in RViz) — that's a second, independent map with its own
save/load:

```
/nvblox_node/save_map   (nvblox_msgs/srv/FilePath)
/nvblox_node/load_map   (nvblox_msgs/srv/FilePath)
/nvblox_node/save_ply   (nvblox_msgs/srv/FilePath)   -- mesh, for viewing in Blender/MeshLab etc.
```

```bash
docker exec spot_nvblox bash -lc \
  "source /ws/install/setup.bash && \
   ros2 service call /nvblox_node/save_map nvblox_msgs/srv/FilePath \"{file_path: '/maps/simple_tunnel_nvblox'}\""
```

Same `/maps` mount, same survives-`docker compose down` guarantee, same
naming convention — just a different (`.nvblx`) file. Without a saved
nvblox map, restarting `nvblox` throws away the reconstruction and it has to
rebuild `/map` from scratch as Spot re-explores.

## 7. Reuse a saved map in a later session

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

Load nvblox's map the same way, right after `nvblox` comes up:

```bash
docker exec spot_nvblox bash -lc \
  "source /ws/install/setup.bash && \
   ros2 service call /nvblox_node/load_map nvblox_msgs/srv/FilePath \"{file_path: '/maps/simple_tunnel_nvblox'}\""
```

Do this before Spot starts moving — nvblox integrates new depth data on top
of whatever's loaded, so loading mid-mission just means the freshly-explored
area coexists a little awkwardly with the older reconstruction rather than
actively conflicting with it.

## 8. Shut down

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
