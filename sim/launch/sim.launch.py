"""
Simulation bringup: champ-based Spot in Gazebo Sim + D435i sensor bridge.

Includes the upstream spot_gazebo_ros2 bringup (Gazebo world, champ
controller, robot_state_publisher, spawn) and adds the ros_gz bridge
for the injected D435i topics.

TF note (mirrors the real robot exactly):
  - champ publishes odom -> base_link  (stands in for Spot's kinematic odom)
  - cuVSLAM publishes map -> odom      (identical to hardware deployment)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_upstream_bringup():
    """Locate the champ Spot bringup launch file, tolerant to repo layout."""
    for pkg in ("spot_bringup", "champ_bringup", "spot_gazebo"):
        try:
            share = get_package_share_directory(pkg)
        except Exception:
            continue
        for name in ("spot.gazebo.launch.py", "spot_gazebo.launch.py", "gazebo.launch.py",
                     "bringup.launch.py", "spawn_robot.launch.py", "champ_bringup.launch.py"):
            path = os.path.join(share, "launch", name)
            if os.path.exists(path):
                return path
    raise RuntimeError(
        "Could not find the upstream Spot simulation launch file. "
        "Check the spot_gazebo_ros2 repo layout and edit sim.launch.py."
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("spot_sim_slam")
    headless = LaunchConfiguration("headless")

    upstream = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(find_upstream_bringup()),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="d435i_bridge",
        parameters=[
            {
                "config_file": os.path.join(pkg_share, "config", "gz_bridge.yaml"),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    # The upstream bringup starts gz_sim.launch.py without "-r" in gz_args
    # (hardcoded, not exposed as an overridable launch argument), so the
    # world boots paused and nothing ever steps physics. Unpause it directly
    # via the world control service once it comes up. "simple_tunnel" is the
    # upstream launch file's default world_file.
    unpause_world = ExecuteProcess(
        cmd=[
            "bash", "-c",
            # `ign service` exits 0 even when the call times out, so success
            # has to be detected from its output rather than its exit code.
            # With real camera sensors loading (software-rendered, no GPU
            # passthrough on this container), world init can take ~2 minutes,
            # so budget for that: 90 attempts * up to ~3s each.
            "for i in $(seq 1 90); do "
            "out=$(ign service -s /world/simple_tunnel/control "
            "--reqtype ignition.msgs.WorldControl --reptype ignition.msgs.Boolean "
            "--timeout 2000 --req 'pause: false' 2>&1); "
            "echo \"$out\"; "
            "echo \"$out\" | grep -q 'data: true' && exit 0; "
            "sleep 1; done",
        ],
        output="screen",
    )

    # Gazebo's own depth_camera point-cloud generation has a bug in this
    # version: every point comes out with x >= 0 regardless of which image
    # column it came from, even though the depth image and its camera_info
    # are both correctly centered/symmetric. Generate the point cloud
    # ourselves from those (verified-good) inputs instead of trusting
    # Gazebo's "/camera/depth/points" -> we never bridge that topic at all
    # (see gz_bridge.yaml).
    #
    # depth_image_proc's point_cloud_xyz_node was tried first, but its
    # image/camera_info synchronizer needs matching timestamps and our
    # image and camera_info come from independent Gazebo publish schedules
    # with a consistent ~60-70ms gap -- they never sync. Our own node just
    # caches the (static) intrinsics and applies them to each depth frame
    # directly, no message-level sync needed.
    depth_to_points = Node(
        package="spot_sim_slam",
        executable="depth_to_points",
        name="depth_to_points",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            upstream,
            bridge,
            unpause_world,
            depth_to_points,
        ]
    )
