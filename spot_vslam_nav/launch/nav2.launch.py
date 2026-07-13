"""
Nav2 bringup for Spot with external (cuVSLAM) localization.

- No AMCL / map_server: map->odom comes from Isaac ROS Visual SLAM.
- Depth pointcloud is converted to a virtual LaserScan for the costmaps
  (much cheaper on the Orin Nano than raw PointCloud2 observation buffers).
- Nav2's cmd_vel is routed:  controller -> velocity_smoother -> cmd_vel_gate
  -> /<spot_name>/cmd_vel (the spot_ros2 driver's topic).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("spot_vslam_nav")

    params_file = LaunchConfiguration("params_file")
    base_frame = LaunchConfiguration("base_frame")
    odom_topic = LaunchConfiguration("odom_topic")
    spot_cmd_vel = LaunchConfiguration("spot_cmd_vel")
    scan_transform_tolerance = LaunchConfiguration("scan_transform_tolerance")

    args = [
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="body",
            description="Spot base frame ('body' or '<spot_name>/body').",
        ),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odometry",
            description="Kinematic odometry topic from spot_ros2 "
            "(namespaced if the driver uses a spot_name).",
        ),
        DeclareLaunchArgument(
            "spot_cmd_vel",
            default_value="/cmd_vel",
            description="cmd_vel topic the spot_ros2 driver subscribes to "
            "(e.g. /MySpot/cmd_vel when namespaced).",
        ),
        DeclareLaunchArgument(
            "scan_transform_tolerance",
            default_value="0.05",
            description="depth_to_scan's TF wait tolerance. The real D435i "
            "publishes at a steady ~30 Hz so 50 ms is plenty; a software-"
            "rendered sim with jittery sensor timing needs it relaxed "
            "(e.g. 0.5) or every cloud gets silently dropped.",
        ),
    ]

    # Rewrite frame names in nav2_params.yaml so a tf_prefix'd Spot works
    # without editing YAML by hand.
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "robot_base_frame": base_frame,
            "odom_topic": odom_topic,
        },
        convert_types=True,
    )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    nodes = [
        # Depth cloud -> virtual 2D scan for costmaps
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="depth_to_scan",
            parameters=[
                {
                    "target_frame": base_frame,
                    "transform_tolerance": ParameterValue(scan_transform_tolerance, value_type=float),
                    # Ignore the floor and anything above Spot's back
                    "min_height": 0.10,
                    "max_height": 0.60,
                    "angle_min": -0.7854,   # RealSense H-FOV ~90 deg
                    "angle_max": 0.7854,
                    "angle_increment": 0.0087,
                    "scan_time": 0.1,
                    "range_min": 0.3,
                    "range_max": 5.5,
                    "use_inf": True,
                }
            ],
            remappings=[
                ("cloud_in", "/camera/depth/color/points"),
                ("scan", "/scan"),
            ],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[configured_params],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            output="screen",
            parameters=[configured_params],
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            output="screen",
            parameters=[configured_params],
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            output="screen",
            parameters=[configured_params],
            remappings=[
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel_smoothed"),
            ],
        ),
        # Safety gate: forwards cmd_vel to Spot only when VSLAM is healthy
        # and publishes zero-twist on E-stop conditions.
        Node(
            package="spot_vslam_nav",
            executable="cmd_vel_gate",
            name="cmd_vel_gate",
            output="screen",
            parameters=[{"output_topic": spot_cmd_vel}],
            remappings=[("cmd_vel_in", "cmd_vel_smoothed")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"autostart": True, "node_names": lifecycle_nodes},
            ],
        ),
    ]

    return LaunchDescription(args + nodes)
