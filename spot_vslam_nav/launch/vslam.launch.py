"""
Isaac ROS Visual SLAM (cuVSLAM) bringup for Spot.

Default sensor: Intel RealSense D435i / D455 mounted on Spot (rigidly, on the
payload rail next to the Jetson Orin Nano). The stereo IR pair (infra1/infra2)
with the IR emitter DISABLED is fed to cuVSLAM, together with the IMU.

TF contract (critical!):
  - The spot_ros2 driver already publishes odom -> body (kinematic odometry).
  - cuVSLAM is therefore configured to publish ONLY map -> odom
    (publish_map_to_odom_tf: true, publish_odom_to_base_tf: false).
  - A static transform body -> camera_link describes the physical mount.
    Adjust CAMERA_MOUNT_XYZ / RPY below to match your payload.

Tested against: Isaac ROS 3.2 (JetPack 6.x, ROS 2 Humble), realsense-ros 4.55+.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# ---------------------------------------------------------------------------
# EDIT: physical mount of the RealSense on Spot's body frame.
# x forward, y left, z up (meters); roll/pitch/yaw (radians).
# ---------------------------------------------------------------------------
CAMERA_MOUNT_XYZ = ["0.30", "0.0", "0.12"]
CAMERA_MOUNT_RPY = ["0.0", "0.0", "0.0"]


def generate_launch_description():
    pkg_share = get_package_share_directory("spot_vslam_nav")

    spot_name = LaunchConfiguration("spot_name")
    launch_realsense = LaunchConfiguration("launch_realsense")

    args = [
        DeclareLaunchArgument(
            "spot_name",
            default_value="",
            description="Spot name / TF prefix used by the spot_ros2 driver "
            "(leave empty if the driver runs without a prefix).",
        ),
        DeclareLaunchArgument(
            "launch_realsense",
            default_value="true",
            description="Launch the RealSense driver in the same container.",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="body",
            description="Robot base frame published by the spot driver "
            "(prefixed automatically if spot_name is set, e.g. 'MySpot/body').",
        ),
        DeclareLaunchArgument(
            "odom_frame",
            default_value="odom",
            description="Odometry frame published by the spot driver.",
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
    ]

    base_frame = LaunchConfiguration("base_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_frame = LaunchConfiguration("map_frame")

    # ------------------------------------------------------------------
    # RealSense driver 
    # ------------------------------------------------------------------
    realsense_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name="camera",
        namespace="camera",
        parameters=[os.path.join(pkg_share, "config", "realsense.yaml")],
    )

    # ------------------------------------------------------------------
    # cuVSLAM
    # ------------------------------------------------------------------
    visual_slam_node = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam",
        parameters=[
            os.path.join(pkg_share, "config", "vslam.yaml"),
            {
                "base_frame": base_frame,
                "odom_frame": odom_frame,
                "map_frame": map_frame,
                # Spot's driver owns odom->body; cuVSLAM only corrects drift:
                "publish_map_to_odom_tf": True,
                "publish_odom_to_base_tf": False,
                "camera_optical_frames": [
                    "camera_infra1_optical_frame",
                    "camera_infra2_optical_frame",
                ],
                "imu_frame": "camera_imu_optical_frame",
            },
        ],
        remappings=[
            # Isaac ROS >= 3.0 multicam interface
            ("visual_slam/image_0", "/camera/infra1/image_rect_raw"),
            ("visual_slam/camera_info_0", "/camera/infra1/camera_info"),
            ("visual_slam/image_1", "/camera/infra2/image_rect_raw"),
            ("visual_slam/camera_info_1", "/camera/infra2/camera_info"),
            ("visual_slam/imu", "/camera/imu"),
        ],
    )

    container = ComposableNodeContainer(
        name="vslam_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[visual_slam_node],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )

    realsense_container = ComposableNodeContainer(
        name="realsense_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[realsense_node],
        output="screen",
        condition=IfCondition(launch_realsense),
    )

    # ------------------------------------------------------------------
    # Static TF: Spot body -> camera_link (physical mount)
    # ------------------------------------------------------------------
    body_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="body_to_camera_tf",
        arguments=CAMERA_MOUNT_XYZ
        + CAMERA_MOUNT_RPY
        + [base_frame, "camera_link"],
        output="screen",
    )

    # ------------------------------------------------------------------
    # Health monitor: watches cuVSLAM tracking state, stops Spot on loss
    # ------------------------------------------------------------------
    health_monitor = Node(
        package="spot_vslam_nav",
        executable="vslam_health_monitor",
        name="vslam_health_monitor",
        parameters=[{"spot_name": spot_name}],
        output="screen",
    )

    return LaunchDescription(
        args + [realsense_container, container, body_to_camera_tf, health_monitor]
    )
