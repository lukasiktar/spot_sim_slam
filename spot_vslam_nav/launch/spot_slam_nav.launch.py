"""
Full-stack bringup on the Jetson Orin Nano:

    spot_ros2 driver  +  Isaac ROS Visual SLAM  +  Nav2  +  auto-bringup

Usage:
    ros2 launch spot_vslam_nav spot_slam_nav.launch.py \
        spot_config:=/path/to/spot_ros2_config.yaml \
        [spot_name:=MySpot] [launch_driver:=true] [auto_stand:=true]

If the spot_ros2 driver is already running elsewhere (e.g. Spot CORE),
set launch_driver:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("spot_vslam_nav")

    spot_name = LaunchConfiguration("spot_name")
    spot_config = LaunchConfiguration("spot_config")
    launch_driver = LaunchConfiguration("launch_driver")
    auto_stand = LaunchConfiguration("auto_stand")
    launch_rviz = LaunchConfiguration("launch_rviz")

    # When spot_name is empty, frames/topics are unprefixed.
    has_name = PythonExpression(["'", spot_name, "' != ''"])
    base_frame = PythonExpression(
        ["'", spot_name, "/body' if '", spot_name, "' else 'body'"]
    )
    odom_topic = PythonExpression(
        ["'/", spot_name, "/odometry' if '", spot_name, "' else '/odometry'"]
    )
    cmd_vel_topic = PythonExpression(
        ["'/", spot_name, "/cmd_vel' if '", spot_name, "' else '/cmd_vel'"]
    )

    args = [
        DeclareLaunchArgument("spot_name", default_value=""),
        DeclareLaunchArgument(
            "spot_config",
            default_value=os.path.join(
                pkg_share, "config", "spot_ros2_config.yaml"
            ),
            description="spot_ros2 driver config (credentials, hostname...).",
        ),
        DeclareLaunchArgument("launch_driver", default_value="true"),
        DeclareLaunchArgument("auto_stand", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
    ]

    # ------------------------------------------------------------------
    # 1. spot_ros2 driver
    # ------------------------------------------------------------------
    spot_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("spot_driver"), "launch", "spot_driver.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": spot_config,
            "spot_name": spot_name,
            "launch_rviz": "False",
            "publish_point_clouds": "False",  # we use the RealSense instead
        }.items(),
        condition=IfCondition(launch_driver),
    )

    # ------------------------------------------------------------------
    # 2. Visual SLAM (delayed so the driver's TF tree exists first)
    # ------------------------------------------------------------------
    vslam_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "vslam.launch.py")
                ),
                launch_arguments={
                    "spot_name": spot_name,
                    "base_frame": base_frame,
                }.items(),
            )
        ],
    )

    # ------------------------------------------------------------------
    # 3. Nav2 (delayed until VSLAM publishes map->odom)
    # ------------------------------------------------------------------
    nav2_launch = TimerAction(
        period=15.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "nav2.launch.py")
                ),
                launch_arguments={
                    "base_frame": base_frame,
                    "odom_topic": odom_topic,
                    "spot_cmd_vel": cmd_vel_topic,
                }.items(),
            )
        ],
    )

    # ------------------------------------------------------------------
    # 4. Claim -> power on -> stand (optional)
    # ------------------------------------------------------------------
    auto_bringup = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="spot_vslam_nav",
                executable="spot_auto_bringup",
                name="spot_auto_bringup",
                parameters=[{"spot_name": spot_name}],
                output="screen",
                condition=IfCondition(auto_stand),
            )
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "spot_nav.rviz")],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        args + [spot_driver_launch, vslam_launch, nav2_launch, auto_bringup, rviz]
    )
