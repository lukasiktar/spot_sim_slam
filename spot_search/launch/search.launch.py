"""The mission controller.

Runs alongside Nav2 (it drives the navigate_to_pose and spin action servers)
and waits for a target count on /search/start.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    base_frame = LaunchConfiguration("base_frame")
    map_frame = LaunchConfiguration("map_frame")
    use_sim_time = LaunchConfiguration("use_sim_time")

    args = [
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="'base_link' in sim (champ), 'body' on the real Spot.",
        ),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
    ]

    mission = Node(
        package="spot_search",
        executable="search_mission",
        name="search_mission",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_frame": map_frame,
                "base_frame": base_frame,
                "min_frontier_cells": 8,
                "robot_radius": 0.45,
                "frontier_size_weight": 0.15,
                "blacklist_radius": 2.0,
                "goal_timeout": 180.0,
                "scan_segments": 2,
                "scan_time_allowance": 30.0,
            }
        ],
    )

    return LaunchDescription(args + [mission])
