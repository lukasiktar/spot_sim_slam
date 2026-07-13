"""
CPU fallback SLAM (no NVIDIA GPU required): RTAB-Map stereo mode consuming
the SAME topics as cuVSLAM and publishing the SAME map->odom TF.

This is an interface-compatible stand-in — it lets you validate the Spot sim,
Nav2 tuning, the cmd_vel gate, and the whole TF contract on any laptop.
It is NOT cuVSLAM; final validation of SLAM quality still needs the GPU
container or the Jetson itself.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common = {
        "use_sim_time": True,
        "frame_id": "base_link",
        "odom_frame_id": "odom",
        "map_frame_id": "map",
        "subscribe_stereo": True,
        "subscribe_depth": False,
        "approx_sync": True,
        # publish map->odom only (odom->base_link comes from champ),
        # identical to the cuVSLAM configuration:
        "publish_tf": True,
    }

    remaps = [
        ("left/image_rect", "/camera/infra1/image_rect_raw"),
        ("left/camera_info", "/camera/infra1/camera_info"),
        ("right/image_rect", "/camera/infra2/image_rect_raw"),
        ("right/camera_info", "/camera/infra2/camera_info"),
        ("imu", "/camera/imu"),
        ("odom", "/spot/odometry"),
    ]

    return LaunchDescription(
        [
            Node(
                package="rtabmap_slam",
                executable="rtabmap",
                name="rtabmap",
                output="screen",
                parameters=[
                    common,
                    {
                        # Use external (champ) odometry; rtabmap corrects it.
                        "subscribe_odom_info": False,
                        "Reg/Force3DoF": "true",
                        "Grid/FromDepth": "false",
                    },
                ],
                remappings=remaps,
                arguments=["--delete_db_on_start"],
            ),
        ]
    )
