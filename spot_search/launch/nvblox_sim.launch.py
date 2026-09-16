"""nvblox 3D reconstruction for the simulated Spot + D435i.

nvblox integrates the depth stream into a TSDF, derives a Euclidean signed
distance field from it, and publishes a horizontal slice of that ESDF as the
robot's 2D view of the world. `map_slice_to_occupancy` then turns that slice
into a standard OccupancyGrid on /map, which is simultaneously Nav2's static
layer, the explorer's source of unknown space, and the canvas the person pins
are drawn on.

nvblox reconstructs in `global_frame`. Building in `map`
keeps the reconstruction globally consistent with cuVSLAM's corrections, at
the cost of a visible discontinuity whenever a loop closes.

The default here is `odom`, not `map`, specifically for sim: this world's
tunnel repeats identical tiles every ~20m, which aliases cuVSLAM's stereo
matching badly enough to put map->odom off by tens of metres, and that
pollutes anything built in `map`. `odom` here is Gazebo's own ground-truth
pose (see sim/spot_sim_slam/odom_rerooter.py) -- trustworthy in a way a real
robot's kinematic odom never is -- so building in it sidesteps the aliasing
problem entirely. 

The slice heights are given in the GLOBAL frame, not relative to the floor.
The odom/map origin sits at Spot's spawn pose, roughly `floor_offset` above
the ground, so the defaults below bracket knee-to-chest height.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    global_frame = LaunchConfiguration("global_frame")
    voxel_size = LaunchConfiguration("voxel_size")
    slice_min_height = LaunchConfiguration("slice_min_height")
    slice_max_height = LaunchConfiguration("slice_max_height")
    slice_height = LaunchConfiguration("slice_height")
    max_integration_distance = LaunchConfiguration("max_integration_distance")

    args = [
        DeclareLaunchArgument("global_frame", default_value="odom"),
        DeclareLaunchArgument(
            "voxel_size",
            default_value="0.08",
            description="Metres per voxel. 0.05 is the nvblox default; this "
            "world is ~100 m across and the coarser grid keeps GPU memory and "
            "ESDF update time reasonable.",
        ),
        DeclareLaunchArgument(
            "slice_min_height",
            default_value="-0.4",
            description="Bottom of the 2D ESDF slice, in the global frame. "
            "Empirically verified against this world/spawn: with map's "
            "origin at Spot's spawn pose, real wall/floor geometry shows up "
            "in this negative-leaning band, not a floor-relative "
            "knee-to-chest range -- confirm with "
            "`ros2 run tf2_ros tf2_echo map base_link` plus /map cell "
            "counts before changing these.",
        ),
        DeclareLaunchArgument("slice_max_height", default_value="0.3"),
        DeclareLaunchArgument("slice_height", default_value="-0.2"),
        DeclareLaunchArgument(
            "max_integration_distance",
            default_value="6.0",
            description="Depth beyond this is discarded. The simulated D435i "
            "clips at 10 m but its far returns are too noisy to reconstruct.",
        ),
    ]

    nvblox = Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "global_frame": global_frame,
                "voxel_size": ParameterValue(voxel_size, value_type=float),
                "mapping_type": "static_tsdf",
                "num_cameras": 1,
                "use_tf_transforms": True,
                "use_topic_transforms": False,
                "publish_esdf_distance_slice": True,
                "esdf_slice_min_height": ParameterValue(
                    slice_min_height, value_type=float
                ),
                "esdf_slice_max_height": ParameterValue(
                    slice_max_height, value_type=float
                ),
                "esdf_slice_height": ParameterValue(slice_height, value_type=float),
                # The sim's depth camera runs at 15 Hz
                "integrate_depth_rate_hz": 15.0,
                "integrate_color_rate_hz": 5.0,
                "update_esdf_rate_hz": 2.0,
                "update_mesh_rate_hz": 1.0,
                "static_mapper.projective_integrator_max_integration_distance_m":
                    ParameterValue(max_integration_distance, value_type=float),
                "static_mapper.esdf_integrator_max_distance_m": 4.0,
            }
        ],
        remappings=[
            ("camera_0/depth/image", "/camera/depth/image_rect_raw"),
            ("camera_0/depth/camera_info", "/camera/depth/camera_info"),
            ("camera_0/color/image", "/camera/color/image_raw"),
            ("camera_0/color/camera_info", "/camera/color/camera_info"),
        ],
    )

    slice_to_map = Node(
        package="spot_search",
        executable="map_slice_to_occupancy",
        name="map_slice_to_occupancy",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "slice_topic": "/nvblox_node/static_map_slice",
                "map_topic": "/map",
            }
        ],
    )

    return LaunchDescription(args + [nvblox, slice_to_map])
