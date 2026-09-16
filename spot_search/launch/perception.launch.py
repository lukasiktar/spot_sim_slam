"""Person perception: detect people in RGB, pin them on the map.

"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    map_frame = LaunchConfiguration("map_frame")
    confidence = LaunchConfiguration("confidence")
    min_hits = LaunchConfiguration("min_hits")
    use_sim_time = LaunchConfiguration("use_sim_time")

    args = [
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "confidence",
            default_value="0.35",
            description="YOLO score floor. Lower than a typical deployment "
            "because the sim's victim models are rendered at 640x480 and score "
            "below what a real camera would give; person_map's min_hits is "
            "what rejects the resulting false positives.",
        ),
        DeclareLaunchArgument(
            "min_hits",
            default_value="4",
            description="Sightings before a track becomes a pin.",
        ),
    ]

    detector = Node(
        package="spot_search",
        executable="person_detector",
        name="person_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "model_path": "/models/yolov8n.pt",
                "confidence": ParameterValue(confidence, value_type=float),
                "max_rate_hz": 4.0,
                "max_range": 8.0,
                "target_frame": map_frame,
                "depth_is_aligned_to_color": False,
                "publish_debug_image": True,
            }
        ],
    )

    person_map = Node(
        package="spot_search",
        executable="person_map",
        name="person_map",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "map_frame": map_frame,
                "min_hits": ParameterValue(min_hits, value_type=int),
                "association_radius": 1.5,
                "output_file": "/maps/found_persons.json",
            }
        ],
    )

    return LaunchDescription(args + [detector, person_map])
