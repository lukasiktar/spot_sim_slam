"""
Isaac ROS Visual SLAM for the SIMULATED Spot + D435i.

Differences vs. the on-robot launch (spot_vslam_nav/launch/vslam.launch.py):
  - use_sim_time: true (clock bridged from Gazebo)
  - base frame is champ's 'base_link' instead of Spot driver's 'body'
  - no RealSense driver (Gazebo publishes the same topics)
  - denoising off (sim images are clean)
  - IMU fusion OFF by default -- see below

IMU fusion note: this used to be on, to exercise the same code path as the
Jetson. In practice it made cuVSLAM diverge catastrophically here (map->odom
reaching hundreds of metres, in one case (-348, -419, +449)). The cause is
timing, not the IMU data itself: Gazebo's renderer falls back to software
rasterisation on this stack, the sim runs at a real-time factor around
0.15, and camera frames arrive 66-99 ms apart against cuVSLAM's 34 ms
threshold. IMU preintegration double-integrates acceleration between frames,
so irregular gaps of that size accumulate error explosively, while
stereo-only VO matches each frame independently and rides it out.

Turn it back on with `enable_imu_fusion:=true` when running somewhere the
sim holds near real-time -- the failure is a symptom of a starved simulator,
not of the fusion code.

Runs standalone inside the vslam container (this file is copied to /launch).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    enable_imu_fusion = LaunchConfiguration("enable_imu_fusion")
    image_jitter_threshold_ms = LaunchConfiguration("image_jitter_threshold_ms")

    args = [
        DeclareLaunchArgument(
            "enable_imu_fusion",
            default_value="false",
            description="Fuse the D435i IMU into cuVSLAM. Off by default in "
            "sim -- diverges when the simulator runs well below real time.",
        ),
        DeclareLaunchArgument(
            "image_jitter_threshold_ms",
            default_value="150.0",
            description="How late a frame may be before cuVSLAM complains. "
            "The 34 ms default assumes a steady 30 Hz; this sim delivers "
            "66-99 ms gaps under load, which is expected here rather than a "
            "fault worth warning about on every frame.",
        ),
    ]

    visual_slam_node = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam",
        parameters=[
            {
                "use_sim_time": True,
                "num_cameras": 2,
                "rectified_images": True,
                "enable_imu_fusion": ParameterValue(
                    enable_imu_fusion, value_type=bool
                ),
                "enable_ground_constraint_in_odometry": True,
                "enable_ground_constraint_in_slam": True,
                "gyro_noise_density": 0.000244,
                "gyro_random_walk": 0.000019393,
                "accel_noise_density": 0.001862,
                "accel_random_walk": 0.003,
                "calibration_frequency": 200.0,
                "enable_localization_n_mapping": True,
                "enable_slam_visualization": True,
                "enable_observations_view": True,
                "enable_landmarks_view": True,
                "enable_image_denoising": False,
                "image_jitter_threshold_ms": ParameterValue(
                    image_jitter_threshold_ms, value_type=float
                ),
                # Same TF contract as on the real robot:
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_map_to_odom_tf": True,
                "publish_odom_to_base_tf": False,
                "camera_optical_frames": [
                    "camera_infra1_optical_frame",
                    "camera_infra2_optical_frame",
                ],
                "imu_frame": "camera_imu_optical_frame",
            }
        ],
        remappings=[
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
    )


    return LaunchDescription(args + [container])
