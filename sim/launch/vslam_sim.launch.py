"""
Isaac ROS Visual SLAM for the SIMULATED Spot + D435i.

Differences vs. the on-robot launch (spot_vslam_nav/launch/vslam.launch.py):
  - use_sim_time: true (clock bridged from Gazebo)
  - base frame is champ's 'base_link' instead of Spot driver's 'body'
  - no RealSense driver (Gazebo publishes the same topics)
  - denoising off (sim images are clean), IMU fusion kept ON to exercise
    the same code path you'll run on the Jetson

Runs standalone inside the vslam container (this file is copied to /launch).
"""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    visual_slam_node = ComposableNode(
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        name="visual_slam",
        parameters=[
            {
                "use_sim_time": True,
                "num_cameras": 2,
                "rectified_images": True,
                "enable_imu_fusion": True,
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
                "img_jitter_threshold_ms": 34.0,
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

    return LaunchDescription([container])
