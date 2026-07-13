# =============================================================================
# Simulation image: ROS 2 Humble + Gazebo Sim + Spot (champ) + simulated D435i
# Matches the Jetson deployment distro (Humble) exactly.
# =============================================================================
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
# "compute,utility" (nvidia-container-toolkit's default) is enough for CUDA
# but not for Gazebo's own OpenGL rendering — without "graphics,display" too,
# it silently falls back to software rasterization even with a GPU attached.
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ---- System + ROS deps ------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    git python3-pip python3-colcon-common-extensions python3-rosdep \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-rtabmap-ros \
    ros-humble-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*

# ---- Gazebo Sim (new Gazebo) + ros_gz for Humble ---------------------------
# The champ Spot sim uses modern Gazebo Sim ("Ignition"). Humble's default
# pairing is Fortress; that is what the spot_gazebo_ros2 repo builds against
# and is the least-friction choice inside a Humble container.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# ---- Workspace: champ-based Spot simulation --------------------------------
WORKDIR /ws/src
RUN git clone --depth 1 https://github.com/g1y5x3/spot_gazebo_ros2.git spot_gazebo_ros2 || \
    git clone --depth 1 https://github.com/justyx404/spot_gazebo_ros2.git spot_gazebo_ros2

# spot_gazebo_ros2's OdometryPublisher plugin publishes a REP-105-violating
# odom frame name ("/odom_spot", with a stray leading slash) that won't match
# any consumer's "odom" frame. Normalize it at the source.
RUN sed -i 's#<odom_frame>/odom_spot</odom_frame>#<odom_frame>odom</odom_frame>#' \
      /ws/src/spot_gazebo_ros2/spot_description/models/spot/model.sdf

# ---- Copy in our packages ---------------------------------------------------
# spot_vslam_nav: the SAME package that deploys to the Jetson (nav2 configs,
#                 cmd_vel gate, health monitor, goal CLI).
# spot_sim_slam:  sim-only glue (D435i sensor overlay, bridge, launch files).
COPY spot_vslam_nav /ws/src/spot_vslam_nav
COPY sim /ws/src/spot_sim_slam

# ---- Inject the simulated D435i into Spot's URDF and model.sdf -------------
RUN python3 /ws/src/spot_sim_slam/scripts/inject_camera.py \
      --search-root /ws/src/spot_gazebo_ros2 \
      --overlay /ws/src/spot_sim_slam/urdf/d435i_sim.urdf.xacro \
      --sdf-overlay /ws/src/spot_sim_slam/urdf/d435i_sim_sensors.sdf.xml

# ---- Build ------------------------------------------------------------------
WORKDIR /ws
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys \
      "isaac_ros_visual_slam spot_driver spot_msgs realsense2_camera" && \
    colcon build --symlink-install --packages-skip-regex ".*test.*" && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
