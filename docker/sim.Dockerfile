# Simulation image: ROS 2 Humble + Gazebo Sim + Spot (champ) + simulated D435i
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
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
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# ---- Workspace: champ-based Spot simulation --------------------------------
WORKDIR /ws/src
RUN git clone --depth 1 https://github.com/g1y5x3/spot_gazebo_ros2.git spot_gazebo_ros2 || \
    git clone --depth 1 https://github.com/justyx404/spot_gazebo_ros2.git spot_gazebo_ros2

RUN sed -i 's#<odom_frame>/odom_spot</odom_frame>#<odom_frame>odom</odom_frame>#' \
      /ws/src/spot_gazebo_ros2/spot_description/models/spot/model.sdf


RUN sed -i 's#package://spot_description/meshes/#package://spot_description/models/spot/meshes/#g' \
      /ws/src/spot_gazebo_ros2/spot_description/models/spot/model.urdf


COPY spot_vslam_nav /ws/src/spot_vslam_nav
COPY sim /ws/src/spot_sim_slam
COPY spot_search /ws/src/spot_search

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
      "isaac_ros_visual_slam spot_driver spot_msgs realsense2_camera \
       nvblox_msgs vision_msgs" && \
    colcon build --symlink-install --packages-skip-regex ".*test.*" && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
