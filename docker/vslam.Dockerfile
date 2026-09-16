# Isaac ROS image — x86_64 laptop with NVIDIA GPU
#
# Hosts the two GPU pieces of the stack:
#   vslam   — Isaac ROS Visual SLAM (cuVSLAM), publishes map->odom
#   nvblox  — Isaac ROS nvblox, 3D reconstruction + the 2D ESDF slice
# Base: CUDA runtime on Ubuntu 22.04, ROS 2 Humble, NVIDIA Isaac apt repo.

FROM nvidia/cuda:12.2.2-runtime-ubuntu22.04

SHELL ["/bin/bash", "-lc"]
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_DRIVER_CAPABILITIES=all

# ---- ROS 2 Humble -----------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl gnupg lsb-release software-properties-common && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    wget \
    && rm -rf /var/lib/apt/lists/*

# ---- NVIDIA Isaac ROS apt repo + Visual SLAM -------------------------------
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | apt-key add - && \
    echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 jammy release-3.0" \
      > /etc/apt/sources.list.d/isaac-ros.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-isaac-ros-visual-slam \
    && rm -rf /var/lib/apt/lists/*

# ---- NVIDIA Isaac ROS nvblox ------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nvblox-ros \
    ros-humble-nvblox-msgs \
    ros-humble-nvblox-nav2 \
    && rm -rf /var/lib/apt/lists/*

# ---- NVIDIA VPI 3 (libnvvpi.so.3) -------------------------------------------
RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
    add-apt-repository -y 'deb https://repo.download.nvidia.com/jetson/x86_64/jammy r36.2 main' && \
    apt-get update && apt-get install -y --no-install-recommends \
    libnvvpi3 \
    && rm -rf /var/lib/apt/lists/*

# ---- Our sim-specific VSLAM launch + params --------------------------------
COPY sim/launch/vslam_sim.launch.py /launch/vslam_sim.launch.py
COPY spot_vslam_nav/config/vslam.yaml /config/vslam.yaml

# ---- spot_search -------------------------------------------------------------
# Only map_slice_to_occupancy runs here, but it has to live in this image: it
# is the one node that needs nvblox_msgs, which exists nowhere else.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions python3-numpy \
    ros-humble-nav-msgs ros-humble-visualization-msgs ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

COPY spot_search /ws/src/spot_search
WORKDIR /ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
