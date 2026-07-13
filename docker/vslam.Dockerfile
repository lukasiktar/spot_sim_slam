# =============================================================================
# Isaac ROS Visual SLAM image — x86_64 laptop with NVIDIA GPU
#
# Base: CUDA runtime on Ubuntu 22.04, ROS 2 Humble, NVIDIA Isaac apt repo.
#
# Run with the NVIDIA container runtime (handled by docker-compose 'deploy'
# section, or manually with:  docker run --gpus all ...)
#
# NOTE: If this packaged install fights your driver/CUDA combo, the bulletproof
# alternative is NVIDIA's own container workflow:
#   git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
#   ./isaac_ros_common/scripts/run_dev.sh
# and `apt install ros-humble-isaac-ros-visual-slam` inside it, then use the
# same launch file mounted from ./sim/launch/vslam_sim.launch.py
# =============================================================================
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

# ---- NVIDIA VPI 3 (libnvvpi.so.3) -------------------------------------------
# ros-humble-isaac-ros-visual-slam dlopens VPI at runtime rather than
# declaring it as an apt dependency, so it has to be installed explicitly or
# the node fails to load with "libnvvpi.so.3: cannot open shared object file".
RUN apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc && \
    add-apt-repository -y 'deb https://repo.download.nvidia.com/jetson/x86_64/jammy r36.2 main' && \
    apt-get update && apt-get install -y --no-install-recommends \
    libnvvpi3 \
    && rm -rf /var/lib/apt/lists/*

# ---- Our sim-specific VSLAM launch + params --------------------------------
COPY sim/launch/vslam_sim.launch.py /launch/vslam_sim.launch.py
COPY spot_vslam_nav/config/vslam.yaml /config/vslam.yaml

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

CMD ["bash"]
