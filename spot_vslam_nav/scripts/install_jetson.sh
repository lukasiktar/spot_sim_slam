#!/usr/bin/env bash
# =============================================================================
# Full-stack install on Jetson Orin Nano (JetPack 6.x, Ubuntu 22.04)
#   1. ROS 2 Humble
#   2. NVIDIA Isaac ROS apt repo + isaac_ros_visual_slam
#   3. Nav2 + support packages
#   4. RealSense (librealsense + realsense-ros)
#   5. spot_ros2 (built from source, ARM64)
#   6. This package (spot_vslam_nav)
#
# Run on the Jetson:  bash install_jetson.sh
# =============================================================================
set -euo pipefail

WS=${WS:-$HOME/spot_ws}
ROS_DISTRO=humble

echo "==> [0/6] Prerequisites"
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release git python3-pip software-properties-common

# Jetson performance: MAXN power mode + jetson_clocks strongly recommended
sudo nvpmodel -m 0 || true
sudo jetson_clocks || true

echo "==> [1/6] ROS 2 Humble"
if ! dpkg -l | grep -q "ros-$ROS_DISTRO-ros-base"; then
  sudo add-apt-repository -y universe
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  sudo apt-get update
  sudo apt-get install -y ros-$ROS_DISTRO-ros-base ros-dev-tools python3-colcon-common-extensions
fi

echo "==> [2/6] Isaac ROS apt repo + Visual SLAM"
# NVIDIA's Isaac apt repository (JetPack 6.x). If the packaged install fails
# on your JetPack version, use NVIDIA's isaac_ros_common dev container or
# build isaac_ros_visual_slam from source instead — see README.
if [ ! -f /etc/apt/sources.list.d/isaac-ros.list ]; then
  wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add - || true
  grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" \
      /etc/apt/sources.list.d/isaac-ros.list 2>/dev/null || \
  echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" \
      | sudo tee /etc/apt/sources.list.d/isaac-ros.list
  sudo apt-get update
fi
sudo apt-get install -y ros-$ROS_DISTRO-isaac-ros-visual-slam || {
  echo "!! Packaged Isaac ROS install failed — fall back to source build (see README §Isaac ROS)."
}

echo "==> [3/6] Nav2 + tools"
sudo apt-get install -y \
  ros-$ROS_DISTRO-navigation2 \
  ros-$ROS_DISTRO-nav2-bringup \
  ros-$ROS_DISTRO-nav2-mppi-controller \
  ros-$ROS_DISTRO-pointcloud-to-laserscan \
  ros-$ROS_DISTRO-topic-tools \
  ros-$ROS_DISTRO-tf2-tools

echo "==> [4/6] RealSense"
sudo apt-get install -y ros-$ROS_DISTRO-realsense2-camera || {
  echo "!! realsense2_camera apt package unavailable — build librealsense with CUDA from source for best Jetson performance."
}

echo "==> [5/6] spot_ros2 (source build)"
mkdir -p "$WS/src"
cd "$WS/src"
if [ ! -d spot_ros2 ]; then
  git clone https://github.com/rai-opensource/spot_ros2.git
  cd spot_ros2
  git submodule init && git submodule update
  ./install_spot_ros2.sh --arm64
  cd ..
fi

echo "==> [6/6] spot_vslam_nav"
# Assumes this script lives in spot_vslam_nav/scripts inside your workspace;
# otherwise copy the spot_vslam_nav package into $WS/src first.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
if [ ! -d "$WS/src/spot_vslam_nav" ]; then
  cp -r "$PKG_DIR" "$WS/src/spot_vslam_nav"
fi

cd "$WS"
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y || true
colcon build --symlink-install --packages-skip-regex ".*test.*"

echo ""
echo "=================================================================="
echo " Done. Add to ~/.bashrc:"
echo "   source /opt/ros/$ROS_DISTRO/setup.bash"
echo "   source $WS/install/setup.bash"
echo ""
echo " Then edit credentials in:"
echo "   $WS/src/spot_vslam_nav/config/spot_ros2_config.yaml"
echo " and launch:"
echo "   ros2 launch spot_vslam_nav spot_slam_nav.launch.py"
echo "=================================================================="
