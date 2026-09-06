#!/usr/bin/env bash
set -euo pipefail

# Bootstrap script: installs OS-level ROS packages, pip deps, and guides how to
# install Interbotix packages into a workspace.
# Usage: sudo ROS_DISTRO=humble ./bootstrap.sh

if [[ -z "${ROS_DISTRO:-}" ]]; then
  echo "Please set ROS_DISTRO environment variable (e.g. export ROS_DISTRO=humble)"
  exit 1
fi

echo "Using ROS distro: $ROS_DISTRO"

# Basic apt deps
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions

# Install ROS python packages (rclpy, cv_bridge, tf2 tools) via apt
sudo apt install -y \
  ros-${ROS_DISTRO}-rclpy \
  ros-${ROS_DISTRO}-cv-bridge \
  ros-${ROS_DISTRO}-tf-transformations \
  ros-${ROS_DISTRO}-tf2-ros || true

# Install pip-only Python packages listed in requirements.txt
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt

# If you use a RealSense or other camera driver, install relevant packages
# (example: librealsense / realsense2 camera driver handled outside this script)

echo "\nTo install Interbotix packages into a ROS workspace, run the following (example):"
cat <<'EOS'
# create a workspace
mkdir -p interbotix_ws/src
cd interbotix_ws

# clone or copy Interbotix repos into src/ (example shown; prefer official sources)
# git clone https://github.com/Interbotix/Interbotix_xsarms_control.git src/...

# Install any OS deps via rosdep
rosdep update || true
rosdep install -i --from-paths src --rosdistro ${ROS_DISTRO} -y || true

# Build the workspace
colcon build --symlink-install

# Source the workspace (in your shell rc or before running scripts)
# source install/setup.bash
EOS

echo "Bootstrap complete. Review messages above for any errors. Remember to source your ROS environment and workspace before running the demo scripts."
