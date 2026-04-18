#!/usr/bin/env bash
set -euo pipefail

# Digital-twin launcher for Unitree Go2.
#
# Runs Isaac Sim in twinbot mode: sim robot joints are driven by the real
# Go2's /lowstate stream, forwarded via twinbot_bridge.py on the Jetson.
#
# Setup (one-time on Jetson):
#   source /opt/ros/humble/setup.bash
#   source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
#   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   python3 scripts/twinbot_bridge.py
#
# Then on this machine:
#   bash run_twinbot.sh [--headless]

export ISAAC_VENV="${ISAAC_VENV:-$HOME/isaac-sim-venv}"
export ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/IsaacLab}"
export OMNI_KIT_ACCEPT_EULA=YES

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$ISAAC_VENV/bin/activate"

ISAAC_ROS2_EXT="$(python -c "import isaacsim, os; print(os.path.join(os.path.dirname(isaacsim.__file__), 'exts', 'isaacsim.ros2.bridge'))")"
BUNDLED_LIB="$ISAAC_ROS2_EXT/$ROS_DISTRO/lib"
BUNDLED_RCLPY="$ISAAC_ROS2_EXT/$ROS_DISTRO/rclpy"

if [[ ! -d "$BUNDLED_LIB" ]]; then
    echo "[run_twinbot] bundled ROS 2 $ROS_DISTRO libs not found at $BUNDLED_LIB" >&2
    exit 1
fi

export PYTHONPATH="$BUNDLED_RCLPY${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$BUNDLED_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

echo "[run_twinbot] starting digital-twin sim (robot=go2, terrain=flat, --twinbot)"
cd "$SCRIPT_DIR"
exec python -u main.py --robot_amount 1 --robot go2 --terrain flat --twinbot "$@"
