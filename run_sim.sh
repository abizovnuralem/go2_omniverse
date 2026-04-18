#!/usr/bin/env bash
set -euo pipefail

# Isaac Sim 5.0 / IsaacLab 0.54.3 launcher for Unitree Go2 digital twin.
#
# We intentionally do NOT source /opt/ros/jazzy because system Jazzy is built
# for Python 3.12 and Isaac Sim 5.0 requires Python 3.11. The Isaac Sim
# ROS 2 bridge extension ships an internal Jazzy (rclpy + msg libs) matching
# Python 3.11 — this script points the loader at those libraries.

export ISAAC_VENV="${ISAAC_VENV:-$HOME/isaac-sim-venv}"
export ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/IsaacLab}"
export OMNI_KIT_ACCEPT_EULA=YES

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Activate Isaac Sim venv (Python 3.11, isaacsim 5.0, isaaclab 0.54.3)
source "$ISAAC_VENV/bin/activate"

ISAAC_ROS2_EXT="$(python -c "import isaacsim, os; print(os.path.join(os.path.dirname(isaacsim.__file__), 'exts', 'isaacsim.ros2.bridge'))")"
BUNDLED_LIB="$ISAAC_ROS2_EXT/$ROS_DISTRO/lib"
BUNDLED_RCLPY="$ISAAC_ROS2_EXT/$ROS_DISTRO/rclpy"

if [[ ! -d "$BUNDLED_LIB" ]]; then
    echo "[run_sim] bundled ROS 2 $ROS_DISTRO libs not found at $BUNDLED_LIB" >&2
    exit 1
fi

# The bridge extension does NOT auto-add its rclpy to Python's sys.path, so
# PYTHONPATH is required for rclpy imports. LD_LIBRARY_PATH is required so
# rclpy's native typesupport libs can be dlopen'd at publisher-creation time.
# When the bridge extension is also enabled, two copies of rcl_interfaces get
# loaded and trigger a ParameterEvent assert — so omniverse_sim.py now enables
# only the OmniGraph core extensions, not isaacsim.ros2.bridge, and publishes
# via rclpy from Python directly.
export PYTHONPATH="$BUNDLED_RCLPY${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$BUNDLED_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$SCRIPT_DIR"
exec python -u main.py --robot_amount 1 --robot go2 --terrain flat "$@"
