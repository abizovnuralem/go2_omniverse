#!/usr/bin/env bash
set -euo pipefail

# Isaac Sim 5.0 / IsaacLab 0.54.3 launcher for Unitree G1 humanoid.

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
    echo "[run_sim_g1] bundled ROS 2 $ROS_DISTRO libs not found at $BUNDLED_LIB" >&2
    exit 1
fi

export LD_LIBRARY_PATH="$BUNDLED_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$BUNDLED_RCLPY${PYTHONPATH:+:$PYTHONPATH}"

cd "$SCRIPT_DIR"
exec python -u main.py --robot_amount 1 --robot g1 "$@"
