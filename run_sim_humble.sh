#!/usr/bin/env bash
set -euo pipefail

# mewtwo-side launcher: Isaac Sim 6.0 + IsaacLab 4.5.22 + bundled Humble.
#
# Differences from run_sim.sh (which targets CaresLab: Isaac Sim 5.0 + bundled
# Jazzy, Python 3.11):
#   - Isaac Sim 6.0 moved the bundled-ROS payload out of
#     `isaacsim.ros2.bridge/{distro}/` and into
#     `isaacsim.ros2.core/{distro}/`. This launcher points LD_LIBRARY_PATH /
#     PYTHONPATH at the new location.
#   - ROS_DISTRO=humble. Isaac Sim 6.0 ships both humble and jazzy bundles;
#     mewtwo's on-robot side is Humble, so we use Humble to avoid a cross-
#     distro hop on the network.
#   - The venv's Python is 3.12 (not 3.11 like CaresLab). The bundled Humble
#     .so's are cpython-312, so this matches.
#
# System /opt/ros/humble is NOT sourced — same ABI-isolation reasoning as the
# Jazzy launcher: the bundled runtime is what Isaac Sim loads, and mixing an
# external rclpy in causes typesupport double-loads.

export ISAAC_VENV="${ISAAC_VENV:-$HOME/Sim/isaac-sim-venv}"
export ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/Sim/IsaacLab}"
export OMNI_KIT_ACCEPT_EULA=YES

export ROS_DISTRO=humble
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$ISAAC_VENV/bin/activate"

ISAAC_ROS2_CORE="$(python -c "import isaacsim, os; print(os.path.join(os.path.dirname(isaacsim.__file__), 'exts', 'isaacsim.ros2.core'))")"
BUNDLED_LIB="$ISAAC_ROS2_CORE/$ROS_DISTRO/lib"
BUNDLED_RCLPY="$ISAAC_ROS2_CORE/$ROS_DISTRO/rclpy"

if [[ ! -d "$BUNDLED_LIB" ]]; then
    echo "[run_sim_humble] bundled ROS 2 $ROS_DISTRO libs not found at $BUNDLED_LIB" >&2
    echo "[run_sim_humble] expected layout: \$ISAAC_ROS2_CORE/{humble,jazzy}/{lib,rclpy}" >&2
    exit 1
fi

export PYTHONPATH="$BUNDLED_RCLPY${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$BUNDLED_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$SCRIPT_DIR"
exec python -u main.py --robot_amount 1 --robot go2 --terrain flat "$@"
