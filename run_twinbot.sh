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

export ISAAC_VENV="${ISAAC_VENV:-$HOME/Sim/isaac-sim-venv}"
export ISAACLAB_PATH="${ISAACLAB_PATH:-$HOME/Sim/IsaacLab}"
export OMNI_KIT_ACCEPT_EULA=YES

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source "$ISAAC_VENV/bin/activate"

# ponytail: Isaac 5.x moved the bundled ROS 2 libs from isaacsim.ros2.bridge -> isaacsim.ros2.core.
# Pick whichever ext actually has $ROS_DISTRO/lib so this works across Isaac versions.
ISAAC_ROS2_EXT="$(python -c "import isaacsim, os; b=os.path.dirname(isaacsim.__file__); d=os.environ['ROS_DISTRO']; print(next(os.path.join(b,'exts',e) for e in ('isaacsim.ros2.core','isaacsim.ros2.bridge') if os.path.isdir(os.path.join(b,'exts',e,d,'lib'))))")"
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
# ponytail: --rendering_mode quality bumps the default 'balanced' RT2 preset up to the
# 'quality' preset (better GI/shadows/reflections). One robot + flat terrain is light
# enough to run it live on the 5070; pass --rendering_mode balanced after to override.
exec python -u main.py --robot_amount 1 --robot go2 --terrain flat --twinbot --rendering_mode quality "$@"
