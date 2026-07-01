# Running go2_omniverse on Ubuntu 24.04 / ROS 2 Jazzy / Isaac Sim 5.0

This document describes an experimental port of `go2_omniverse` to the modern
NVIDIA Isaac stack (Isaac Sim 5.0 + IsaacLab 0.54.3) on an Ubuntu 24.04 host
whose system ROS 2 distro is Jazzy. The original upstream targets Ubuntu
22.04 + Isaac Sim 2023.1.1 + Orbit 0.3.0 + ROS 2 Humble; that path still
works and is not changed by this port.

## Target stack

| Component               | Version                            |
|-------------------------|------------------------------------|
| OS                      | Ubuntu 24.04.4 LTS                 |
| GPU driver              | NVIDIA 570.x (tested on 570.211)   |
| GPU                     | RTX 5080 (any RTX 30/40/50 works)  |
| Isaac Sim (pip)         | 5.0.0.0                            |
| IsaacLab (pip)          | 0.54.3                             |
| Python                  | 3.11 (Isaac Sim's venv)            |
| Host ROS 2              | Jazzy (used only for tooling)      |
| In-sim ROS 2 runtime    | Jazzy — **bundled inside** Isaac Sim's `isaacsim.ros2.bridge` extension |
| RMW                     | `rmw_fastrtps_cpp`                 |

### Why we do *not* source `/opt/ros/jazzy` in the launcher

System Jazzy on Ubuntu 24.04 is built against **Python 3.12**. Isaac Sim 5.0
ships a pinned **Python 3.11** interpreter. Mixing the two produces an
`rclpy` `.so` / ABI mismatch at import time.

Isaac Sim's `isaacsim.ros2.bridge` extension ships its own **internal** ROS 2
Jazzy runtime (`rclpy`, `rmw_fastrtps_cpp`, and all common message
packages — `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`,
`tf2_msgs`, etc.), built for Python 3.11 and installed under
`…/isaacsim/exts/isaacsim.ros2.bridge/jazzy/`. The launcher scripts point
`LD_LIBRARY_PATH` and `PYTHONPATH` at that bundle, which is the path NVIDIA
officially supports for headless Isaac Sim ROS 2 use.

## What the bundled Jazzy runtime does *not* ship

- `tf2_ros` (Python + C++ transform broadcaster helpers)
- `sensor_msgs_py` (pointcloud convenience builder)
- `go2_interfaces` (the Humble-only custom `Go2State.msg` from the upstream
  workspace)

The port works around these by:

- Publishing TF directly as `tf2_msgs/TFMessage` on `/tf`.
- Building `sensor_msgs/PointCloud2` with plain `numpy.tobytes()` packing.
- Publishing foot-force as `std_msgs/Float32MultiArray` on
  `robot<N>/foot_force` instead of `go2_interfaces/Go2State`.

Consumers that expected `go2_interfaces/Go2State` need one of:

1. switch to `std_msgs/Float32MultiArray` on the new topic, or
2. run a small bridge node on the Humble side that converts
   `Float32MultiArray` → `Go2State`.

## First-time setup

```bash
# 1. Create the Isaac Sim venv (Python 3.11)
python3.11 -m venv ~/Sim/isaac-sim-venv
source ~/Sim/isaac-sim-venv/bin/activate
pip install --upgrade pip

# 2. Install Isaac Sim 5.0 + IsaacLab 0.54.3 into the venv
#    Follow NVIDIA's official pip install steps:
#    https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html
pip install isaacsim[all,extscache]==5.0.0.0 --extra-index-url https://pypi.nvidia.com
pip install isaaclab==0.54.3

# 3. Accept the EULA once (the launcher sets this for you too)
export OMNI_KIT_ACCEPT_EULA=YES

# 4. Clone this repo
git clone https://github.com/abizovnuralem/go2_omniverse.git
cd go2_omniverse

# 5. Run
./run_sim.sh --headless          # or ./run_sim_g1.sh
```

The launcher auto-points `LD_LIBRARY_PATH` / `PYTHONPATH` at the Isaac Sim
bundled Jazzy runtime — you do **not** need to `source /opt/ros/jazzy`.

Override knobs:

- `ISAAC_VENV` — path to the venv (default `~/Sim/isaac-sim-venv`)
- `ISAACLAB_PATH` — path to a source IsaacLab checkout if you use one
- `RMW_IMPLEMENTATION` — defaults to `rmw_fastrtps_cpp`

## Topics published

Per robot `N`:

| Topic                          | Type                          |
|--------------------------------|-------------------------------|
| `/robot<N>/joint_states`       | `sensor_msgs/JointState`      |
| `/robot<N>/odom`               | `nav_msgs/Odometry`           |
| `/robot<N>/imu`                | `sensor_msgs/Imu`             |
| `/robot<N>/foot_force`         | `std_msgs/Float32MultiArray`  |
| `/robot<N>/point_cloud2`       | `sensor_msgs/PointCloud2`     |
| `/tf`                          | `tf2_msgs/TFMessage`          |

## Interop with a Humble-based physical Go2

Humble (dog) ↔ Jazzy (sim) on the wire works when **both sides run Fast DDS**
(`rmw_fastrtps_cpp`), which is Humble's default and what this launcher sets
on the sim side. The two distros use compatible IDL / DDS wire formats for
all standard messages used here.

Guidance for mixed-distro setups:

- Keep `ROS_DOMAIN_ID` aligned across both hosts (default `0`).
- Use the same RMW on both sides. Fast DDS is the safer pick.
- Avoid `Go2State` on the shared wire until you have a bridge on the
  Humble side, since the sim no longer publishes it.
- Run discovery on the same L2 segment or use the Fast DDS discovery
  server — standard ROS 2 mixed-distro guidance applies.

Not validated here: a full end-to-end dog ↔ sim link, since the lab dog is
not on this bench. Treat the interop section as documented expectation, not
verified behavior.

## What is verified on the lab PC as of 2026-04-18

- Ubuntu 24.04.4, ROS 2 Jazzy installed (system), Python 3.12 system / 3.11 in venv.
- Isaac Sim 5.0.0.0 + IsaacLab 0.54.3 import cleanly in the venv.
- Bundled Jazzy runtime is present and complete for the message types
  used: `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`,
  `tf2_msgs`, Fast DDS typesupport.
- All modified `.py` files parse under Python 3.11.
- No remaining `omni.isaac.orbit` / `omni.isaac.ros2_bridge` /
  `omni.isaac.core_nodes` / `omni.isaac.orbit_tasks` imports.
- `tf2_ros`, `sensor_msgs_py`, and `go2_interfaces` are no longer imported.

## What is *not* verified

- A full runtime boot producing the expected ROS topics in a reproducible
  timeframe on this specific machine (first-time Isaac Sim 5.0 shader-cache
  compilation dominates the boot, see README note below).
- Mixed-distro (Jazzy sim host ↔ Humble dog) live topic exchange.
- Custom envs (`--custom_env`), VR support, Nav2 stack on Jazzy —
  untouched by this port and not re-validated.

## Known limitations

- `go2_interfaces/Go2State` is no longer published. Downstream tools that
  require it need the bridge described above.
- The legacy RSL-RL `OnPolicyRunner` checkpoint API was removed in the
  current `rsl_rl-lib`. `omniverse_sim.py` now rebuilds a matching MLP
  actor directly from the checkpoint's `actor.*` weights for inference.
  Re-training still needs the runner from a matching RSL-RL version and is
  out of scope for this port.
- The original `conda activate orbit` + `./orbit.sh --install` path is
  gone; the new path is pip-into-venv per NVIDIA's current guidance.
