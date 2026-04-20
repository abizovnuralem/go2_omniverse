# Go2 Isaac Sim digital-twin — IMU-driven fix (2026-04-20)

Working snapshot of the Unitree Go2 → Isaac Sim twinbot after fixing two bugs:
joint-scale mismatch and missing base-orientation forwarding. The sim dog now
mirrors the real Go2 1:1 (joints + base orientation) via kinematic playback.

Everything in `modified_files/` was edited in-place at
`~/go2_ws/src/go2_omniverse/` on careslab. To restore on another machine:
copy each file back to the same path.

---

## System layout

```
┌─────────────────────────┐   eth (CycloneDDS)         ┌───────────────────┐
│  Unitree Go2            │  /lowstate @ 500 Hz        │  Jetson payload   │
│  192.168.123.161        │──────────────────────────▶ │  192.168.123.18   │
│  (IMU + motor_state)    │                            │  enP8p1s0         │
└─────────────────────────┘                            │                   │
                                                        │  twinbot_bridge.py│
                                                        │  (dual-RMW)       │
                                                        └──────┬────────────┘
                                                               │ WiFi (FastDDS)
                                                               │ /real_dog/joint_states
                                                               │ /real_dog/odom
                                                               ▼
                                                    ┌──────────────────────┐
                                                    │  careslab (Isaac Sim)│
                                                    │  192.168.0.4         │
                                                    │                      │
                                                    │  omniverse_sim.py    │
                                                    │    + twinbot.py      │
                                                    │  kinematic playback  │
                                                    └──────────────────────┘
```

Real dog is only reachable from the Jetson (ethernet subnet `192.168.123.x`).
Jetson bridges to careslab via WiFi + FastDDS because Isaac Sim's bundled
ROS 2 Jazzy uses FastDDS and the Go2 SDK uses CycloneDDS — the two RMWs
cannot discover each other without a bridge.

---

## Bug 1: joint target blended half-real/half-default

**Symptom.** "Dog's regular stand isn't correct. Back hips aren't accurate."

**Root cause.** In `omniverse_sim.py` (pre-fix), the twin drove the sim via
the RL action pipeline:

```python
twin_actions = twin.actions(device)      # action = (real - default) / 0.5
obs, _, _, _ = env.step(twin_actions)    # env target = default + action * 0.25
```

- `twinbot.py` divided by `action_scale=0.5`
- `custom_rl_env.py:308` overrides the env's `joint_pos.scale = 0.25`

Net effect:
```
target = default + [(real - default) / 0.5] * 0.25
       = default + (real - default) * 0.5
       = (default + real) / 2
```

**The sim target was the midpoint between Isaac Lab's RL nominal pose and
the real dog.** Isaac Lab's nominal (`unitree.py:157-165`) has rear thighs
at 1.0 rad vs front 0.8, and hips at ±0.1 — so the rear/hips looked
noticeably off.

**First fix attempt** (later superseded): align `_action_scale` to `0.25`
inside `twinbot.py`.

## Bug 2: no base-pose forwarding → sim dog flipped upside down

**Symptom.** After fix 1, joints tracked but the whole sim dog toppled.

**Root cause.** `twinbot_bridge.py` only forwarded joint state. The sim
robot was spawned at `(0, 0, 0.4)` with free base physics and no RL policy
correction (because twin replaces policy). Any small leg-force imbalance
tipped it — gravity did the rest.

**Fix (what's captured in this folder):**

1. **Bridge extracts IMU from `/lowstate`** (quaternion + gyroscope — which
   are *already* inside every LowState message — no new subscription) and
   publishes `nav_msgs/Odometry` on `/real_dog/odom`.

2. **Sim consumer (`twinbot.py`) switches to pure kinematic playback:**
   every frame, `write_joint_state_to_sim()` overrides joints and
   `write_root_pose_to_sim()` + `write_root_velocity_to_sim()` override
   the base with the real dog's orientation (XY pinned at spawn, Z
   pinned at spawn, orientation from IMU). Physics still runs but gets
   overwritten after `env.step()` — the sim dog becomes a pure visual
   mirror, immune to contact/gravity instability.

3. **Main loop** (`omniverse_sim.py`) calls `twin.apply(device)` *after*
   `env.step()` so the RL action pipeline can't undo the override:
   ```python
   actions = policy(obs)
   obs, _, _, _ = env.step(actions)
   if twin is not None:
       twin.apply(device)   # kinematic playback — overrides physics
   ```

---

## Files in this snapshot

### `modified_files/` — the three files changed to make this work

| File | What changed |
|------|--------------|
| `scripts/twinbot_bridge.py` | `_cb` now also extracts `msg.imu_state.quaternion` and `msg.imu_state.gyroscope` and pushes them on the mp.Queue. `_drain` publishes a companion `Odometry` message on `/real_dog/odom` with `pose.orientation` = SDK quaternion (converted wxyz→xyzw by ROS layout) and `twist.angular` = gyro. |
| `twinbot.py` | Rewritten to subscribe to both `/real_dog/joint_states` **and** `/real_dog/odom`. Cached `_latest_pos / _latest_vel / _latest_quat / _latest_ang_vel`. New `apply()` uses `Articulation.write_joint_state_to_sim()` + `write_root_pose_to_sim()` + `write_root_velocity_to_sim()` for kinematic playback. Spawn XY/Z is captured at init and re-used every frame so the sim dog stays visible at origin even though IMU has no absolute position. |
| `omniverse_sim.py` | Main loop now calls `policy(obs)` + `env.step(actions)` + `twin.apply(device)` — in that order. The old `twin.actions()` path (RL-action-based) was removed from the loop. |
| `run_twinbot.sh` | Unchanged — included for completeness; launches Isaac Sim with `--twinbot`. |

### `reference/`

| File | Why it's here |
|------|---------------|
| `custom_rl_env.py` | Shows where `self.actions.joint_pos.scale = 0.25` overrides the 0.5 default. This was the root of Bug 1 — document it here so the next person doesn't retune blindly. |
| `isaaclab_unitree_assets.py` | Isaac Lab's stock `UNITREE_GO2_CFG`. Lines 157-165 show the nominal stand pose (`.*L_hip_joint: 0.1`, `.*R_hip_joint: -0.1`, front thigh 0.8, rear thigh 1.0, calf -1.5). Reading this is what made Bug 1 diagnosable. |
| `last_bridge_run.log` | ~500 Hz `/lowstate` forwarding log from the working session. |
| `last_sim_run.log` | Isaac Sim startup log from the working session. Shows `TwinbotSubscriber ready — listening /real_dog/joint_states + /real_dog/odom`. |

---

## Reproducing the setup

### 1. Jetson — deploy & launch the bridge

```bash
# from careslab, copy the bridge to the Jetson (keyless SSH assumed)
scp modified_files/twinbot_bridge.py unitree@192.168.0.2:~/twinbot_bridge.py

# launch on the Jetson, detached
ssh unitree@192.168.0.2 "bash -lc '
  source /opt/ros/humble/setup.bash &&
  source ~/unitree_ros2/cyclonedds_ws/install/setup.bash &&
  setsid nohup python3 ~/twinbot_bridge.py > ~/twinbot_bridge.log 2>&1 < /dev/null &
  disown'"
```

Verify: `ssh unitree@192.168.0.2 'tail -5 ~/twinbot_bridge.log'` should show
`forwarded N msgs — FL_hip=... rad` lines at ~500 Hz.

### 2. careslab — launch Isaac Sim with `--twinbot`

```bash
cd ~/go2_ws/src/go2_omniverse
bash run_twinbot.sh
```

Look for these log lines:
- `TwinbotSubscriber ready — listening /real_dog/joint_states + /real_dog/odom`
- `entering main loop`

The "Error loading custom environment" and Camera `_rep_registry` messages
are **pre-existing** go2_omniverse noise, not related to this fix.

### 3. Sanity checks if it's not twinning

```bash
# confirm FastDDS topics are visible from careslab's WiFi:
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic list | grep real_dog           # should show both topics
ros2 topic echo /real_dog/odom --once     # should print a quaternion
```

If the bridge is up but no topics visible on careslab — the Jetson's WiFi
interface may have changed. Bridge defaults to FastDDS's automatic WiFi
selection; no config needed normally.

---

## Diagnostic artifacts from the session (for future reference)

Live real-dog joint readings while standing on 2026-04-20 (SDK motor order):

```
FR_hip   -0.021   FR_thigh +0.846   FR_calf -1.466
FL_hip   -0.003   FL_thigh +0.810   FL_calf -1.580
RR_hip   +0.156   RR_thigh +0.805   RR_calf -1.490
RL_hip   -0.101   RL_thigh +0.566   RL_calf -1.487
```

IMU on the same sample:
```
quat (w,x,y,z) = 0.714, -0.020, -0.002, 0.700   (yaw ≈ 1.55 rad / 89°)
rpy            = -0.032, 0.025, 1.550
gyro           = -0.005, -0.015, -0.001
```

Isaac Lab's nominal stand pose (reference, not the real-dog pose):
```
.*L_hip_joint:  +0.1      .*R_hip_joint:  -0.1
F[L,R]_thigh:   +0.8      R[L,R]_thigh:   +1.0
.*_calf:        -1.5
```

Note how Isaac Lab expects rear thighs at 1.0 but the real dog stands at
0.8 — this is why the pre-fix half-blend showed a visibly crouched rear.

---

## Why we *didn't* swap to a new repo

We briefly explored `unitreerobotics/unitree_sim_isaaclab` — their official
"same DDS as real robot" Isaac Lab project — but it only supports G1 and
H1-2 humanoids, no Go2. `CLeARoboticsLab/go2_isaac_ros2` was the other
candidate (Go2-specific, also consumes `/lowstate`). We stuck with
extending our existing `go2_omniverse` bridge because the fix was
localized and kept our infrastructure intact.

---

## What's still open

- **No absolute base position.** Sim dog rotates with the real dog but
  stays at the Isaac Sim spawn XY. If you want the sim dog to walk around
  matching the real one, forward `/robot0/odom` or `/utlidar/robot_odom`
  from the Jetson as well and plumb the position into `twinbot.py`. At
  time of writing, `/robot0/odom` was advertised but silent on the real
  dog (SLAM stack wasn't running).
- **Camera `_rep_registry` error** in Isaac Sim startup. Pre-existing
  go2_omniverse issue; does not affect twinning.
