"""Script to play a checkpoint if an RL agent from RSL-RL."""
from __future__ import annotations


"""Launch Isaac Sim Simulator first."""
import argparse
from isaaclab.app import AppLauncher


import cli_args  
import time
import os
import threading


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--custom_env", type=str, default="office", help="Setup the environment")
parser.add_argument("--robot", type=str, default="go2", help="Setup the robot")
parser.add_argument("--terrain", type=str, default="rough", help="Setup the robot")
parser.add_argument("--robot_amount", type=int, default=1, help="Setup the robot amount")
parser.add_argument("--twinbot", action="store_true", default=False,
                    help="Digital-twin mode: drive sim joints from real Go2 via /real_dog/joint_states "
                         "(requires twinbot_bridge.py running on the Jetson)")


# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)


# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


def _ckpt(msg: str):
    print(f"[go2_omniverse] {time.strftime('%H:%M:%S')} {msg}", flush=True)


_ckpt("AppLauncher: constructing...")
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
_ckpt("AppLauncher: ready")


import omni
_ckpt("import omni: ok")


ext_manager = omni.kit.app.get_app().get_extension_manager()
# Required OmniGraph + ROS 2 bridge extensions. We use the non-immediate
# set_extension_enabled + simulation_app.update() pump: set_extension_enabled_immediate
# was observed to deadlock on isaacsim.core.nodes on Isaac Sim 5.0 / Ubuntu 24.04.
# isaacsim.ros2.bridge depends on isaacsim.core.nodes so enabling the bridge
# transitively enables the nodes extension.
# NOTE: we deliberately do NOT enable `isaacsim.ros2.bridge`. That extension
# loads its own in-process rcl_interfaces typesupport which conflicts with the
# rclpy we use from Python and triggers a ParameterEvent assertion. Since this
# project publishes ROS 2 data through rclpy from omniverse_sim.py directly
# (not via OmniGraph ROS2Helper nodes), we only need the OmniGraph core
# extensions for the action-graph based camera stream.
_required_exts = (
    "omni.graph.core",
    "omni.graph.action",
    "omni.graph.nodes",
    # Needed because ros2.py imports isaacsim.sensors.rtx.LidarRtx. This is
    # transitively enabled by isaacsim.ros2.bridge, but we don't enable the
    # bridge (see note above).
    "isaacsim.sensors.rtx",
)
for _ext in _required_exts:
    _ckpt(f"requesting extension: {_ext}")
    ext_manager.set_extension_enabled(_ext, True)

# Pump frames until all requested extensions are actually enabled (or give up).
_t0 = time.time()
while time.time() - _t0 < 120:
    simulation_app.update()
    pending = [e for e in _required_exts if not ext_manager.is_extension_enabled(e)]
    if not pending:
        break
    time.sleep(0.05)
still_pending = [e for e in _required_exts if not ext_manager.is_extension_enabled(e)]
_ckpt(f"extensions enabled in {time.time()-_t0:.2f}s (still_pending={still_pending})")

# FOR VR SUPPORT
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.core", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.system.steamvr", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.system.simulatedxr", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.system.openxr", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.telemetry", True)
# ext_manager.set_extension_enabled_immediate("omni.kit.xr.profile.vr", True)


"""Rest everything follows."""
_ckpt("import gymnasium")
import gymnasium as gym
_ckpt("import torch")
import torch
_ckpt("import carb")
import carb


_ckpt("import isaaclab_tasks.utils.parse_cfg")
from isaaclab_tasks.utils.parse_cfg import get_checkpoint_path
_ckpt("import isaaclab_rl.rsl_rl")
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
_ckpt("import isaaclab.sim")
import isaaclab.sim as sim_utils
_ckpt("import omni.appwindow")
import omni.appwindow


_ckpt("import rclpy")
import rclpy
_ckpt("import ros2 module")
from ros2 import RobotBaseNode, add_camera, add_rtx_lidar, pub_robo_data_ros2
from geometry_msgs.msg import Twist


_ckpt("import agent_cfg")
from agent_cfg import unitree_go2_agent_cfg, unitree_g1_agent_cfg
_ckpt("import custom_rl_env (this pulls isaaclab_assets Unitree USD cfg)")
from custom_rl_env import UnitreeGo2CustomEnvCfg, G1RoughEnvCfg
import custom_rl_env
_ckpt("import omnigraph")
from omnigraph import create_front_cam_omnigraph
_ckpt("all imports complete")

# twinbot import is deferred until after rclpy.init() in run_sim()


def _load_mlp_policy(ckpt_path: str, hidden_dims, activation_name: str, device: str):
    """Load an MLP actor from a legacy rsl_rl ActorCritic checkpoint.

    The installed rsl_rl-lib (5.x) has a different config/load API than the one
    used to train the shipped checkpoints (pre-2025). The checkpoint only
    contains MLP weights for actor / critic plus a learned std, so we rebuild a
    matching nn.Sequential for inference and skip the runner entirely.
    """
    import torch.nn as nn

    state = torch.load(ckpt_path, map_location=device, weights_only=False)
    sd = state["model_state_dict"]

    # Derive input dim from first layer
    actor_in = sd["actor.0.weight"].shape[1]
    actor_out = sd["actor.6.weight"].shape[0]

    activation = {"elu": nn.ELU, "relu": nn.ReLU, "tanh": nn.Tanh}[activation_name.lower()]

    layers = []
    dims = [actor_in, *hidden_dims]
    for i in range(len(hidden_dims)):
        layers.append(nn.Linear(dims[i], dims[i + 1]))
        layers.append(activation())
    layers.append(nn.Linear(hidden_dims[-1], actor_out))
    actor = nn.Sequential(*layers)

    actor_sd = {k[len("actor."):]: v for k, v in sd.items() if k.startswith("actor.")}
    actor.load_state_dict(actor_sd)
    actor.to(device).eval()
    return actor, actor_in, actor_out


def sub_keyboard_event(event, *args, **kwargs) -> bool:

    if len(custom_rl_env.base_command) > 0:
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            if event.input.name == 'W':
                custom_rl_env.base_command["0"] = [1, 0, 0]
            if event.input.name == 'S':
                custom_rl_env.base_command["0"] = [-1, 0, 0]
            if event.input.name == 'A':
                custom_rl_env.base_command["0"] = [0, 1, 0]
            if event.input.name == 'D':
                custom_rl_env.base_command["0"] = [0, -1, 0]
            if event.input.name == 'Q':
                custom_rl_env.base_command["0"] = [0, 0, 1]
            if event.input.name == 'E':
                custom_rl_env.base_command["0"] = [0, 0, -1]

            if len(custom_rl_env.base_command) > 1:
                if event.input.name == 'I':
                    custom_rl_env.base_command["1"] = [1, 0, 0]
                if event.input.name == 'K':
                    custom_rl_env.base_command["1"] = [-1, 0, 0]
                if event.input.name == 'J':
                    custom_rl_env.base_command["1"] = [0, 1, 0]
                if event.input.name == 'L':
                    custom_rl_env.base_command["1"] = [0, -1, 0]
                if event.input.name == 'U':
                    custom_rl_env.base_command["1"] = [0, 0, 1]
                if event.input.name == 'O':
                    custom_rl_env.base_command["1"] = [0, 0, -1]
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            for i in range(len(custom_rl_env.base_command)):
                custom_rl_env.base_command[str(i)] = [0, 0, 0]
    return True


def setup_custom_env():
    try:
        if (args_cli.custom_env == "warehouse" and args_cli.terrain == 'flat'):
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/warehouse.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(0.0, 0.0, 0.0))

        if (args_cli.custom_env == "office" and args_cli.terrain == 'flat'):
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/office.usd")
            cfg_scene.func("/World/office", cfg_scene, translation=(0.0, 0.0, 0.0))
    except:
        print("Error loading custom environment. You should download custom envs folder from: https://drive.google.com/drive/folders/1vVGuO1KIX1K6mD6mBHDZGm9nk2vaRyj3?usp=sharing")


def cmd_vel_cb(msg, num_robot):
    x = msg.linear.x
    y = msg.linear.y
    z = msg.angular.z
    custom_rl_env.base_command[str(num_robot)] = [x, y, z]



def add_cmd_sub(num_envs):
    node_test = rclpy.create_node('position_velocity_publisher')
    for i in range(num_envs):
        node_test.create_subscription(Twist, f'robot{i}/cmd_vel', lambda msg, i=i: cmd_vel_cb(msg, str(i)), 10)
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node_test,), daemon=True)
    thread.start()



def specify_cmd_for_robots(numv_envs):
    for i in range(numv_envs):
        custom_rl_env.base_command[str(i)] = [0, 0, 0]
def run_sim():
    
    # acquire input interface
    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, sub_keyboard_event)

    """Play with RSL-RL agent."""
    # parse configuration
    
    env_cfg = UnitreeGo2CustomEnvCfg()
    
    if args_cli.robot == "g1":
        env_cfg = G1RoughEnvCfg()

    # add N robots to env
    env_cfg.scene.num_envs = args_cli.robot_amount

    specify_cmd_for_robots(env_cfg.scene.num_envs)

    agent_cfg = unitree_go2_agent_cfg

    if args_cli.robot == "g1":
        agent_cfg = unitree_g1_agent_cfg

    # create isaac environment
    _ckpt(f"gym.make task={args_cli.task} num_envs={env_cfg.scene.num_envs}")
    env = gym.make(args_cli.task, cfg=env_cfg)
    _ckpt("gym.make: done")
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)
    _ckpt("RslRlVecEnvWrapper: wrapped")
    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg["experiment_name"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # Legacy checkpoint — build a matching MLP for inference (see helper above).
    device = str(env.unwrapped.device)
    actor, _, _ = _load_mlp_policy(
        resume_path,
        hidden_dims=agent_cfg["policy"]["actor_hidden_dims"],
        activation_name=agent_cfg["policy"]["activation"],
        device=device,
    )

    def policy(obs_dict):
        obs_tensor = obs_dict["policy"] if hasattr(obs_dict, "__getitem__") and "policy" in obs_dict else obs_dict
        return actor(obs_tensor)

    # reset environment
    _ckpt("env.get_observations()")
    obs = env.get_observations()
    _ckpt("env.get_observations: done")

    # initialize ROS2 node
    _ckpt("rclpy.init + RobotBaseNode")
    rclpy.init()
    base_node = RobotBaseNode(env_cfg.scene.num_envs)
    add_cmd_sub(env_cfg.scene.num_envs)
    _ckpt("ROS2 publishers up")

    twin = None
    if args_cli.twinbot:
        from twinbot import TwinbotSubscriber
        twin = TwinbotSubscriber(env)
        _ckpt("TwinbotSubscriber ready — waiting for /real_dog/joint_states")

    # Lidar disabled pending Unitree_L1.json update for Isaac Sim 5.0 schema.
    annotator_lst = []
    try:
        add_camera(env_cfg.scene.num_envs, args_cli.robot)
        _ckpt("camera added")
    except Exception as e:
        _ckpt(f"add_camera skipped ({type(e).__name__}: {e}) — isaaclab.sensors.Camera API changed in 0.54.x")

    # ROS 2 camera OmniGraph stream requires isaacsim.ros2.bridge, which we
    # deliberately do not enable (see extension-enable note above). Skip.
    _ckpt("camera omnigraph skipped (bridge extension disabled for rclpy compat)")
    _ckpt("entering main loop")

    setup_custom_env()

    start_time = time.time()
    # simulate environment
    while simulation_app.is_running():
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)
            if twin is not None:
                # Overwrite physics-stepped state with the real dog's state.
                # Kinematic playback — bypasses PD/gravity for an exact mirror.
                twin.apply(device)
            pub_robo_data_ros2(args_cli.robot, env_cfg.scene.num_envs, base_node, env, annotator_lst, start_time)
    env.close()