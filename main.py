# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


"""Script to play a checkpoint if an RL agent from RSL-RL."""


from __future__ import annotations


"""Launch Isaac Sim Simulator first."""
import argparse
from omni.isaac.orbit.app import AppLauncher


import cli_args  
import time
import os


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument("--custom_env", type=str, default="office", help="Setup the environment")


# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)


# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()


# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import omni


ext_manager = omni.kit.app.get_app().get_extension_manager()
ext_manager.set_extension_enabled_immediate("omni.isaac.ros2_bridge", True)


"""Rest everything follows."""
import gymnasium as gym
import torch
import carb


from omni.isaac.orbit_tasks.utils import get_checkpoint_path
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper
)
import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.sensors import CameraCfg, Camera
from omni.isaac.sensor import LidarRtx
import omni.replicator.core as rep
import omni.appwindow
from rsl_rl.runners import OnPolicyRunner
from scipy.spatial.transform import Rotation


import rclpy
from ros2 import RobotBaseNode


from agent_cfg import unitree_go2_agent_cfg
from custom_rl_env import UnitreeGo2CustomEnvCfg


base_command = [0, 0, 0]


def sub_keyboard_event(event, *args, **kwargs) -> bool:
    global base_command
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input.name == 'W':
            base_command = [1, 0, 0]
        if event.input.name == 'S':
            base_command = [-1, 0, 0]
        if event.input.name == 'A':
            base_command = [0, 1, 0]
        if event.input.name == 'D':
            base_command = [0, -1, 0]
        if event.input.name == 'Q':
            base_command = [0, 0, 1]
        if event.input.name == 'E':
            base_command = [0, 0, -1]
    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        base_command = [0, 0, 0]
    return True


def update_meshes_for_cloud2(position_array, origin, rot):
    q = rot.cpu().numpy()
    rotation = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    rotated_vectors = rotation.apply(position_array)
    # Recalculate origin
    rotated_vectors += origin.cpu().numpy()
    return rotated_vectors


def setup_custom_env():
    try:
        if (args_cli.custom_env == "warehouse"):
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/warehouse.usd")
            cfg_scene.func("/World/warehouse", cfg_scene, translation=(0.0, 0.0, 0.0))

        if (args_cli.custom_env == "office"):
            cfg_scene = sim_utils.UsdFileCfg(usd_path="./envs/office.usd")
            cfg_scene.func("/World/office", cfg_scene, translation=(0.0, 0.0, 0.0))
    except:
        print("Error loading custom environment. You should download custom envs folder from: https://drive.google.com/drive/folders/1vVGuO1KIX1K6mD6mBHDZGm9nk2vaRyj3?usp=sharing")



def main():

    # acquire input interface
    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, sub_keyboard_event)

    """Play with RSL-RL agent."""
    # parse configuration
    
    env_cfg = UnitreeGo2CustomEnvCfg()
    env_cfg.scene.num_envs = 1
    
    agent_cfg: RslRlOnPolicyRunnerCfg = unitree_go2_agent_cfg

    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg)
    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env)
    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg["experiment_name"])
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")

    resume_path = get_checkpoint_path(log_root_path, agent_cfg["load_run"], agent_cfg["load_checkpoint"])
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # load previously trained model
    ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    ppo_runner.load(resume_path)
    print(f"[INFO]: Loading model checkpoint from: {resume_path}")

    # obtain the trained policy for inference
    policy = ppo_runner.get_inference_policy(device=env.unwrapped.device)

    # reset environment
    obs, _ = env.get_observations()

    # initialize ROS2 node
    rclpy.init()
    base_node = RobotBaseNode()
    lidar_sensor = LidarRtx(f'/World/envs/env_0/Robot/base/lidar_sensor',
                                         translation=(0.28945, 0, -0.046825),
                                         orientation=(1.0, 0.0, 0.0, 0.0),
                                         config_file_name= "Unitree_L1",
                                         )
    cameraCfg = CameraCfg(
        prim_path="/World/envs/env_0/Robot/base/front_cam",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.32487, -0.00095, 0.05362), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    )
    Camera(cameraCfg)
    # Create the debug draw pipeline in the post process graph
    # writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloudBuffer")
    # writer.attach([lidar_sensor.get_render_product_path()])

    annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
    annotator.attach(lidar_sensor.get_render_product_path())

    start_time = time.time()

    setup_custom_env()

    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # agent stepping
            actions = policy(obs)

            # env stepping
            obs, _, _, _ = env.step(actions)

            # publish ros2 info
            base_node.publish_joints(env.env.scene["robot"].data.joint_names, env.env.scene["robot"].data.joint_pos[0])
            base_node.publish_odom(env.env.scene["robot"].data.root_state_w[0, :3], env.env.scene["robot"].data.root_state_w[0, 3:7])
            base_node.publish_robot_state([
                env.env.scene["contact_forces"].data.net_forces_w[0][4][2], 
                env.env.scene["contact_forces"].data.net_forces_w[0][8][2], 
                env.env.scene["contact_forces"].data.net_forces_w[0][14][2], 
                env.env.scene["contact_forces"].data.net_forces_w[0][18][2]
                ])
    
            try:
                if (time.time() - start_time) > 1/20:
                    data = annotator.get_data()
                    point_cloud = update_meshes_for_cloud2(
                        data['data'], env.env.scene["robot"].data.root_state_w[0, :3], 
                        env.env.scene["robot"].data.root_state_w[0, 3:7]
                        )
                    base_node.publish_lidar(point_cloud)
                    start_time = time.time()
            except :
                pass
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
