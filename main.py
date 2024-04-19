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



# local imports
import cli_args  # isort: skip


# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--cpu", action="store_true", default=False, help="Use CPU pipeline.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default="Isaac-Velocity-Rough-Unitree-Go2-v0", help="Name of the task.")
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")


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


import os
import math
import gymnasium as gym
import torch
import carb
import usdrt.Sdf


from omni.isaac.orbit_tasks.utils import get_checkpoint_path
from omni.isaac.orbit_tasks.utils.wrappers.rsl_rl import (
    RslRlOnPolicyRunnerCfg,
    RslRlVecEnvWrapper
)
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit_assets.unitree import UNITREE_GO2_CFG 
from omni.isaac.orbit.envs import RLTaskEnvCfg
import omni.isaac.orbit.sim as sim_utils
from omni.isaac.orbit.assets import ArticulationCfg, AssetBaseCfg
from omni.isaac.orbit.managers import CurriculumTermCfg as CurrTerm
from omni.isaac.orbit.managers import EventTermCfg as EventTerm
from omni.isaac.orbit.managers import ObservationGroupCfg as ObsGroup
from omni.isaac.orbit.managers import ObservationTermCfg as ObsTerm
from omni.isaac.orbit.managers import RewardTermCfg as RewTerm
from omni.isaac.orbit.managers import SceneEntityCfg
from omni.isaac.orbit.managers import TerminationTermCfg as DoneTerm
from omni.isaac.orbit.scene import InteractiveSceneCfg
from omni.isaac.orbit.sensors import ContactSensorCfg, RayCasterCfg, patterns, CameraCfg
from omni.isaac.sensor import LidarRtx

from omni.isaac.orbit.terrains import TerrainImporterCfg
from omni.isaac.orbit.utils import configclass
from omni.isaac.orbit.utils.noise import AdditiveUniformNoiseCfg as Unoise
import omni.isaac.orbit_tasks.locomotion.velocity.mdp as mdp
import omni.appwindow  # Contains handle to keyboard
from rsl_rl.runners import OnPolicyRunner


from typing import Literal
from dataclasses import MISSING
from scipy.spatial.transform import Rotation


from omnigraph import create_front_cam_omnigraph
from agent_cfg import unitree_go2_agent_cfg
from terrain_cfg import ROUGH_TERRAINS_CFG


import rclpy
from ros2 import RobotBaseNode
from pxr import Gf
import omni.replicator.core as rep


base_command = [0, 0, 0]


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with a legged robot."""

    # ground terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=ROUGH_TERRAINS_CFG,
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/Base/Architecture/Shingles_01.mdl",
            project_uvw=True,
        ),
        debug_vis=False,
    )
    # robots
    robot: ArticulationCfg = MISSING

    # sensors
    camera = CameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base/front_cam",
        update_period=0.1,
        height=480,
        width=640,
        data_types=["rgb", "distance_to_image_plane"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
        ),
        offset=CameraCfg.OffsetCfg(pos=(0.32487, -0.00095, 0.05362), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
    )

    height_scanner = RayCasterCfg(
        prim_path="{ENV_REGEX_NS}/Robot/base",
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        attach_yaw_only=True,
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
        debug_vis=False,
        mesh_prim_paths=["/World/ground"],
    )

    contact_forces = ContactSensorCfg(prim_path="{ENV_REGEX_NS}/Robot/.*", history_length=3, track_air_time=True)
    
    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    sky_light = AssetBaseCfg(
        prim_path="/World/skyLight",
        spawn=sim_utils.DomeLightCfg(color=(0.13, 0.13, 0.13), intensity=1000.0),
    )


def constant_commands(env: RLTaskEnvCfg) -> torch.Tensor:
    global base_command
    """The generated command from the command generator."""
    return torch.tensor([base_command], device=env.device).repeat(env.num_envs, 1)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""
    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        projected_gravity = ObsTerm(
            func=mdp.projected_gravity,
            noise=Unoise(n_min=-0.05, n_max=0.05),
        )
        velocity_commands = ObsTerm(func=constant_commands)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        actions = ObsTerm(func=mdp.last_action)
        height_scan = ObsTerm(
            func=mdp.height_scan,
            params={"sensor_cfg": SceneEntityCfg("height_scanner")},
            clip=(-1.0, 1.0),
        )

        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""
    joint_pos = mdp.JointPositionActionCfg(asset_name="robot", joint_names=[".*"], scale=0.5, use_default_offset=True)


@configclass
class CommandsCfg:
    """Command specifications for the MDP."""
    base_velocity = mdp.UniformVelocityCommandCfg(
        asset_name="robot",
        resampling_time_range=(0.0, 0.0),
        rel_standing_envs=0.02,
        rel_heading_envs=1.0,
        heading_command=True,
        heading_control_stiffness=0.5,
        debug_vis=True,
        ranges=mdp.UniformVelocityCommandCfg.Ranges(
            lin_vel_x=(0.0, 0.0), lin_vel_y=(0.0, 0.0), ang_vel_z=(0.0, 0.0), heading=(0, 0)
        ),
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # -- task
    track_lin_vel_xy_exp = RewTerm(
        func=mdp.track_lin_vel_xy_exp, weight=1.0, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    track_ang_vel_z_exp = RewTerm(
        func=mdp.track_ang_vel_z_exp, weight=0.5, params={"command_name": "base_velocity", "std": math.sqrt(0.25)}
    )
    # -- penalties
    lin_vel_z_l2 = RewTerm(func=mdp.lin_vel_z_l2, weight=-2.0)
    ang_vel_xy_l2 = RewTerm(func=mdp.ang_vel_xy_l2, weight=-0.05)
    dof_torques_l2 = RewTerm(func=mdp.joint_torques_l2, weight=-1.0e-5)
    dof_acc_l2 = RewTerm(func=mdp.joint_acc_l2, weight=-2.5e-7)
    action_rate_l2 = RewTerm(func=mdp.action_rate_l2, weight=-0.01)
    feet_air_time = RewTerm(
        func=mdp.feet_air_time,
        weight=0.125,
        params={
            "sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*FOOT"),
            "command_name": "base_velocity",
            "threshold": 0.5,
        },
    )
    undesired_contacts = RewTerm(
        func=mdp.undesired_contacts,
        weight=-1.0,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names=".*THIGH"), "threshold": 1.0},
    )
    # -- optional penalties
    flat_orientation_l2 = RewTerm(func=mdp.flat_orientation_l2, weight=0.0)
    dof_pos_limits = RewTerm(func=mdp.joint_pos_limits, weight=0.0)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    base_contact = DoneTerm(
        func=mdp.illegal_contact,
        params={"sensor_cfg": SceneEntityCfg("contact_forces", body_names="base"), "threshold": 1.0},
    )


@configclass
class EventCfg:
    """Configuration for events."""
    # startup
    physics_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
            "static_friction_range": (0.8, 0.8),
            "dynamic_friction_range": (0.6, 0.6),
            "restitution_range": (0.0, 0.0),
            "num_buckets": 64,
        },
    )

@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""
    terrain_levels = CurrTerm(func=mdp.terrain_levels_vel)

@configclass
class ViewerCfg:
    """Configuration of the scene viewport camera."""
    eye: tuple[float, float, float] = (7.5, 7.5, 7.5)

    lookat: tuple[float, float, float] = (0.0, 0.0, 0.0)

    cam_prim_path: str = "/OmniverseKit_Persp"

    resolution: tuple[int, int] = (1920, 1080)

    origin_type: Literal["world", "env", "asset_root"] = "world"

    env_index: int = 0

    asset_name: str | None = None


@configclass
class LocomotionVelocityRoughEnvCfg(RLTaskEnvCfg):
    """Configuration for the locomotion velocity-tracking environment."""
    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=2.5)
    viewer: ViewerCfg = ViewerCfg()
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 4
        self.episode_length_s = 20.0
        # simulation settings
        self.sim.dt = 0.005
        self.sim.disable_contact_processing = True
        self.sim.physics_material = self.scene.terrain.physics_material
        # update sensor update periods
        # we tick all the sensors based on the smallest update period (physics update period)
        if self.scene.height_scanner is not None:
            self.scene.height_scanner.update_period = self.decimation * self.sim.dt
        
        if self.scene.contact_forces is not None:
            self.scene.contact_forces.update_period = self.sim.dt
        
        # check if terrain levels curriculum is enabled - if so, enable curriculum for terrain generator
        # this generates terrains with increasing difficulty and is useful for training
        if getattr(self.curriculum, "terrain_levels", None) is not None:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = True
        else:
            if self.scene.terrain.terrain_generator is not None:
                self.scene.terrain.terrain_generator.curriculum = False

@configclass
class UnitreeGo2RoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        self.scene.robot = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/base"

        # reduce action scale
        self.actions.joint_pos.scale = 0.25

        # rewards
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_foot"
        self.rewards.feet_air_time.weight = 0.01
        self.rewards.undesired_contacts = None
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        self.rewards.dof_acc_l2.weight = -2.5e-7

        # terminations
        self.terminations.base_contact.params["sensor_cfg"].body_names = "base"

        #create ros2 camera stream omnigraph
        create_front_cam_omnigraph()


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


def main():

    # acquire input interface
    _input = carb.input.acquire_input_interface()
    _appwindow = omni.appwindow.get_default_app_window()
    _keyboard = _appwindow.get_keyboard()
    _sub_keyboard = _input.subscribe_to_keyboard_events(_keyboard, sub_keyboard_event)

    """Play with RSL-RL agent."""
    # parse configuration
    env_cfg = UnitreeGo2RoughEnvCfg()
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

    # Create the debug draw pipeline in the post process graph
    writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloudBuffer")
    writer.attach([lidar_sensor.get_render_product_path()])

    annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
    annotator.attach(lidar_sensor.get_render_product_path())

    import time
    start_time = time.time()

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
                    print(f"data: {data['data'].shape}")
                    point_cloud = update_meshes_for_cloud2(
                        data['data'], env.env.scene["robot"].data.root_state_w[0, :3], 
                        env.env.scene["robot"].data.root_state_w[0, 3:7]
                        )
                    base_node.publish_lidar(point_cloud)
                    start_time = time.time()
            except :
                pass
            
    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()