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

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg


G1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="./robots/g1/g1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=4,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.70),
        joint_pos={
            ".*_hip_pitch_joint": -0.28,
            ".*_knee_joint": 0.63,
            ".*_ankle_pitch_joint": -0.35,
            ".*_elbow_pitch_joint": 0.87,
            "left_shoulder_roll_joint": 0.16,
            "left_shoulder_pitch_joint": 0.35,
            "right_shoulder_roll_joint": -0.16,
            "right_shoulder_pitch_joint": 0.35,
            "left_one_joint": 1.0,
            "right_one_joint": -1.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
                "torso_joint",
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*_hip_yaw_joint": 150.0,
                ".*_hip_roll_joint": 150.0,
                ".*_hip_pitch_joint": 200.0,
                ".*_knee_joint": 200.0,
                "torso_joint": 200.0,
            },
            damping={
                ".*_hip_yaw_joint": 5.0,
                ".*_hip_roll_joint": 5.0,
                ".*_hip_pitch_joint": 5.0,
                ".*_knee_joint": 5.0,
                "torso_joint": 5.0,
            },
            armature={
                ".*_hip_yaw_joint": 0.1,
                ".*_hip_roll_joint": 0.1,
                ".*_hip_pitch_joint": 0.1,
                ".*_knee_joint": 0.1,
                "torso_joint": 0.1,
            },
        ),
        "feet": ImplicitActuatorCfg(
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness={".*_ankle_pitch_joint": 20.0, ".*_ankle_roll_joint": 20.0},
            damping={".*_ankle_pitch_joint": 4.0, ".*_ankle_roll_joint": 4.0},
            armature={".*_ankle_pitch_joint": 0.1, ".*_ankle_roll_joint": 0.1},
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_pitch_joint",
                ".*_elbow_roll_joint",
                ".*_five_joint",
                ".*_three_joint",
                ".*_six_joint",
                ".*_four_joint",
                ".*_zero_joint",
                ".*_one_joint",
                ".*_two_joint",
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                ".*_shoulder_pitch_joint": 40.0,
                ".*_shoulder_roll_joint": 40.0,
                ".*_shoulder_yaw_joint": 40.0,
                ".*_elbow_pitch_joint": 40.0,
                ".*_elbow_roll_joint": 40.0,
                ".*_five_joint": 40.0,
                ".*_three_joint": 40.0,
                ".*_six_joint": 40.0,
                ".*_four_joint": 40.0,
                ".*_zero_joint": 40.0,
                ".*_one_joint": 40.0,
                ".*_two_joint": 40.0,
            },
            damping={
                ".*_shoulder_pitch_joint": 10.0,
                ".*_shoulder_roll_joint": 10.0,
                ".*_shoulder_yaw_joint": 10.0,
                ".*_elbow_pitch_joint": 10.0,
                ".*_elbow_roll_joint": 10.0,
                ".*_five_joint": 10.0,
                ".*_three_joint": 10.0,
                ".*_six_joint": 10.0,
                ".*_four_joint": 10.0,
                ".*_zero_joint": 10.0,
                ".*_one_joint": 10.0,
                ".*_two_joint": 10.0,
            },
            armature={
                ".*_shoulder_pitch_joint": 0.1,
                ".*_shoulder_roll_joint": 0.1,
                ".*_shoulder_yaw_joint": 0.1,
                ".*_elbow_pitch_joint": 0.1,
                ".*_elbow_roll_joint": 0.1,
                ".*_five_joint": 0.1,
                ".*_three_joint": 0.1,
                ".*_six_joint": 0.1,
                ".*_four_joint": 0.1,
                ".*_zero_joint": 0.1,
                ".*_one_joint": 0.1,
                ".*_two_joint": 0.1,
            },
        ),
    },
)
"""Configuration for the Unitree G1 Humanoid robot."""
