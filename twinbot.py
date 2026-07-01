"""Twinbot subscriber — runs inside the Isaac Sim process.

Consumes `/real_dog/joint_states` (joint positions in SDK motor order) and
`/real_dog/odom` (base orientation from the IMU) published by
`twinbot_bridge.py` on the Jetson, and overwrites the sim articulation's
joint state and root pose each frame.

This is kinematic playback — physics is bypassed so the sim dog mirrors
the real dog exactly regardless of contact forces or policy feedback.
"""
from __future__ import annotations

import threading
from typing import Optional

import torch
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import rclpy


class TwinbotSubscriber:
    """Thread-safe mirror of the real Go2's joint + base state."""

    def __init__(self, env, pin_xy: bool = True):
        self._env = env
        self._lock = threading.Lock()
        self._latest_pos: Optional[torch.Tensor] = None   # [1, num_joints] sim DOF order
        self._latest_vel: Optional[torch.Tensor] = None
        self._latest_quat: Optional[torch.Tensor] = None  # [1, 4] (w, x, y, z)
        self._latest_ang_vel: Optional[torch.Tensor] = None  # [1, 3]

        robot = env.unwrapped.scene["robot"]
        self._sim_names = robot.data.joint_names
        self._device = str(env.unwrapped.device)

        # Fixed sim-world XY for the base (we don't integrate IMU to position).
        # The sim dog rotates with the real dog but stays visible at spawn XY.
        self._pin_xy = pin_xy
        default_root = robot.data.default_root_state[0].clone()  # (13,) pos+quat+linvel+angvel
        self._spawn_pos = default_root[0:3].clone().to(self._device)

        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node as RclNode

        self._ctx = Context()
        rclpy.init(context=self._ctx)
        self._node = RclNode("twinbot_subscriber", context=self._ctx)
        self._node.create_subscription(JointState, "/real_dog/joint_states", self._cb_joints, 10)
        self._node.create_subscription(Odometry, "/real_dog/odom", self._cb_odom, 10)
        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._node.get_logger().info(
            "TwinbotSubscriber ready — listening /real_dog/joint_states + /real_dog/odom"
        )

    def _cb_joints(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}
        robot = self._env.unwrapped.scene["robot"]
        default_pos = robot.data.default_joint_pos[0]
        ordered_pos = [name_to_pos.get(n, float(default_pos[i])) for i, n in enumerate(self._sim_names)]
        ordered_vel = [name_to_vel.get(n, 0.0) for n in self._sim_names]
        pos = torch.tensor(ordered_pos, dtype=torch.float32, device=self._device).unsqueeze(0)
        vel = torch.tensor(ordered_vel, dtype=torch.float32, device=self._device).unsqueeze(0)
        with self._lock:
            self._latest_pos = pos
            self._latest_vel = vel

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        quat = torch.tensor([[q.w, q.x, q.y, q.z]], dtype=torch.float32, device=self._device)
        w = msg.twist.twist.angular
        ang = torch.tensor([[w.x, w.y, w.z]], dtype=torch.float32, device=self._device)
        with self._lock:
            self._latest_quat = quat
            self._latest_ang_vel = ang

    def apply(self, device: str) -> bool:
        """Kinematic playback: overwrite joint state + root pose this frame.

        Returns True if any real-robot data has been applied.
        """
        with self._lock:
            pos = self._latest_pos
            vel = self._latest_vel
            quat = self._latest_quat
            ang = self._latest_ang_vel

        if pos is None and quat is None:
            return False

        robot = self._env.unwrapped.scene["robot"]

        if pos is not None:
            vel_in = vel if vel is not None else torch.zeros_like(pos)
            robot.write_joint_state_to_sim(pos.to(device), vel_in.to(device))

        if quat is not None and self._pin_xy:
            # Root pose = [px, py, pz, qw, qx, qy, qz]
            root_pose = torch.cat([self._spawn_pos.to(device).unsqueeze(0), quat.to(device)], dim=1)
            robot.write_root_pose_to_sim(root_pose)
            # Root velocity: zero linear, real angular
            ang_in = ang if ang is not None else torch.zeros(1, 3, device=device)
            root_vel = torch.cat([torch.zeros(1, 3, device=device), ang_in.to(device)], dim=1)
            robot.write_root_velocity_to_sim(root_vel)

        return True
