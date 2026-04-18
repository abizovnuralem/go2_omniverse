"""Twinbot subscriber — runs inside the Isaac Sim process.

Subscribes to /real_dog/joint_states (published by twinbot_bridge.py on Jetson)
and exposes the latest real-robot joint positions in the sim's joint-name order.

Usage:
    from twinbot import TwinbotSubscriber
    twin = TwinbotSubscriber(env)          # call after rclpy.init()
    ...
    real_actions = twin.actions(device)    # in the main loop, replaces policy(obs)
"""
from __future__ import annotations

import threading
from typing import Optional

import torch
from sensor_msgs.msg import JointState
import rclpy


class TwinbotSubscriber:
    """Thread-safe mirror of the real Go2's joint state."""

    def __init__(self, env):
        self._env = env
        self._lock = threading.Lock()
        self._latest_pos: Optional[torch.Tensor] = None  # [1, num_joints] in sim DOF order

        # Build real-dog name → sim DOF index map once
        robot = env.unwrapped.scene["robot"]
        sim_names = robot.data.joint_names          # list[str], sim DOF order
        self._sim_names = sim_names
        self._device = str(env.unwrapped.device)

        # Not used for direct writes, kept for reference only.
        self._action_scale = 0.5

        # Use an isolated rclpy Context + SingleThreadedExecutor so this node's
        # spin thread never touches the shared wait-set used by add_cmd_sub().
        # (Two concurrent rclpy.spin() calls on the default context raise
        # "ValueError: generator already executing".)
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node as RclNode

        self._ctx = Context()
        rclpy.init(context=self._ctx)
        self._node = RclNode("twinbot_subscriber", context=self._ctx)
        self._node.create_subscription(
            JointState, "/real_dog/joint_states", self._cb, 10
        )
        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._node.get_logger().info(
            f"TwinbotSubscriber ready — action_scale={self._action_scale}, "
            f"listening on /real_dog/joint_states"
        )

    def _cb(self, msg: JointState):
        name_to_pos = dict(zip(msg.name, msg.position))
        # reorder to sim DOF order; joints not in msg fall back to default
        robot = self._env.unwrapped.scene["robot"]
        default_pos = robot.data.default_joint_pos[0]   # [num_joints]
        ordered = [
            name_to_pos.get(n, float(default_pos[i]))
            for i, n in enumerate(self._sim_names)
        ]
        pos = torch.tensor(ordered, dtype=torch.float32, device=self._device).unsqueeze(0)
        with self._lock:
            self._latest_pos = pos

    def latest_pos(self, device: str) -> Optional[torch.Tensor]:
        """Return latest real-robot joint positions in sim DOF order, or None."""
        with self._lock:
            pos = self._latest_pos
        return pos.to(device) if pos is not None else None

    def apply(self, device: str) -> bool:
        """Write real-robot joint positions directly to the articulation.

        Bypasses the RL action space (avoids PD scale / offset issues).
        Returns True if a new position was applied, False if no data yet.
        """
        with self._lock:
            real_pos = self._latest_pos
        if real_pos is None:
            return False

        robot = self._env.unwrapped.scene["robot"]
        robot.set_joint_position_target(real_pos.to(device))
        return True

    def actions(self, device: str) -> Optional[torch.Tensor]:
        """Legacy: return env-space action that approximates real-robot pose.

        Kept as fallback; prefer apply() for accuracy.
        action = (real_pos - default_pos) / action_scale
        """
        with self._lock:
            real_pos = self._latest_pos
        if real_pos is None:
            return None

        robot = self._env.unwrapped.scene["robot"]
        default_pos = robot.data.default_joint_pos
        return (real_pos.to(device) - default_pos.to(device)) / self._action_scale
