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

from __future__ import annotations

import argparse
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import RslRlOnPolicyRunnerCfg


def add_rsl_rl_args(parser: argparse.ArgumentParser):
    """Add RSL-RL arguments to the parser.

    Args:
        parser: The parser to add the arguments to.
    """
    # create a new argument group
    arg_group = parser.add_argument_group(
        "rsl_rl", description="Arguments for RSL-RL agent."
    )
    # -- experiment arguments
    arg_group.add_argument(
        "--experiment_name",
        type=str,
        default=None,
        help="Name of the experiment folder where logs will be stored.",
    )
    arg_group.add_argument(
        "--run_name",
        type=str,
        default=None,
        help="Run name suffix to the log directory.",
    )
    # -- load arguments
    arg_group.add_argument(
        "--resume", type=bool, default=None, help="Whether to resume from a checkpoint."
    )
    arg_group.add_argument(
        "--load_run",
        type=str,
        default=None,
        help="Name of the run folder to resume from.",
    )
    arg_group.add_argument(
        "--checkpoint", type=str, default=None, help="Checkpoint file to resume from."
    )
    # -- logger arguments
    arg_group.add_argument(
        "--logger",
        type=str,
        default=None,
        choices={"wandb", "tensorboard", "neptune"},
        help="Logger module to use.",
    )
    arg_group.add_argument(
        "--log_project_name",
        type=str,
        default=None,
        help="Name of the logging project when using wandb or neptune.",
    )


def parse_rsl_rl_cfg(
    task_name: str, args_cli: argparse.Namespace
) -> RslRlOnPolicyRunnerCfg:
    """Parse configuration for RSL-RL agent based on inputs.

    Args:
        task_name: The name of the environment.
        args_cli: The command line arguments.

    Returns:
        The parsed configuration for RSL-RL agent based on inputs.
    """
    from omni.isaac.lab_tasks.utils.parse_cfg import load_cfg_from_registry

    # load the default configuration
    rslrl_cfg: RslRlOnPolicyRunnerCfg = load_cfg_from_registry(
        task_name, "rsl_rl_cfg_entry_point"
    )

    # override the default configuration with CLI arguments
    if args_cli.seed is not None:
        rslrl_cfg.seed = args_cli.seed
    if args_cli.resume is not None:
        rslrl_cfg.resume = args_cli.resume
    if args_cli.load_run is not None:
        rslrl_cfg.load_run = args_cli.load_run
    if args_cli.checkpoint is not None:
        rslrl_cfg.load_checkpoint = args_cli.checkpoint
    if args_cli.run_name is not None:
        rslrl_cfg.run_name = args_cli.run_name
    if args_cli.logger is not None:
        rslrl_cfg.logger = args_cli.logger
    # set the project name for wandb and neptune
    if rslrl_cfg.logger in {"wandb", "neptune"} and args_cli.log_project_name:
        rslrl_cfg.wandb_project = args_cli.log_project_name
        rslrl_cfg.neptune_project = args_cli.log_project_name

    return rslrl_cfg
