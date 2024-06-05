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


unitree_go2_agent_cfg = {
        'seed': 42, 
        'device': 'cuda', 
        'num_steps_per_env': 24, 
        'max_iterations': 15000, 
        'empirical_normalization': False, 
        'policy': {
            'class_name': 'ActorCritic', 
            'init_noise_std': 1.0, 
            'actor_hidden_dims': [512, 256, 128], 
            'critic_hidden_dims': [512, 256, 128], 
            'activation': 'elu'
            }, 
        'algorithm': {
            'class_name': 'PPO', 
            'value_loss_coef': 1.0, 
            'use_clipped_value_loss': True, 
            'clip_param': 0.2, 
            'entropy_coef': 0.01, 
            'num_learning_epochs': 5, 
            'num_mini_batches': 4, 
            'learning_rate': 0.001, 
            'schedule': 'adaptive', 
            'gamma': 0.99, 
            'lam': 0.95, 
            'desired_kl': 0.01, 
            'max_grad_norm': 1.0
        }, 
        'save_interval': 50, 
        'experiment_name': 'unitree_go2_rough', 
        'run_name': '', 
        'logger': 'tensorboard', 
        'neptune_project': 'orbit', 
        'wandb_project': 'orbit', 
        'resume': False, 
        'load_run': '.*', 
        'load_checkpoint': 'model_.*.pt'
        }


unitree_g1_agent_cfg = {
        'seed': 42, 
        'device': 'cuda', 
        'num_steps_per_env': 24, 
        'max_iterations': 15000, 
        'empirical_normalization': False, 
        'policy': {
            'class_name': 'ActorCritic', 
            'init_noise_std': 1.0, 
            'actor_hidden_dims': [512, 256, 128], 
            'critic_hidden_dims': [512, 256, 128], 
            'activation': 'elu'
            }, 
        'algorithm': {
            'class_name': 'PPO', 
            'value_loss_coef': 1.0, 
            'use_clipped_value_loss': True, 
            'clip_param': 0.2, 
            'entropy_coef': 0.01, 
            'num_learning_epochs': 5, 
            'num_mini_batches': 4, 
            'learning_rate': 0.001, 
            'schedule': 'adaptive', 
            'gamma': 0.99, 
            'lam': 0.95, 
            'desired_kl': 0.01, 
            'max_grad_norm': 1.0
        }, 
        'save_interval': 50, 
        'experiment_name': 'g1_rough', 
        'run_name': '', 
        'logger': 'tensorboard', 
        'neptune_project': 'orbit', 
        'wandb_project': 'orbit', 
        'resume': False, 
        'load_run': '.*', 
        'load_checkpoint': 'model_.*.pt'
        }