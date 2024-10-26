import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import torch
import gymnasium as gym
import go2_ctrl
from go2_env import Go2EnvCfg
from ctrl_agent_cfg import unitree_go2_agent_cfg
from omni.isaac.lab_tasks.utils.wrappers.rsl_rl import RslRlVecEnvWrapper, RslRlOnPolicyRunnerCfg
from omni.isaac.lab_tasks.utils import get_checkpoint_path
from rsl_rl.runners import OnPolicyRunner

def run_simulator():
    # Environment setup
    go2_env_cfg = Go2EnvCfg()
    go2_env_cfg.scene.num_envs = args_cli.num_envs
    go2_ctrl.init_base_vel_cmd(args_cli.num_envs)
    env = gym.make("Isaac-Velocity-Rough-Unitree-Go2-v0", cfg=go2_env_cfg)
    env = RslRlVecEnvWrapper(env)

    # rsl control policy
    agent_cfg: RslRlOnPolicyRunnerCfg = unitree_go2_agent_cfg
    ckpt_path = get_checkpoint_path(log_path=os.path.abspath("ckpts"), 
                                    run_dir=agent_cfg["load_run"], checkpoint=agent_cfg["load_checkpoint"])
    ppo_runner = OnPolicyRunner(env, agent_cfg, log_dir=None, device=agent_cfg["device"])
    ppo_runner.load(ckpt_path)
    policy = ppo_runner.get_inference_policy(device=agent_cfg["device"])

    obs, _ = env.reset()
    simulation_count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # sample random action
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)
            simulation_count += 1


def main():
    run_simulator()

if __name__ == "__main__":
    main()
    simulation_app.close()