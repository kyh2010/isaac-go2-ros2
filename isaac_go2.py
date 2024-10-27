import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--anti_aliasing", type=int, default=1, help="Anti Aliasing.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import os
import torch
import omni
import gymnasium as gym
import carb
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

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)


    # Run simulation
    obs, _ = env.reset()
    while simulation_app.is_running():
        with torch.inference_mode():
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

def main():
    run_simulator()

if __name__ == "__main__":
    main()
    simulation_app.close()