import argparse

from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--anti_aliasing", type=int, default=0, help="Anti Aliasing.")
parser.add_argument("--height", type=int, default=1080, help="Resolution Height.")
parser.add_argument("--width", type=int, default=1920, help="Resolution Width.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import rclpy
import torch
import omni
import carb
import go2_ctrl
import go2_ros2_bridge
from go2_env import Go2EnvCfg


def run_simulator():
    # Environment setup
    go2_env_cfg = Go2EnvCfg()
    go2_env_cfg.scene.num_envs = args_cli.num_envs
    go2_ctrl.init_base_vel_cmd(args_cli.num_envs)
    env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    # env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)
    
    # ROS2 Bridge
    rclpy.init()
    dm = go2_ros2_bridge.RobotDataManager(env)

    # Run simulation
    obs, _ = env.reset()
    while simulation_app.is_running():
        with torch.inference_mode():
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            # publish to ROS2
            dm.pub_ros2_data()


if __name__ == "__main__":
    run_simulator()
    simulation_app.close()