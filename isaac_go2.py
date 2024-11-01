import argparse
from isaacsim import SimulationApp
from omni.isaac.lab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
parser.add_argument("--anti_aliasing", type=int, default=0, help="Anti Aliasing.")
parser.add_argument("--height", type=int, default=1080, help="Resolution Height.")
parser.add_argument("--width", type=int, default=1920, help="Resolution Width.")

# append AppLauncher cli args
# AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
# app_launcher = AppLauncher(args_cli)
# simulation_app = app_launcher.app
simulation_app = SimulationApp({"headless": False, "anti_aliasing": 0})


import rclpy
import torch
import omni
import carb
import go2_ctrl
import go2_ros2_bridge
from go2_env import Go2RSLEnvCfg
import go2_sensors
import threading
import time

def run_simulator():
    # from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
    # from omni.isaac.nucleus import get_assets_root_path

    # assets_root_path = get_assets_root_path()
    # if assets_root_path is None:
    #     carb.log_error("Could not find Isaac Sim assets folder")

    # prim = get_prim_at_path("/World/Warehouse")
    # if not prim.IsValid():
    #     prim = define_prim("/World/Warehouse", "Xform")
    #     asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    #     prim.GetReferences().AddReference(asset_path)

    # Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = args_cli.num_envs
    go2_ctrl.init_base_vel_cmd(args_cli.num_envs)
    sim_step_dt = go2_env_cfg.sim.dt * go2_env_cfg.decimation
    # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)
    lidar_annotators = go2_sensors.add_rtx_lidar(args_cli.num_envs)

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)
    
    # ROS2 Bridge
    rclpy.init()
    dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators)
    thread = threading.Thread(target=rclpy.spin, args=(dm,), daemon=True)
    thread.start()    

    # Run simulation
    obs, _ = env.reset()
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            # publish to ROS2
            dm.pub_ros2_data()

            end_time = time.time()
            elapsed_time = end_time - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        actual_loop_time = time.time() - start_time
        print("actual_loop_time: ", actual_loop_time)


if __name__ == "__main__":
    run_simulator()
    simulation_app.close()