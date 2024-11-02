import argparse
from isaacsim import SimulationApp

# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates different legged robots.")
parser.add_argument("--num_envs", type=int, default=1, help="Number of environments to spawn.")
args_cli = parser.parse_args()

# # launch omniverse app
simulation_app = SimulationApp({"headless": False, "anti_aliasing": 0})


import rclpy
import torch
import omni
import carb
import go2_ctrl
import go2_ros2_bridge
from go2_env import Go2RSLEnvCfg
import go2_sensors
import time

def run_simulator():
    from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
    from omni.isaac.nucleus import get_assets_root_path

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        carb.log_error("Could not find Isaac Sim assets folder")

    prim = get_prim_at_path("/World/Warehouse")
    if not prim.IsValid():
        prim = define_prim("/World/Warehouse", "Xform")
        asset_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"
        prim.GetReferences().AddReference(asset_path)

    # Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = args_cli.num_envs
    go2_ctrl.init_base_vel_cmd(args_cli.num_envs)
    # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)


    # Sensor setup
    sm = go2_sensors.SensorManager(args_cli.num_envs)
    lidar_annotators = sm.add_rtx_lidar()
    cameras = sm.add_camera()

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(
        omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)
    
    # ROS2 Bridge
    rclpy.init()
    dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras)

    # Run simulation
    sim_step_dt = go2_env_cfg.sim.dt * go2_env_cfg.decimation
    obs, _ = env.reset()
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        rclpy.spin_once(dm)
        actual_loop_time = time.time() - start_time
        print("actual_loop_time: ", actual_loop_time)


if __name__ == "__main__":
    run_simulator()
    simulation_app.close()