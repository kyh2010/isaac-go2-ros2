from isaacsim import SimulationApp

# launch omniverse app
simulation_app = SimulationApp({"headless": False, "anti_aliasing": 0,
                                "width": 1280, "height": 720, 
                                "hide_ui": False})
import rclpy
import torch
import omni
import carb
import go2.go2_ctrl as go2_ctrl
import ros2.go2_ros2_bridge as go2_ros2_bridge
from go2.go2_env import Go2RSLEnvCfg, camera_follow
import env.sim_env as sim_env
import go2.go2_sensors as go2_sensors
import time
import os
import hydra

FILE_PATH = os.path.join(os.path.dirname(__file__), "cfg")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    # Go2 Environment setup
    go2_env_cfg = Go2RSLEnvCfg()
    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    # env, policy = go2_ctrl.get_rsl_flat_policy(go2_env_cfg)
    env, policy = go2_ctrl.get_rsl_rough_policy(go2_env_cfg)

    # Simulation environment
    if (cfg.env_name == "obstacle"):
        sim_env.create_obstacle_env() # obstacles
    elif (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env() # warehouse
    elif (cfg.env_name == "warehouse_forklifts"):
        sim_env.create_warehouse_forklifts_env() # warehouse forklifts
    elif (cfg.env_name == "warehouse_shelves"):
        sim_env.create_warehouse_shelves_env() # warehouse shelves
    elif (cfg.env_name == "full_warehouse"):
        sim_env.create_full_warehouse_env() # full warehouse

    # Sensor setup
    sm = go2_sensors.SensorManager(cfg.num_envs)
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
    sim_step_dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    obs, _ = env.reset()
    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():            
            # control joints
            actions = policy(obs)

            # step the environment
            obs, _, _, _ = env.step(actions)

            # # ROS2 data
            dm.pub_ros2_data()

            # Camera follow
            if (cfg.camera_follow):
                camera_follow(env)

            # limit loop time
            elapsed_time = time.time() - start_time
            if elapsed_time < sim_step_dt:
                sleep_duration = sim_step_dt - elapsed_time
                time.sleep(sleep_duration)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, sim_step_dt/elapsed_time)
        print(f"\rStep time: {actual_loop_time*1000:.2g}ms, Real Time Factor: {rtf:.2g}", end='', flush=True)


if __name__ == "__main__":
    run_simulator()
    simulation_app.close()