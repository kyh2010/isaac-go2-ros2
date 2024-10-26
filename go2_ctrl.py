import torch
from omni.isaac.lab.envs import ManagerBasedEnv

base_vel_cmd_input = {}

def init_base_vel_cmd(num_envs):
    global base_vel_cmd_input
    for i in range(num_envs):
        base_vel_cmd_input[str(i)] = [0, 0, 0]

def base_vel_cmd(env: ManagerBasedEnv) -> torch.Tensor:
    global base_vel_cmd_input
    """The generated command from the command generator."""
    cmd_vel_list = torch.tensor([0.0, 0.0, 0.0], dtype=torch.float32, device=env.device).repeat(env.num_envs, 1)
    for i in range(env.num_envs):
        cmd_vel_list[i] = torch.tensor(base_vel_cmd_input[str(i)], dtype=torch.float32, device=env.device)
    return cmd_vel_list