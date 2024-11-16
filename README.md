#  Isaac Sim Unitree Go2
[![Python](https://img.shields.io/badge/python-3.10-blue.svg)](https://docs.python.org/3/whatsnew/3.10.html)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-orange.svg)](https://docs.ros.org/en/humble/index.html)
[![IsaacSim](https://img.shields.io/badge/IsaacSim-4.2.0-red.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Linux platform](https://img.shields.io/badge/platform-Ubuntu--22.04-green.svg)](https://releases.ubuntu.com/22.04/)

Welcome to the Isaac Sim Unitree Go2 repository! This repository provides a Unitree Go2 quadruped robot simulator, leveraging the powerful Isaac Sim/Isaac Lab framework and integrating seamlessly with a ROS 2 interface. It offers a flexible platform for testing navigation, decision-making, and other autonomous tasks in various scenarios.
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/8a3fb64a-966c-43f1-9124-f95ba060adff" alt="go2 navigation - gif" style="width: 100%;"></td>
    <td><img src="https://github.com/user-attachments/assets/e8aab9ea-a7aa-4d31-9c12-65e0b0159ac3" alt="warehouse demo - gif" style="width: 100%;"></td>
  </tr>
</table>

## Installation Guide
**Step I:** Please follow the latest [Isaac Lab official documentation](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/binaries_installation.html) to install the Isaac Sim and Isaac Lab.

**Step II:** Please install [ROS2 Humble](https://docs.ros.org/en/humble/index.html) with the official installation guide.

**Step III:** Install the prerequisite C extension in the conda environment. [reference link](https://stackoverflow.com/questions/58424974/anaconda-importerror-usr-lib64-libstdc-so-6-version-glibcxx-3-4-21-not-fo)
```
# default conda env for Isaac Lab
conda activate isaaclab      
conda install -c conda-forge libstdcxx-ng
```

**Step IV:** Clone this repo to your local directory.
```
git clone https://github.com/Zhefan-Xu/isaac-go2-ros2.git
```

## Run Simulator 
To run the simulator, please use the following command:
```
conda activate isaaclab
python isaac_go2_ros2.py
```
Once the simulator is loaded, the robot can be teleoperated by the keyboard:

```W```: Forward, ```A```: Left, ```S```: Backward, ```D```: Right, ```Z```: Left Turn, ```C```: Right Turn.


https://github.com/user-attachments/assets/7abb41fd-26f7-4e5d-bc7f-98ee10467a6a



## Simulation Environments & settings
The simulation environments and settings can be changed in ```isaac-go2-ros2/cfg/sim.yaml``` config file. 

#### Launch different simulation environments
The current implementation contains a few environments which can be found on ```isaac-go2-ros2/env/sim_env.py```, which follows standard Isaac Sim method for importing USD environments. To change the environment, please change the ```env_name``` in the config file ```isaac-go2-ros2/cfg/sim.yaml```. Current available environments:
- ```warehouse```: A simple warehouse environment in Isaac Sim.
- ```warehouse_forklifts```: A warehouse environment with forklifts.
- ```warehouse_shelves```: A warehouse environment with shelves.
- ```full_warehouse```: A full warehouse environment containing everything.
- ```obstacle-sparse```: A sparse obstacle field environment.
- ```obstacle-medium```: A  medium obstacle field environment.
- ```obstacle-dense```: A dense obstacle field environment.


#### Launch multiple robots in the environment
This repository supports running multiple Unitree Go2 environment and you can change the number of robots by changing the ```num_envs``` parameter in the config file ```isaac-go2-ros2/cfg/sim.yaml```.

https://github.com/user-attachments/assets/47ef05c1-5124-4feb-afc8-a3f2c306a212




## Example Usage
The following video shows an example of using this simulator with an RL agent to achieve navigation and collision avoidance:


https://github.com/user-attachments/assets/ccc986c6-bf94-41fe-a4d5-3417ce8b3384





