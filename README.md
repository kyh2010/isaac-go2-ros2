#  Isaac Sim Unitree Go2 with ROS2
Welcome to the Isaac Sim Unitree Go2 repository. This repo implements the Unitree Go2 quadruped using Isaac Sim/IsaacLab with ROS2 interface. 
<table>
  <tr>
    <td><img src="https://github.com/user-attachments/assets/8a3fb64a-966c-43f1-9124-f95ba060adff" alt="go2 navigation - gif" style="width: 100%;"></td>
    <td><img src="https://github.com/user-attachments/assets/e8aab9ea-a7aa-4d31-9c12-65e0b0159ac3" alt="warehouse demo - gif" style="width: 100%;"></td>
  </tr>
</table>

### Common Issue
If the following issue is encountered:
```
ImportError: /home/zhefan/miniconda3/envs/isaaclab/bin/../lib/libstdc++.so.6: version `GLIBCXX_3.4.30' not found (required by /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so)
The C extension '/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so' failed to be imported while being present on the system. Please refer to 'https://docs.ros.org/en/{distro}/Guides/Installation-Troubleshooting.html#import-failing-even-with-library-present-on-the-system' for possible solutions
```
Please run: 
```
conda install -c conda-forge libstdcxx-ng
```
reference: [link](https://stackoverflow.com/questions/58424974/anaconda-importerror-usr-lib64-libstdc-so-6-version-glibcxx-3-4-21-not-fo)


## TODO:
4. Environment (todo)
