import omni
import numpy as np
from pxr import Gf
import omni.replicator.core as rep
from omni.isaac.sensor import Camera, IMUSensor
import omni.isaac.core.utils.numpy.rotations as rot_utils

class SensorManager:
    def __init__(self, num_envs):
        self.num_envs = num_envs

    def add_rtx_lidar(self):
        lidar_annotators = []
        debug = True
        for env_idx in range(self.num_envs):
            # # Unitree L1 Lidar
            # _, sensor = omni.kit.commands.execute(
            #     "IsaacSensorCreateRtxLidar",
            #     path="/lidar_L1",
            #     parent=f"/World/envs/env_{env_idx}/Go2/base",
            #     config="Unitree_L1",
            #     # config="Velodyne_VLS128",
            #     translation=(0.28945, 0, -0.046825),
            #     orientation=Gf.Quatd(0.13132, 0.0, 0.99134, 0.0),  # Gf.Quatd is w,i,j,k
            # )

            # annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
            # hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
            # annotator.attach(hydra_texture.path)

            # if debug:
            #     writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
            #     writer.attach(hydra_texture.path)

            # lidar_annotators.append(annotator)

            # Ouster OS-1 Lidar
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                path="/lidar_OS1",
                parent=f"/World/envs/env_{env_idx}/Go2/base",
                config="OS1_REV6_32ch10hz1024res",
                # config="Velodyne_VLS128",
                translation=(0.2, 0, 0.238195),
                orientation=Gf.Quatd(0.0, 0.0, 0.0, 1.0),  # Gf.Quatd is w,i,j,k
            )

            annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
            hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
            annotator.attach(hydra_texture.path)

            if debug:
                writer = rep.writers.get("RtxLidarDebugDrawPointCloudBuffer")
                writer.attach(hydra_texture.path)

            lidar_annotators.append(annotator)
        return lidar_annotators

    def add_camera(self):
        cameras = []
        for env_idx in range(self.num_envs):
            camera = Camera(
                prim_path=f"/World/envs/env_{env_idx}/Go2/base/front_cam",
                translation=np.array([0.4, 0.0, 0.2]),
                frequency=25,
                resolution=(640, 480),
                orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True),
            )
            camera.initialize()
            camera.set_focal_length(1.5)
            cameras.append(camera)
        return cameras
    
    def add_imu(self):
        imus = []
        for env_idx in range(self.num_envs):
            lidar_imu = IMUSensor(
                prim_path=f"/World/envs/env_{env_idx}/Go2/base/imu_OS1",
                translation=np.array([0.206253, -0.011775, 0.207645]),
                frequency=200,
                orientation=np.array([1.0, 0.0, 0.0, 0.0]),
            )
            lidar_imu.initialize()
            imus.append(lidar_imu)
        return imus