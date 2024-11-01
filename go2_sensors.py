import omni
from pxr import Gf
import omni.replicator.core as rep
from omni.isaac.sensor import LidarRtx
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")


def add_rtx_lidar(num_envs):
    lidar_annotators = []
    for env_idx in range(num_envs):
        # _, sensor = omni.kit.commands.execute(
        #     "IsaacSensorCreateRtxLidar",
        #     path="/lidar",
        #     parent=f"/World/envs/env_{env_idx}/Go2/base",
        #     # config="Hesai_XT32_SD10",
        #     config="Velodyne_VLS128",
        #     translation=(0.3, 0, 0.5),
        #     orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
        #     visibility=True
        # )

        sensor = LidarRtx(f"/World/envs/env_{env_idx}/Go2/base/lidar_sensor",
                            rotation_frequency = 200,
                            pulse_time=1, 
                            translation=(0.0, 0, 0.4),
                            orientation=(1.0, 0.0, 0.0, 0.0),
                            config_file_name= "Hesai_XT32_SD10",
                            )

        annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
        # hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
        # annotator.attach(hydra_texture.path)
        annotator.attach(sensor.get_render_product_path())
        # annotator.attach(hydra_texture.path)
        lidar_annotators.append(annotator)
    

        # sensor = LidarRtx(f"/World/envs/env_{env_idx}/Go2/base/lidar_sensor",
        #                     rotation_frequency = 200,
        #                     pulse_time=1, 
        #                     translation=(0.0, 0, 0.4),
        #                     orientation=(1.0, 0.0, 0.0, 0.0),
        #                     config_file_name= "Hesai_XT32_SD10",
        #                     )

        # # RTX sensors are cameras and must be assigned to their own render product
        # hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
        # # hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

        # # Create Point cloud publisher pipeline in the post process graph
        # writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloudBuffer")
        # if (num_envs == 1):
        #     writer.initialize(topicName="point_cloud", frameId=f"unitree_go2/lidar_frame")
        #     writer.attach([hydra_texture])
        #     # writer.attach([sensor.get_render_product_path()])

        #     # Create the debug draw pipeline in the post process graph
        #     # writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
        #     writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloudBuffer")
        #     writer.attach([hydra_texture])
        #     # writer.attach([sensor.get_render_product_path()])

        #     # Create LaserScan publisher pipeline in the post process graph
        #     writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
        #     writer.initialize(topicName="scan", frameId=f"unitree_go2/lidar_frame")
        #     writer.attach([hydra_texture])
        #     # writer.attach([sensor.get_render_product_path()])        
        # else:
        #     writer.initialize(topicName="point_cloud", frameId=f"unitree_go2_{env_idx}/lidar_frame")
        #     # writer.attach([hydra_texture])

        #     # Create the debug draw pipeline in the post process graph
        #     writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
        #     # writer.attach([hydra_texture])

        #     # Create LaserScan publisher pipeline in the post process graph
        #     writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
        #     writer.initialize(topicName="scan", frameId=f"unitree_go2_{env_idx}/lidar_frame")
        #     # writer.attach([hydra_texture])
    return lidar_annotators