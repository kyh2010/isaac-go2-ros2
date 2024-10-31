import omni
from pxr import Gf
import omni.replicator.core as rep
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")




def add_rtx_lidar(num_envs):
    for i in range(num_envs):
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/lidar",
            parent=f"/World/envs/env_{i}/Go2/base",
            config="Hesai_XT32_SD10",
            translation=(0.3, 0, 0.15),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
        )

        # RTX sensors are cameras and must be assigned to their own render product
        hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")

        # Create Point cloud publisher pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "ROS2PublishPointCloud")
        writer.initialize(topicName="point_cloud", frameId="map")
        writer.attach([hydra_texture])

        # Create the debug draw pipeline in the post process graph
        # writer = rep.writers.get("RtxLidar" + "DebugDrawPointCloud")
        # writer.attach([hydra_texture])

        # Create LaserScan publisher pipeline in the post process graph
        writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
        writer.initialize(topicName="scan", frameId="sim_lidar")
        writer.attach([hydra_texture])