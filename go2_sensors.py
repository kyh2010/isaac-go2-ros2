import omni
from pxr import Gf
import omni.replicator.core as rep

def add_rtx_lidar(num_envs):
    lidar_annotators = []
    for env_idx in range(num_envs):
        _, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/lidar",
            parent=f"/World/envs/env_{env_idx}/Go2/base",
            config="Hesai_XT32_SD10",
            # config="Velodyne_VLS128",
            translation=(0.23, 0, 0.2),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
        )

        annotator = rep.AnnotatorRegistry.get_annotator("RtxSensorCpuIsaacCreateRTXLidarScanBuffer")
        hydra_texture = rep.create.render_product(sensor.GetPath(), [1, 1], name="Isaac")
        annotator.attach(hydra_texture.path)
        lidar_annotators.append(annotator)
    return lidar_annotators