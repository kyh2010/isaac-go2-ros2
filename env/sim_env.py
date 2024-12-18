from omni.isaac.core.utils.prims import define_prim, get_prim_at_path
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.lab.terrains import TerrainImporterCfg, TerrainImporter
from omni.isaac.lab.terrains import TerrainGeneratorCfg
import omni.isaac.lab.terrains as terrain_gen
import omni.isaac.lab.sim as sim_utils
from env.terrain_cfg import HfUniformDiscreteObstaclesTerrainCfg

def create_obstacle_sparse_env():
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=100 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 

def create_obstacle_medium_env():
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=200 ,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain) 


def create_obstacle_dense_env():
    # Terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/obstacleTerrain",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            seed=0,
            size=(50, 50),
            color_scheme="height",
            sub_terrains={"t1": HfUniformDiscreteObstaclesTerrainCfg(
                seed=0,
                size=(50, 50),
                obstacle_width_range=(0.5, 1.0),
                obstacle_height_range=(1.0, 2.0),
                num_obstacles=400,
                obstacles_distance=2.0,
                border_width=5,
                avoid_positions=[[0, 0]]
            )},
        ),
        visual_material=None,     
    )
    TerrainImporter(terrain)


def create_rough_terrain_env():
    assets_root_path = get_assets_root_path()
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="generator",
        terrain_generator=TerrainGeneratorCfg(
            size=(8.0, 8.0),
            border_width=20.0,
            num_rows=10,
            num_cols=20,
            horizontal_scale=0.1,
            vertical_scale=0.005,
            slope_threshold=0.75,
            use_cache=False,
            sub_terrains={
                "pyramid_stairs": terrain_gen.MeshPyramidStairsTerrainCfg(
                    proportion=0.2,
                    step_height_range=(0.05, 0.23),
                    step_width=0.3,
                    platform_width=3.0,
                    border_width=1.0,
                    holes=False,
                ),
                "pyramid_stairs_inv": terrain_gen.MeshInvertedPyramidStairsTerrainCfg(
                    proportion=0.2,
                    step_height_range=(0.05, 0.23),
                    step_width=0.3,
                    platform_width=3.0,
                    border_width=1.0,
                    holes=False,
                ),
                "boxes": terrain_gen.MeshRandomGridTerrainCfg(
                    proportion=0.2, grid_width=0.45, grid_height_range=(0.05, 0.2), platform_width=2.0
                ),
                "random_rough": terrain_gen.HfRandomUniformTerrainCfg(
                    proportion=0.2, noise_range=(0.02, 0.10), noise_step=0.02, border_width=0.25
                ),
                "hf_pyramid_slope": terrain_gen.HfPyramidSlopedTerrainCfg(
                    proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
                ),
                "hf_pyramid_slope_inv": terrain_gen.HfInvertedPyramidSlopedTerrainCfg(
                    proportion=0.1, slope_range=(0.0, 0.4), platform_width=2.0, border_width=0.25
                ),
            },
        ),
        max_init_terrain_level=5,
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="multiply",
            restitution_combine_mode="multiply",
            static_friction=1.0,
            dynamic_friction=1.0,
        ),
        visual_material=sim_utils.MdlFileCfg(
            mdl_path=assets_root_path+"/Isaac/IsaacLab/Materials/TilesMarbleSpiderWhiteBrickBondHoned/TilesMarbleSpiderWhiteBrickBondHoned.mdl",
            project_uvw=True,
            texture_scale=(0.25, 0.25),
        ),
        debug_vis=False,
    )
    TerrainImporter(terrain)

def create_warehouse_env():
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_forklifts_env():
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_shelves_env():
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    prim.GetReferences().AddReference(asset_path)

def create_full_warehouse_env():
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_hospital_env():
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Hospital")
    prim = define_prim("/World/Hospital", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Hospital/hospital.usd"
    prim.GetReferences().AddReference(asset_path)

def create_office_env():
    assets_root_path = get_assets_root_path()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)