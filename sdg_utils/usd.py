import os

from pxr import Gf, Sdf, UsdGeom, Usd, UsdShade
import isaacsim.core.api.objects as objects
from isaacsim.core.utils.stage import open_stage, add_reference_to_stage, get_current_stage
from isaacsim.core.api import World


def load_usd_file(usd_file_path: str) -> tuple[World, Usd.Stage]:
    """加载USD文件, 并返回World和Stage"""
    assert os.path.exists(usd_file_path), f"场景文件不存在: {usd_file_path}"
    open_stage(usd_file_path)
    # 获取stage
    stage = get_current_stage()
    # 添加地面平面（不可见）
    objects.GroundPlane("/World/ground_plane", visible=False)
    # 初始化物理世界
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    
    return world, stage

