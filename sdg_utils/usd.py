import os

from pxr import Gf, Sdf, UsdGeom, Usd, UsdShade
import isaacsim.core.api.objects as objects
from isaacsim.core.utils.stage import open_stage, add_reference_to_stage, get_current_stage
from isaacsim.core.api import World


def load_usd_file(usd_file_path: str) -> tuple[World, Usd.Stage]:
    """加载USD文件, 并返回World和Stage

    注意: 这里**故意不再**创建 GroundPlane。
    Isaac Sim 的 GroundPlane 是一个 10000x10000、带无限碰撞的不可见地面,
    若在加载阶段就注入,会污染随后的 occupancy 体素扫描(参见 docs/fix_bug.md):
    occupied/free 都基于整个 PhysX 物理场景生成, z≈0 那层会被它判成 occupied,
    并把联合包围盒撑到 ±5000, 在大 Z 跨度的户外地形(如 AsianVillage)上造成
    free positions 严重错位。

    但 GroundPlane 在 PathTracing/RTX 渲染下又承担了重要的"地面反射兜底"作用:
    它虽然 visible=False、不直接出现在画面里,却参与光线弹射,为场景补一层来自
    下方的间接光。缺少它时室内场景(如 home_000)整体会发黑。

    为了同时满足两类场景, GroundPlane 的创建被拆分到独立的 add_ground_plane(),
    由调用方在 occupancy 计算完成之后、相机渲染开始之前再调用(见 gen_data.py)。
    """
    assert os.path.exists(usd_file_path), f"场景文件不存在: {usd_file_path}"
    open_stage(usd_file_path)
    # 获取stage
    stage = get_current_stage()
    # 初始化物理世界
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    
    return world, stage


def add_ground_plane(prim_path: str = "/World/ground_plane") -> None:
    """创建一个不可见的 GroundPlane, 用于渲染阶段的地面反射/间接光兜底。

    **必须在 occupancy 计算完成之后、相机渲染开始之前调用**, 以免无限碰撞地面
    污染 occupancy 体素扫描(详见 load_usd_file 的说明与 docs/fix_bug.md)。

    Args:
        prim_path: GroundPlane 的 USD prim 路径。
    """
    objects.GroundPlane(prim_path, visible=False)

