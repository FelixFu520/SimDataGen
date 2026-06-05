from typing import Dict, List, Optional, Tuple, TypedDict

import numpy as np
from loguru import logger
from PIL import Image, ImageDraw
from plyfile import PlyData, PlyElement

import omni.physx
import omni.usd
from pxr import UsdGeom, Usd, UsdPhysics, Gf, Sdf
from isaacsim.asset.gen.omap.bindings import _omap

from .misc import generate_high_contrast_colors


def get_mesh_paths(stage: Usd.Stage) -> List[str]:
    """
    获取物理碰撞体(Mesh)的USD路径列表, 仅返回包含物理碰撞体(Mesh)的USD路径
    注意: 如果Mesh没有物理碰撞体(Mesh), 则不会返回该USD路径

    Args:
        stage: Usd.Stage
    Returns:
        List[str]: 物理碰撞体(Mesh)的USD路径列表, 仅返回包含物理碰撞体(Mesh)的USD路径
    """
    mesh_paths = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            path = prim.GetPath()
            
            if UsdPhysics.CollisionAPI(prim) and UsdPhysics.MeshCollisionAPI(prim):
                mesh_paths.append(path)
            else:
                continue
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim)
    
    return mesh_paths


def build_mesh_id_map(mesh_paths: List[str]) -> Dict[str, int]:
    """构建 mesh path -> semantic_id 的映射,语义 ID 从 1 开始。

    设计意图:
    - 让 occupancy 体素的 semantic_id与 相机端语义图 使用同一份映射。
    - 0 保留给 BACKGROUND (天空 / 未命中区域 / occupancy 中的 free)。
    - 每个 mesh prim 一个独立 semantic_id;颜色 LUT 用固定 seed 生成,从而保证
      同一个 mesh(同一 Section)在所有帧、所有相机的语义可视化中颜色一致。

    Args:
        mesh_paths: 带物理碰撞的 mesh path 列表(顺序敏感,与遍历 stage 的顺序一致)。

    Returns:
        {str(mesh_path): semantic_id}, semantic_id 取值范围 1..len(mesh_paths)
    """
    mesh_path_to_id: Dict[str, int] = {}
    for i, path in enumerate(mesh_paths):
        mesh_path_to_id[str(path)] = i + 1
    return mesh_path_to_id


def _apply_semantics_via_attributes(prim: Usd.Prim, label: str) -> None:
    """通过直接写入 USD 属性的方式注入 Semantics schema(降级路径)。

    与 Isaac Sim 官方 `add_labels` / `add_update_semantics` 内部行为一致,
    用于不便依赖 Isaac Sim 高层 API 的场景(例如版本差异 / API 重命名)。
    """
    instance_name = "Semantics_class"
    sem_type_attr = f"semantic:{instance_name}:params:semanticType"
    sem_data_attr = f"semantic:{instance_name}:params:semanticData"

    # 在 prim 上声明 SemanticsAPI:Semantics_class 已被 apply
    # USD Python 层用 AddAppliedSchema 即可写入 apiSchemas 元数据,无需 schema 类
    try:
        prim.AddAppliedSchema(f"SemanticsAPI:{instance_name}")
    except Exception:
        # 兼容旧版 USD: AddAppliedSchema 不存在时,仅写属性也能被 Replicator 识别
        pass

    type_attr = prim.CreateAttribute(sem_type_attr, Sdf.ValueTypeNames.String, False)
    type_attr.Set("class")

    data_attr = prim.CreateAttribute(sem_data_attr, Sdf.ValueTypeNames.String, False)
    data_attr.Set(label)


def apply_semantics_to_meshes(stage: Usd.Stage, mesh_path_to_id: Dict[str, int]) -> None:
    """给 stage 中的每个 mesh prim 注入语义 label,供 `semantic_segmentation` annotator 使用。

    优先调用 Isaac Sim 官方接口(2.3.0+ 的 `add_labels`,或更早版本的
    `add_update_semantics`),失败时降级到直接写 USD 属性的兼容实现。
    label 内容统一为 `id_{semantic_id}`,相机端会按此前缀做反查,把 annotator
    内部 ID remap 成与 occupancy 一致的 `semantic_id`。

    Args:
        stage: Usd.Stage
        mesh_path_to_id: 由 `build_mesh_id_map` 生成的映射表
    """
    # 选择最合适的高层 API,优先级如下:
    #   1) isaacsim.core.utils.semantics.add_labels      ← Isaac Sim 5.x 推荐 (UsdSemantics.LabelsAPI)
    #   2) isaaclab.sim.utils.semantics.add_labels       ← Isaac Lab 2.3.0+
    #   3) omni.isaac.core.utils.semantics.add_update_semantics ← 旧版 (deprecated, 会打 carb warn)
    #   4) 直接写 USD 属性                                ← 兜底
    add_labels = None
    add_update_semantics = None
    try:
        from isaacsim.core.utils.semantics import add_labels as _add_labels  # type: ignore
        add_labels = _add_labels
    except Exception:
        try:
            from isaaclab.sim.utils.semantics import add_labels as _add_labels  # type: ignore
            add_labels = _add_labels
        except Exception:
            try:
                from omni.isaac.core.utils.semantics import add_update_semantics as _add_update  # type: ignore
                add_update_semantics = _add_update
            except Exception:
                pass

    applied = 0
    for path_str, sid in mesh_path_to_id.items():
        prim = stage.GetPrimAtPath(Sdf.Path(path_str))
        if not prim or not prim.IsValid():
            logger.warning(f"apply_semantics_to_meshes: prim 不存在, 跳过: {path_str}")
            continue

        label = f"id_{sid}"
        # 关键: 同时写入新版 UsdSemantics.LabelsAPI (Isaac Sim 5.x 推荐) 与旧版
        # SemanticsAPI:Semantics_class 属性。
        # 实测 (见 docs/transparent_and_far_filter.md / 2026-05 的 ModernHomeHouse
        # 场景) Replicator 的 semantic_segmentation annotator 在某些帧上只能识别
        # 旧版 SemanticsAPI 属性,只用 add_labels 时部分 mesh (例如电视屏 SM_Painting_22)
        # 在某些视角下会被标记为 BACKGROUND,导致同一 mesh 在不同帧颜色不一致。
        # 双写后两条路都能找到 label,annotator 输出更稳定。
        ok = False
        try:
            if add_labels is not None:
                add_labels(prim, labels=[label], instance_name="class")
                ok = True
            elif add_update_semantics is not None:
                add_update_semantics(prim, semantic_label=label, type_label="class")
                ok = True
        except Exception as e:
            logger.warning(f"apply_semantics_to_meshes: 高层 API 失败 {path_str}, 原因: {e}")

        try:
            # 无论高层 API 是否成功,都补一份 SemanticsAPI 属性 (兜底 + 兼容旧 Replicator)
            _apply_semantics_via_attributes(prim, label)
            ok = True
        except Exception as ee:
            logger.warning(f"apply_semantics_to_meshes: 属性写入失败 {path_str}, 原因: {ee}")

        if ok:
            applied += 1
        else:
            logger.warning(f"apply_semantics_to_meshes: 写入 semantics 彻底失败 {path_str}")

    logger.info(f"apply_semantics_to_meshes: 共 {applied}/{len(mesh_path_to_id)} 个 mesh 注入了 class label")


def get_semantic_occupancy(stage: Usd.Stage, resolution: float = 0.05,
                           mesh_paths: List[str] = None,
                           mesh_path_to_id: Dict[str, int] = None,
                           margin_times: int = 0) -> np.array:
    """
    获取带语义的Occupancy
    Args:
        stage: Usd.Stage
        resolution: 分辨率
        mesh_paths: 物理碰撞体(Mesh)的USD路径列表, 仅返回包含物理碰撞体(Mesh)的USD路径
        mesh_path_to_id: 可选的 mesh_path -> semantic_id 映射(由 build_mesh_id_map 提供)。
            若为 None,则按 mesh_paths 顺序内部生成。强烈建议传入,以保证 occupancy
            的 ID 与相机端语义图的 ID 完全一致。
        margin_times: 边缘裁剪/扩大倍数, 默认0, 即不裁剪边缘, 也不扩大
    Returns:
        np.array: 带语义的Occupancy
    """
    all_semantic_points = []
    
    physx = omni.physx.acquire_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    # 计算所有 mesh 的联合包围盒,用于生成 free positions
    all_w_min = None
    all_w_max = None

    if mesh_path_to_id is None:
        mesh_path_to_id = build_mesh_id_map(mesh_paths)

    for path in mesh_paths:
        prim = stage.GetPrimAtPath(path)
        # 复用统一的 semantic_id,确保与相机语义图、id_map.json 一致
        semantic_id = mesh_path_to_id.get(str(path))
        if semantic_id is None:
            logger.warning(f"get_semantic_occupancy: {path} 不在 mesh_path_to_id 中, 跳过")
            continue
        
        # 使用世界坐标 AABB 以确保完全覆盖旋转物体
        # 1. 获取世界坐标下的对齐包围盒 (AABB)
        # ComputeWorldBound 会自动计算应用了所有变换(旋转/缩放/位移)后的包围盒
        world_bbox = bbox_cache.ComputeWorldBound(prim)
        world_range = world_bbox.ComputeAlignedRange()
        
        w_min = world_range.GetMin()
        w_max = world_range.GetMax()
        w_center = world_range.GetMidpoint()
        
        # 2. 设置 Generator 参数
        # origin: 设置为包围盒中心,旋转保持为 0 (与世界坐标轴对齐)
        # 这样生成的体素网格是世界轴对齐的,能最稳定地包裹物体
        origin = (
            float(w_center[0]), float(w_center[1]), float(w_center[2]),
            0.0, 0.0, 0.0
        )
        
        # 3. 计算相对于 origin (中心点) 的局部边界
        # 增加 Margin 防止边缘裁剪
        margin = resolution * margin_times
        # lower 和 upper 是相对于 origin 的坐标
        lower = (
            float(w_min[0] - w_center[0] - margin), 
            float(w_min[1] - w_center[1] - margin), 
            float(w_min[2] - w_center[2] - margin)
        )
        upper = (
            float(w_max[0] - w_center[0] + margin), 
            float(w_max[1] - w_center[1] + margin), 
            float(w_max[2] - w_center[2] + margin)
        )

        # 4.调用生成器
        generator = _omap.Generator(physx, stage_id)
        # settings: voxel_size, occupied_thresh, free_thresh, unknown_thresh
        generator.update_settings(resolution, 1, 0, 255)
        # origin, lower, upper, 仅仅保留3位有效数字
        origin = (round(origin[0], 3), round(origin[1], 3), round(origin[2], 3), round(origin[3], 3), round(origin[4], 3), round(origin[5], 3))
        lower = (round(lower[0], 3), round(lower[1], 3), round(lower[2], 3))
        upper = (round(upper[0], 3), round(upper[1], 3), round(upper[2], 3))
        generator.set_transform(origin, lower, upper)
        
        generator.generate3d()
        pts_occupied = np.array(generator.get_occupied_positions()).astype(np.float32)
        pts_free = np.array(generator.get_free_positions()).astype(np.float32)
        
        if len(pts_occupied) > 10:
            # 拼接 ID: [x, y, z, semantic_id]
            ids = np.full((pts_occupied.shape[0], 1), semantic_id, dtype=np.float32)
            combined = np.hstack([pts_occupied, ids])
            all_semantic_points.append(combined)
            logger.info(f"成功标注: {path} -> ID: {semantic_id}, occupied点数: {len(pts_occupied)}, free点数: {len(pts_free)}")
        else:
            logger.warning(f"警告: {path} 标注点数为 {len(pts_occupied)}, 跳过该 Mesh, 请检查该 Mesh 是否在物理层可见")
            continue

        # 5.更新联合包围盒
        if all_w_min is None:
            all_w_min = w_min
            all_w_max = w_max
        else:
            all_w_min = Gf.Vec3d(
                min(all_w_min[0], w_min[0]),
                min(all_w_min[1], w_min[1]),
                min(all_w_min[2], w_min[2])
            )
            all_w_max = Gf.Vec3d(
                max(all_w_max[0], w_max[0]),
                max(all_w_max[1], w_max[1]),
                max(all_w_max[2], w_max[2])
            )
        
    # 为整个场景生成 free positions(使用联合包围盒)
    logger.info(f"开始生成场景 free positions, 联合包围盒: {all_w_min} ~ {all_w_max}")
    if all_w_min is not None and len(mesh_paths) > 0:
        all_w_center = (all_w_min + all_w_max) / 2.0
        margin = resolution * margin_times  # 为 free space 增加更大的 margin
        
        origin_all = (
            float(all_w_center[0]), float(all_w_center[1]), float(all_w_center[2]),
            0.0, 0.0, 0.0
        )
        lower_all = (
            float(all_w_min[0] - all_w_center[0] - margin),
            float(all_w_min[1] - all_w_center[1] - margin),
            float(all_w_min[2] - all_w_center[2] - margin)
        )
        upper_all = (
            float(all_w_max[0] - all_w_center[0] + margin),
            float(all_w_max[1] - all_w_center[1] + margin),
            float(all_w_max[2] - all_w_center[2] + margin)
        )
        
        generator_all = _omap.Generator(physx, stage_id)
        generator_all.update_settings(resolution, 1, 0, 255)
        origin_all = (round(origin_all[0], 3), round(origin_all[1], 3), round(origin_all[2], 3), 
                     round(origin_all[3], 3), round(origin_all[4], 3), round(origin_all[5], 3))
        lower_all = (round(lower_all[0], 3), round(lower_all[1], 3), round(lower_all[2], 3))
        upper_all = (round(upper_all[0], 3), round(upper_all[1], 3), round(upper_all[2], 3))
        generator_all.set_transform(origin_all, lower_all, upper_all)
        
        generator_all.generate3d()
        pts_free_all = np.array(generator_all.get_free_positions()).astype(np.float32)
        
        if len(pts_free_all) > 0:
            # free positions 使用语义 ID 0
            ids_free = np.full((pts_free_all.shape[0], 1), 0, dtype=np.float32)
            combined_free = np.hstack([pts_free_all, ids_free])
            all_semantic_points.append(combined_free)
            logger.info(f"场景 free positions: {len(pts_free_all)} 个点, 语义 ID: 0")

    return np.vstack(all_semantic_points) if all_semantic_points else np.array([])


class OccSlice2DMeta(TypedDict):
    """z 横截面 2D occupancy 元数据。image 行 0 为图像顶部（世界 +Y 侧）。"""

    image: np.ndarray
    origin: Tuple[float, float]
    resolution: float
    slice_z: float
    width: int
    height: int


def build_occ_slice_2d(
    occupied_xyz: np.ndarray,
    free_xyz: np.ndarray,
    slice_z: float,
    resolution: float = 0.1,
    padding: int = 2,
) -> OccSlice2DMeta:
    """从 3D occupancy 点云提取 z=slice_z 横截面 2D 栅格。

    占据体素为 0（黑），可通行体素为 255（白）。同一栅格既有占据又有空闲时，占据优先。
    """
    occ3 = occupied_xyz[:, :3] if len(occupied_xyz) else np.zeros((0, 3))
    free3 = free_xyz[:, :3] if len(free_xyz) else np.zeros((0, 3))
    half = resolution * 0.5

    occ_mask = np.abs(occ3[:, 2] - slice_z) <= half if len(occ3) else np.zeros(0, dtype=bool)
    free_mask = np.abs(free3[:, 2] - slice_z) <= half if len(free3) else np.zeros(0, dtype=bool)
    occ_slice = occ3[occ_mask]
    free_slice = free3[free_mask]

    if len(occ_slice) == 0 and len(free_slice) == 0:
        occ_mask = np.abs(occ3[:, 2] - slice_z) <= resolution if len(occ3) else np.zeros(0, dtype=bool)
        free_mask = np.abs(free3[:, 2] - slice_z) <= resolution if len(free3) else np.zeros(0, dtype=bool)
        occ_slice = occ3[occ_mask]
        free_slice = free3[free_mask]

    if len(occ_slice) + len(free_slice) == 0:
        raise ValueError(
            f"z={slice_z} 横截面无 occupancy 点，请检查 slice_z 或 --occupancy-resolution"
        )

    all_xy = np.vstack([occ_slice[:, :2], free_slice[:, :2]])
    xy_min = all_xy.min(axis=0)
    xy_max = all_xy.max(axis=0)

    nx = int(np.ceil((xy_max[0] - xy_min[0]) / resolution)) + 1 + 2 * padding
    ny = int(np.ceil((xy_max[1] - xy_min[1]) / resolution)) + 1 + 2 * padding
    origin_x = float(xy_min[0] - padding * resolution)
    origin_y = float(xy_min[1] - padding * resolution)

    grid = np.full((ny, nx), 255, dtype=np.uint8)

    def _world_to_grid(xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ix = np.round((xy[:, 0] - origin_x) / resolution).astype(np.int64)
        iy_world = np.round((xy[:, 1] - origin_y) / resolution).astype(np.int64)
        iy = (ny - 1) - iy_world
        return ix, iy

    if len(free_slice):
        fx, fy = _world_to_grid(free_slice[:, :2])
        valid = (fx >= 0) & (fx < nx) & (fy >= 0) & (fy < ny)
        grid[fy[valid], fx[valid]] = 255

    if len(occ_slice):
        ox, oy = _world_to_grid(occ_slice[:, :2])
        valid = (ox >= 0) & (ox < nx) & (oy >= 0) & (oy < ny)
        grid[oy[valid], ox[valid]] = 0

    return OccSlice2DMeta(
        image=grid,
        origin=(origin_x, origin_y),
        resolution=float(resolution),
        slice_z=float(slice_z),
        width=int(nx),
        height=int(ny),
    )


def world_xy_to_occ_pixel(x: float, y: float, meta: OccSlice2DMeta) -> Tuple[int, int]:
    """世界 XY 转图像像素坐标（行 0 为顶部）。"""
    origin_x, origin_y = meta["origin"]
    res = meta["resolution"]
    width_m = res * meta["width"]
    height_m = res * meta["height"]
    u = (x - origin_x) / width_m
    v = 1.0 - (y - origin_y) / height_m
    px = int(round(u * meta["width"]))
    py = int(round(v * meta["height"]))
    return px, py


def render_occ_map_with_rig(
    meta: OccSlice2DMeta,
    pose: List[float],
    recorded: Optional[List[List[float]]] = None,
) -> np.ndarray:
    """在 occupancy 灰度图上绘制 rig 位置、朝向与已录制轨迹，返回 RGBA uint8。"""
    pil = Image.fromarray(meta["image"]).convert("RGB")
    draw = ImageDraw.Draw(pil)

    if recorded:
        for p in recorded:
            rpx, rpy = world_xy_to_occ_pixel(float(p[0]), float(p[1]), meta)
            draw.ellipse((rpx - 2, rpy - 2, rpx + 2, rpy + 2), fill=(80, 200, 80))

    px, py = world_xy_to_occ_pixel(float(pose[0]), float(pose[1]), meta)
    yaw_rad = np.deg2rad(float(pose[5]))
    arrow_len = 14
    ex = px + arrow_len * np.cos(yaw_rad)
    ey = py - arrow_len * np.sin(yaw_rad)
    draw.line((px, py, ex, ey), fill=(255, 60, 60), width=2)
    draw.ellipse((px - 4, py - 4, px + 4, py + 4), fill=(255, 40, 40), outline=(255, 200, 200))

    return np.asarray(pil.convert("RGBA"), dtype=np.uint8)


def save_occ_slice_2d_png(meta: OccSlice2DMeta, png_path: str) -> None:
    Image.fromarray(meta["image"]).save(png_path)


def save_semantic_occupancy_ply(semantic_occupancy: np.array, ply_path: str):
    """
    保存语义Occupancy为PLY文件
    Args:
        semantic_occupancy: 语义Occupancy
        path: PLY文件路径
    """
    vertices = []
    if int(np.max(semantic_occupancy[:, 3])) == 0:
        color_list = [(100, 150, 255)]  # free positions (ID=0) 使用浅蓝色,occupied positions 使用随机颜色
    else:
        color_list = generate_high_contrast_colors(int(np.max(semantic_occupancy[:, 3])))
    for p in semantic_occupancy:
        semantic_id = int(p[3])
        c = color_list[semantic_id % len(color_list)]
        vertices.append((p[0], p[1], p[2], c[0], c[1], c[2]))
        
    vertex_element = np.array(vertices, dtype=[
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')
    ])
    PlyData([PlyElement.describe(vertex_element, 'vertex')], text=True).write(ply_path)


def save_ply(filename, points, colors):
    """保存点云为PLY格式"""
    with open(filename, 'w') as f:
        # PLY文件头
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        
        # 写入点云数据
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i]
            f.write(f"{x} {y} {z} {int(r)} {int(g)} {int(b)}\n")