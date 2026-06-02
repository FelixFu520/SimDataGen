"""
3D 轨迹生成(室内室外通用):
流程:
  1. 输入 free_xyz / occupied_xyz 体素点
  2. 3D 腐蚀 free_xyz(向内收缩 M 层),让 free 点远离自身连通区域的边界
  3. 3D 膨胀 occupied_xyz 得到"障碍禁区",再把落在禁区内的 free 点剔除,
     让 free 点离所有障碍物至少 N 个体素(防贴墙/穿模)
  4. 对 occupied_xyz 做"闭运算 + 3D 孔洞填充"得到整体外接 shape,
     只保留落在该外接 shape 内部的 free 点(房间中心等"远离墙但仍在场景
     主体内部"的点会被正确保留, 天空/远离建筑的旷野点会被剔除)
     —— 仅当 obstacle_envelope_iterations > 0 时启用
  5. 在过滤后的 free 点中随机选起点,xy 方向保持 max_angle_deviation 平滑、
     z 方向用 max_dz_per_step 限制,按 step_size 做 3D 随机游走生成轨迹
  6. 将每条轨迹保存为 PLY(点云 + 线段)
"""

import os
import random
import numpy as np
from loguru import logger
from typing import List, Optional, Tuple

from plyfile import PlyData, PlyElement
from scipy.ndimage import binary_erosion, binary_dilation, binary_fill_holes


# ============================================================
# 栅格化工具:xyz 点 <-> 3D 体素网格
# ============================================================
class VoxelGrid3D:
    """将 xyz 世界坐标与 3D 体素索引互相转换的辅助类。"""

    def __init__(self, xyz_min: np.ndarray, resolution: float, shape: Tuple[int, int, int], padding: int = 0):
        self.xyz_min = xyz_min.astype(np.float64)
        self.resolution = float(resolution)
        self.shape = shape  # (nx, ny, nz)
        self.padding = padding

    @classmethod
    def from_points(cls, xyz: np.ndarray, resolution: float, padding: int = 2) -> "VoxelGrid3D":
        xyz_min = xyz.min(axis=0)
        xyz_max = xyz.max(axis=0)
        nx = int(np.round((xyz_max[0] - xyz_min[0]) / resolution)) + 1 + 2 * padding
        ny = int(np.round((xyz_max[1] - xyz_min[1]) / resolution)) + 1 + 2 * padding
        nz = int(np.round((xyz_max[2] - xyz_min[2]) / resolution)) + 1 + 2 * padding
        return cls(xyz_min, resolution, (nx, ny, nz), padding)

    def world_to_index(self, xyz: np.ndarray) -> np.ndarray:
        """返回 (N, 3) int 索引 (ix, iy, iz)。"""
        idx = np.round((xyz - self.xyz_min) / self.resolution).astype(np.int64) + self.padding
        return idx

    def clip_index(self, idx: np.ndarray) -> np.ndarray:
        nx, ny, nz = self.shape
        idx = idx.copy()
        idx[:, 0] = np.clip(idx[:, 0], 0, nx - 1)
        idx[:, 1] = np.clip(idx[:, 1], 0, ny - 1)
        idx[:, 2] = np.clip(idx[:, 2], 0, nz - 1)
        return idx


# ============================================================
# 3D 腐蚀(对 free 体素在 xyz 三向均匀收缩)
# ============================================================
def erode_free_positions_3d(
    free_xyz: np.ndarray,
    occupied_xyz: Optional[np.ndarray],
    resolution: float,
    erode_iterations: int,
) -> np.ndarray:
    """
    将 free_xyz 栅格化为 3D 体素网格,对 free 体素做 3D 形态学腐蚀
    (3x3x3 结构元素迭代 erode_iterations 次),然后仅保留腐蚀后仍然存在的点。

    Args:
        free_xyz:         (N, 3) 或 (N, 4) free 点坐标,若带语义列只使用前3维
        occupied_xyz:     可选 (M, 3/4),仅用来扩展体素网格范围,让腐蚀边界更稳定
        resolution:       与 occupancy 分辨率一致
        erode_iterations: 迭代次数,等于各方向收缩的体素层数
    Returns:
        (K, 3) 过滤后的点坐标
    """
    if erode_iterations <= 0 or len(free_xyz) == 0:
        return free_xyz[:, :3] if free_xyz.shape[1] > 3 else free_xyz

    free_xyz3 = free_xyz[:, :3]
    if occupied_xyz is not None and len(occupied_xyz) > 0:
        all_xyz = np.vstack([free_xyz3, occupied_xyz[:, :3]])
    else:
        all_xyz = free_xyz3

    grid = VoxelGrid3D.from_points(all_xyz, resolution, padding=erode_iterations + 2)
    nx, ny, nz = grid.shape

    volume = np.zeros((nx, ny, nz), dtype=bool)
    idx = grid.clip_index(grid.world_to_index(free_xyz3))
    volume[idx[:, 0], idx[:, 1], idx[:, 2]] = True

    # scipy 3D 腐蚀,structure=None 默认 3x3x3 十字形(6 邻域)；
    # 这里手工给一个 3x3x3 全 1 结构元素(26 邻域),收缩更均匀
    struct = np.ones((3, 3, 3), dtype=bool)
    eroded = binary_erosion(volume, structure=struct, iterations=erode_iterations)

    keep_mask = eroded[idx[:, 0], idx[:, 1], idx[:, 2]]
    filtered = free_xyz3[keep_mask]
    logger.info(
        f"3D 腐蚀过滤: {len(free_xyz3)} -> {len(filtered)} 点 "
        f"(移除 {len(free_xyz3) - len(filtered)} 个边缘点, iterations={erode_iterations})"
    )
    return filtered


# ============================================================
# 3D 膨胀障碍物并剔除贴近障碍的 free 点(室内外通用)
# ============================================================
def filter_free_by_obstacle_dilation(
    free_xyz: np.ndarray,
    occupied_xyz: np.ndarray,
    resolution: float,
    obstacle_dilate_iterations: int = 2,
) -> np.ndarray:
    """
    把 occupied_xyz 做 3D 形态学膨胀得到"障碍禁区"体素,再剔除所有落在禁区
    内的 free 点。几何含义: 让 free 点离任意 occupied 物体至少
    `obstacle_dilate_iterations` 个体素 (等价于至少 N * resolution 米)。

    与旧版 flood-fill "屋内判定" 的区别:
      - 不再区分"室内/室外", 所以室内场景 (有天花板) 和室外场景
        (山/树/建筑物零散分布, 没有封闭外壳) 可以统一处理;
      - 不会因为某个 free 点"和 bbox 外部连通"就剔除它, 因此室外
        开阔区域的 free 点会被正常保留;
      - 仅依赖"远离障碍"这一条件, 语义清晰可控。

    Args:
        free_xyz:                   (N, 3) 或 (N, 4)
        occupied_xyz:               (M, 3) 或 (M, 4), 作为障碍
        resolution:                 与 occupancy 分辨率一致
        obstacle_dilate_iterations: 障碍膨胀层数, 0 表示不膨胀 (仅剔除
                                    与 occupied 完全重合的 free, 通常没有)
    Returns:
        (K, 3) 过滤后的 free 点
    """
    if len(free_xyz) == 0:
        return free_xyz[:, :3] if free_xyz.shape[1] > 3 else free_xyz

    free_xyz3 = free_xyz[:, :3]

    if len(occupied_xyz) == 0 or obstacle_dilate_iterations < 0:
        return free_xyz3

    occupied_xyz3 = occupied_xyz[:, :3]
    all_xyz = np.vstack([free_xyz3, occupied_xyz3])
    padding = max(2, obstacle_dilate_iterations + 1)
    grid = VoxelGrid3D.from_points(all_xyz, resolution, padding=padding)
    nx, ny, nz = grid.shape

    forbidden = np.zeros((nx, ny, nz), dtype=bool)
    occ_idx = grid.clip_index(grid.world_to_index(occupied_xyz3))
    forbidden[occ_idx[:, 0], occ_idx[:, 1], occ_idx[:, 2]] = True

    if obstacle_dilate_iterations > 0:
        struct_full = np.ones((3, 3, 3), dtype=bool)
        forbidden = binary_dilation(
            forbidden, structure=struct_full, iterations=obstacle_dilate_iterations
        )

    free_idx = grid.clip_index(grid.world_to_index(free_xyz3))
    in_forbidden = forbidden[free_idx[:, 0], free_idx[:, 1], free_idx[:, 2]]
    filtered = free_xyz3[~in_forbidden]
    logger.info(
        f"3D 障碍禁区过滤: {len(free_xyz3)} -> {len(filtered)} 点 "
        f"(移除 {len(free_xyz3) - len(filtered)} 个贴近障碍的点, "
        f"obstacle_dilate={obstacle_dilate_iterations})"
    )
    return filtered


# ============================================================
# 取 occupied_xyz 整体外接 shape, 仅保留落在外接 shape 内部的 free 点
# ============================================================
def filter_free_by_obstacle_envelope(
    free_xyz: np.ndarray,
    occupied_xyz: np.ndarray,
    resolution: float,
    obstacle_envelope_iterations: int,
) -> np.ndarray:
    """
    把 occupied_xyz 栅格化后, 通过 "闭运算 + 3D 孔洞填充" 得到整个障碍集合
    的**外接 shape**(贴合墙/地板/天花板的整体外壳, 内部空腔视为 shape 内部),
    只保留落在该外接 shape 内的 free 点。

    与"逐体素膨胀"的关键区别:
      - 不是让 free 点"离某个障碍最多 N 体素";
      - 而是把整个障碍集合封闭起来形成一个**整体外壳**, 外壳内部(包括房间
        中心、走廊中心这种离墙壁很远的位置)全部保留, 外壳之外(天空/远离
        建筑的旷野)才被剔除。

    算法步骤:
      1) 将 occupied_xyz 写入 3D 体素网格得到 `occ_mask`;
      2) 对 `occ_mask` 做 `obstacle_envelope_iterations` 次膨胀, 把墙缝/
         窗洞/小开口连成封闭边界 -> `closed`;
      3) `binary_fill_holes(closed)` 填充所有被 `closed` 完全包围的内部空腔
         (房间/走廊), 得到整体外接 shape `envelope`;
         对没有完全封闭的场景(例如户外缺顶), 填充不会越过开口, 所以外壳
         依然紧贴障碍分布, 不会误扩到无关的天空区域;
      4) 再做 `obstacle_envelope_iterations` 次腐蚀, 抵消第 2 步的人为扩张,
         让外壳尺寸贴近原始障碍边界;
      5) 保留落在 `envelope` 内的 free 点。

    参数语义:
      `obstacle_envelope_iterations` 控制 "闭运算" 的膨胀层数:
        - 太小 (0 或 1): 墙缝/窗户会让孔洞填充"漏气", 房间内部无法被封闭;
        - 适中 (2~5 一般足够): 可将小开口封上, 房间/走廊作为内部空腔被填充;
        - 设为 0 时关闭此过滤(直接返回 free_xyz), 与旧版行为一致.

    Args:
        free_xyz:                     (N, 3) 或 (N, 4)
        occupied_xyz:                 (M, 3) 或 (M, 4), 用于构建外接 shape
        resolution:                   与 occupancy 分辨率一致
        obstacle_envelope_iterations: 闭运算膨胀层数, <=0 关闭此过滤
    Returns:
        (K, 3) 过滤后的 free 点
    """
    if len(free_xyz) == 0:
        return free_xyz[:, :3] if free_xyz.shape[1] > 3 else free_xyz

    free_xyz3 = free_xyz[:, :3]

    if obstacle_envelope_iterations <= 0 or len(occupied_xyz) == 0:
        return free_xyz3

    occupied_xyz3 = occupied_xyz[:, :3]
    all_xyz = np.vstack([free_xyz3, occupied_xyz3])
    # padding 要大于膨胀次数, 保证膨胀不会撞到网格边界; 同时 fill_holes 从
    # 网格外围向内灌"背景", 足够的 padding 才能让背景真正包住外壳
    padding = max(3, obstacle_envelope_iterations + 2)
    grid = VoxelGrid3D.from_points(all_xyz, resolution, padding=padding)
    nx, ny, nz = grid.shape

    occ_mask = np.zeros((nx, ny, nz), dtype=bool)
    occ_idx = grid.clip_index(grid.world_to_index(occupied_xyz3))
    occ_mask[occ_idx[:, 0], occ_idx[:, 1], occ_idx[:, 2]] = True

    struct_full = np.ones((3, 3, 3), dtype=bool)

    # 1) 闭运算膨胀: 把墙缝 / 小开口连起来, 构成封闭边界
    closed = binary_dilation(
        occ_mask, structure=struct_full, iterations=obstacle_envelope_iterations
    )

    # 2) 3D 孔洞填充: 把被边界完全包围的内部空腔 (房间/走廊) 纳入 shape
    envelope = binary_fill_holes(closed, structure=struct_full)

    # 3) 腐蚀回去: 抵消第 1 步的人为扩张, 让 shape 更贴合原始障碍
    envelope = binary_erosion(
        envelope, structure=struct_full, iterations=obstacle_envelope_iterations
    )
    # 腐蚀不能把原始障碍本身"吃掉", 所以把 occ_mask 并回来, 保证障碍体素
    # 始终在 shape 内 (避免贴墙的 free 点被误删)
    envelope |= occ_mask

    free_idx = grid.clip_index(grid.world_to_index(free_xyz3))
    in_envelope = envelope[free_idx[:, 0], free_idx[:, 1], free_idx[:, 2]]
    filtered = free_xyz3[in_envelope]
    logger.info(
        f"3D 障碍外接 shape 过滤: {len(free_xyz3)} -> {len(filtered)} 点 "
        f"(移除 {len(free_xyz3) - len(filtered)} 个落在外接 shape 外的点, "
        f"envelope_iterations={obstacle_envelope_iterations})"
    )

    # 安全回退: 场景若未完全封闭 (例如缺天花板/缺某面墙), binary_fill_holes
    # 会从 padding 背景"灌气"进入内部, envelope 最终近似等于原始 occ_mask,
    # 导致所有 free 点都被剔除。此时直接返回过滤前的结果, 避免下游空点集。
    if len(filtered) == 0:
        logger.warning(
            "3D 障碍外接 shape 过滤后点数为 0, 判断场景未被完全封闭(可能缺天花板"
            "/外墙, 或 envelope_iterations 过小无法封住开口)。已自动跳过此步过滤, "
            "使用上一步结果继续; 如需启用该过滤, 可增大 --obstacle_envelope_iterations"
        )
        return free_xyz3

    return filtered


# ============================================================
# 3D 随机游走生成轨迹
# ============================================================
def _pick_next_point_in_cone(
    current: np.ndarray,
    positions_xyz: np.ndarray,
    desired_angle: float,
    step_size_xy: float,
    dz_limit: float,
    cone_half_angle_rad: float,
    min_progress: float,
) -> Optional[int]:
    """在 current 的"前方锥形"区域里选下一个候选点。
    约束:
      - xy 距离 ∈ [min_progress, step_size_xy * 1.5]
      - z 距离 ≤ dz_limit
      - 相对于 desired_angle 的方位角偏差 ≤ cone_half_angle_rad
    返回落入锥内最靠近理想目标位置的点 idx, 没有则返回 None。
    """
    diff = positions_xyz - current
    dxy = np.sqrt(diff[:, 0] ** 2 + diff[:, 1] ** 2)
    dz = np.abs(diff[:, 2])

    in_range = (dxy >= min_progress) & (dxy <= step_size_xy * 1.5) & (dz <= dz_limit)
    if not np.any(in_range):
        return None

    # 方位角过滤
    cand_idx = np.where(in_range)[0]
    cand_diff = diff[cand_idx]
    cand_ang = np.arctan2(cand_diff[:, 1], cand_diff[:, 0])
    ang_err = np.abs(((cand_ang - desired_angle + np.pi) % (2 * np.pi)) - np.pi)
    cone_mask = ang_err <= cone_half_angle_rad
    if not np.any(cone_mask):
        return None

    cand_idx = cand_idx[cone_mask]
    # 在锥内, 偏好 xy 距离接近 step_size_xy 的点(进展大) + 方位角偏差小
    d_err = np.abs(dxy[cand_idx] - step_size_xy)
    score = d_err + step_size_xy * 0.5 * (ang_err[cone_mask] / max(cone_half_angle_rad, 1e-6))
    return int(cand_idx[int(np.argmin(score))])


def _pick_most_open_point(
    current: np.ndarray,
    positions_xyz: np.ndarray,
    radius: float,
    max_radius: float,
) -> int:
    """救援策略: 在 [radius, max_radius] 范围内, 挑选一个"周围 free 点数最多"的点,
    让后续能从这个开阔点继续游走。实现为在 radius-max_radius 环带中随机采样若干候选,
    选 density 最高者。"""
    diff = positions_xyz - current
    dist = np.sqrt(np.sum(diff ** 2, axis=1))
    band_mask = (dist >= radius) & (dist <= max_radius)
    if not np.any(band_mask):
        # 退而求其次: 在 max_radius 内任意一点
        band_mask = dist <= max_radius
        if not np.any(band_mask):
            return int(np.argmin(dist))

    cand = np.where(band_mask)[0]
    sample_size = min(64, len(cand))
    sampled = np.random.choice(cand, sample_size, replace=False) if len(cand) > sample_size else cand

    # density = 每个候选在小邻域内的 free 点数
    best_idx = int(sampled[0])
    best_density = -1
    nb_r = max(radius * 1.2, 0.3)
    for i in sampled:
        p = positions_xyz[i]
        d = np.sqrt(np.sum((positions_xyz - p) ** 2, axis=1))
        density = int(np.sum(d < nb_r))
        if density > best_density:
            best_density = density
            best_idx = int(i)
    return best_idx


def generate_random_path_3d(
    positions_xyz: np.ndarray,
    num_points: int,
    step_size_xy: float,
    step_size_z: float,
    max_angle_deviation: float,
    max_dz_per_step: float,
    stuck_window: int = 4,
    stuck_threshold_ratio: float = 0.5,
    rescue_max_factor: float = 8.0,
) -> np.ndarray:
    """
    在 3D free 点云里做随机游走, 带"反角落蜷缩"保护机制:
        - 起点: 从点集中随机选一个 (优先选周围开阔的点)
        - 每一步:
            1. 当前 xy 朝向 + [-max_angle_deviation, +max_angle_deviation] 随机扰动 → 期望朝向
            2. 在前方锥内(± max_angle_deviation + 一些余量)选一个满足 xy 进展 >= min_progress 的点
            3. 若锥内没有候选 -> 逐步放宽(加大锥角、降低 min_progress)
            4. 若连续 stuck_window 步累计 xy 位移 < stuck_threshold_ratio * step_xy * window
               -> 判定为"卡住", 随机大角度转向 (180°±90°) + 下一步锥角加大
            5. 若多轮放宽后仍失败, 或卡住超过 2 个 window, 触发救援:
               从 [2*step, rescue_max_factor*step] 环带里挑一个"周围最开阔"的点作为下一个点

    Args:
        positions_xyz:          (N, 3) 过滤后的可行 free 点
        num_points:             每条路径点数
        step_size_xy:           xy 每步目标步长
        step_size_z:            z 方向最大步长
        max_angle_deviation:    xy 朝向最大偏差(度), 也作为基础锥角
        max_dz_per_step:        每步 z 方向最大变化
        stuck_window:           用于卡住检测的滑动窗口长度
        stuck_threshold_ratio:  窗口内平均 xy 进展 < 该比例 * step 认为卡住
        rescue_max_factor:      救援环带最大半径 = step_size_xy * rescue_max_factor
    Returns:
        path_xyz: (num_points, 3)
    """
    if len(positions_xyz) == 0:
        raise ValueError("positions_xyz 为空, 无法生成路径")

    dz_limit = min(max_dz_per_step, step_size_z)
    base_cone = np.deg2rad(max_angle_deviation)
    min_progress_base = max(0.5 * step_size_xy, step_size_xy - 1e-6 * step_size_xy)
    # 放宽策略时, cone 和 min_progress 的逐步放宽系数
    cone_widen = [1.0, 2.0, 4.0, np.pi / max(base_cone, 1e-3)]  # 最后一档 = π (360° 任意)
    progress_shrink = [1.0, 0.6, 0.3, 0.15]

    # --- 起点: 优先选一个周围开阔的点, 而不是完全纯随机 ---
    rand_center = positions_xyz[random.randint(0, len(positions_xyz) - 1)]
    start_idx = _pick_most_open_point(
        rand_center, positions_xyz,
        radius=0.0, max_radius=step_size_xy * rescue_max_factor,
    )
    current = positions_xyz[start_idx].astype(np.float64)
    path = [current.copy()]

    current_angle = random.uniform(0, 2 * np.pi)
    recent_xy_moves: List[float] = []  # 最近 stuck_window 步的 xy 位移
    stuck_count = 0  # 连续 stuck window 计数

    for _ in range(num_points - 1):
        # 期望朝向 = 当前朝向 + 随机偏差
        angle_deviation = random.uniform(-base_cone, base_cone)
        desired_angle = (current_angle + angle_deviation) % (2 * np.pi)

        # 逐级放宽锥角和最小进展, 直到找到候选点
        nearest_idx = None
        for cw, ps in zip(cone_widen, progress_shrink):
            cone = min(np.pi, base_cone * cw)
            min_prog = min_progress_base * ps
            nearest_idx = _pick_next_point_in_cone(
                current, positions_xyz, desired_angle,
                step_size_xy=step_size_xy,
                dz_limit=dz_limit,
                cone_half_angle_rad=cone,
                min_progress=min_prog,
            )
            if nearest_idx is not None:
                break

        # --- 卡住检测 ---
        def window_stuck() -> bool:
            if len(recent_xy_moves) < stuck_window:
                return False
            avg_prog = float(np.mean(recent_xy_moves[-stuck_window:]))
            return avg_prog < stuck_threshold_ratio * step_size_xy

        need_rescue = (nearest_idx is None) or window_stuck()

        if need_rescue:
            stuck_count += 1
            # 先尝试"大角度转向": 在 180° 以内随机选一个新朝向再试一次
            new_angle = (current_angle + np.pi + random.uniform(-np.pi / 2, np.pi / 2)) % (2 * np.pi)
            retry_idx = _pick_next_point_in_cone(
                current, positions_xyz, new_angle,
                step_size_xy=step_size_xy,
                dz_limit=dz_limit,
                cone_half_angle_rad=np.pi,   # 任意方位
                min_progress=min_progress_base * 0.3,
            )
            if retry_idx is not None:
                nearest_idx = retry_idx
                logger.debug("轨迹卡住 -> 大角度转向成功")
            else:
                # 真跳不出来 -> 传送到一个"远且开阔"的点
                nearest_idx = _pick_most_open_point(
                    current, positions_xyz,
                    radius=step_size_xy * 2.0,
                    max_radius=step_size_xy * rescue_max_factor,
                )
                logger.debug("轨迹卡住 -> 救援传送到开阔点")
                recent_xy_moves.clear()
                stuck_count = 0
        else:
            stuck_count = 0

        nxt = positions_xyz[nearest_idx].astype(np.float64)
        dx = nxt[0] - current[0]
        dy = nxt[1] - current[1]
        recent_xy_moves.append(float(np.hypot(dx, dy)))
        if len(recent_xy_moves) > stuck_window * 2:
            recent_xy_moves = recent_xy_moves[-stuck_window * 2:]

        # 更新 xy 朝向; 若位移过小就不更新, 避免继续朝墙
        if np.hypot(dx, dy) > step_size_xy * 0.1:
            current_angle = np.arctan2(dy, dx)
            if current_angle < 0:
                current_angle += 2 * np.pi

        path.append(nxt.copy())
        current = nxt

    return np.asarray(path, dtype=np.float64)


# ============================================================
# PLY 可视化(点云 + 线段)
# ============================================================
def _sphere_points(center: np.ndarray, radius: float, num: int = 80) -> np.ndarray:
    """在半径为 radius 的球体内生成 num 个点(基于斐波那契球面 + 少量径向抖动),
    用于把单个路径点"膨胀"成一个肉眼可见的小球。"""
    if num <= 0:
        return np.empty((0, 3), dtype=np.float64)
    indices = np.arange(num, dtype=np.float64) + 0.5
    phi = np.arccos(1 - 2 * indices / num)
    theta = np.pi * (1 + 5 ** 0.5) * indices  # 黄金角
    r = radius * (0.5 + 0.5 * np.cbrt(np.random.rand(num)))  # 随机径向抖动, 让球"实心"
    x = center[0] + r * np.sin(phi) * np.cos(theta)
    y = center[1] + r * np.sin(phi) * np.sin(theta)
    z = center[2] + r * np.cos(phi)
    return np.stack([x, y, z], axis=1)


def _thick_segment_points(p0: np.ndarray, p1: np.ndarray, step: float, radius: float,
                          cross_samples: int = 8) -> np.ndarray:
    """把线段 p0->p1 离散成密集的"粗管"点,用于在 PLY 里画出肉眼可见的"粗线"。
    Args:
        step: 沿线方向采样步长(米)
        radius: 粗线半径(米)
        cross_samples: 每个位置在横截面上撒几个点
    """
    v = p1 - p0
    L = float(np.linalg.norm(v))
    if L <= 1e-9:
        return _sphere_points(p0, radius, num=cross_samples)
    n_along = max(2, int(np.ceil(L / step)) + 1)
    ts = np.linspace(0.0, 1.0, n_along)
    centers = p0[None, :] + ts[:, None] * v[None, :]  # (n_along, 3)

    # 构造两个与 v 正交的单位向量 e1, e2
    v_hat = v / L
    # 任取一个不平行于 v_hat 的向量
    if abs(v_hat[2]) < 0.9:
        tmp = np.array([0.0, 0.0, 1.0])
    else:
        tmp = np.array([1.0, 0.0, 0.0])
    e1 = np.cross(v_hat, tmp)
    e1 /= (np.linalg.norm(e1) + 1e-12)
    e2 = np.cross(v_hat, e1)

    # 每个中心位置在横截面圆盘上撒 cross_samples 个点
    angles = np.linspace(0, 2 * np.pi, cross_samples, endpoint=False)
    rs = radius * (0.3 + 0.7 * np.sqrt(np.random.rand(cross_samples)))
    offsets = (rs[:, None] * np.cos(angles)[:, None] * e1[None, :] +
               rs[:, None] * np.sin(angles)[:, None] * e2[None, :])  # (cross, 3)

    # 广播叠加: (n_along, 1, 3) + (1, cross, 3) => (n_along, cross, 3)
    pts = centers[:, None, :] + offsets[None, :, :]
    return pts.reshape(-1, 3)


def _viridis_like_colormap(t: np.ndarray) -> np.ndarray:
    """简化的暖色渐变 (蓝 -> 青 -> 绿 -> 黄 -> 红), 用于标识路径点顺序。
    t: (N,) 取值 [0,1], 返回 (N,3) uint8"""
    t = np.clip(t, 0.0, 1.0)
    stops = np.array([
        [30, 60, 200],    # 起点: 深蓝
        [30, 180, 220],   # 青
        [60, 220, 80],    # 绿
        [255, 220, 40],   # 黄
        [230, 40, 40],    # 终点: 红
    ], dtype=np.float64)
    n = len(stops) - 1
    idx = t * n
    lo = np.clip(np.floor(idx).astype(int), 0, n - 1)
    hi = lo + 1
    frac = (idx - lo)[:, None]
    colors = stops[lo] * (1 - frac) + stops[hi] * frac
    return np.clip(colors, 0, 255).astype(np.uint8)


def save_path_ply(
    path_xyz: np.ndarray,
    free_xyz: Optional[np.ndarray],
    ply_path: str,
    background_color: Tuple[int, int, int] = (210, 210, 210),
    max_background_points: int = 50_000,
    line_radius: Optional[float] = None,
    line_step: Optional[float] = None,
    point_radius: Optional[float] = None,
    start_color: Tuple[int, int, int] = (0, 255, 0),
    end_color: Tuple[int, int, int] = (255, 0, 0),
):
    """
    保存单条 3D 路径到 PLY, 让轨迹在 MeshLab/CloudCompare 中肉眼可见:
      - 背景 free 点云: 浅灰, 下采样到 max_background_points
      - 路径"粗线": 将相邻点之间采样成密集"粗管", 沿路径从蓝->青->绿->黄->红渐变
      - 路径点"小球": 每个路径点膨胀成一个实心小球; 起点绿色大球, 终点红色大球
      - 同时保留 PLY 的 edge 元素, 兼容支持 edge 渲染的看图软件

    Args:
        path_xyz:              (N, 3)
        free_xyz:              背景点云, 或 None
        ply_path:              输出文件
        background_color:      背景点颜色
        max_background_points: 背景点上限, 默认 5 万(比之前更少, 突出轨迹)
        line_radius:           粗管半径(米), 默认根据场景尺度自适应
        line_step:             粗管沿线采样步长(米), 默认 = line_radius
        point_radius:          路径点小球半径(米), 默认 = 3 * line_radius
    """
    os.makedirs(os.path.dirname(ply_path), exist_ok=True)

    path_xyz = np.asarray(path_xyz, dtype=np.float64)
    n_pts = len(path_xyz)

    # 自适应粗细: 基于路径整体 bbox 对角线
    if n_pts >= 2:
        bbox_diag = float(np.linalg.norm(path_xyz.max(axis=0) - path_xyz.min(axis=0)))
    else:
        bbox_diag = 1.0
    if line_radius is None:
        line_radius = max(0.02, bbox_diag * 0.006)  # 约 bbox 对角线的 0.6%
    if line_step is None:
        line_step = line_radius
    if point_radius is None:
        point_radius = line_radius * 3.0
    start_end_radius = point_radius * 1.5

    all_pts: List[np.ndarray] = []
    all_cols: List[np.ndarray] = []

    # ---- 背景点云 ----
    if free_xyz is not None and len(free_xyz) > 0:
        bg = free_xyz[:, :3]
        if len(bg) > max_background_points:
            idx = np.random.choice(len(bg), max_background_points, replace=False)
            bg = bg[idx]
        bg_cols = np.tile(np.array(background_color, dtype=np.uint8), (len(bg), 1))
        all_pts.append(bg)
        all_cols.append(bg_cols)

    # ---- 路径"粗管"线段(按段着色, 颜色沿路径渐变) ----
    if n_pts >= 2:
        for i in range(n_pts - 1):
            t = i / max(1, n_pts - 2)  # 段的颜色取该段起点位置对应的渐变
            color = _viridis_like_colormap(np.array([t]))[0]
            seg_pts = _thick_segment_points(
                path_xyz[i], path_xyz[i + 1],
                step=line_step, radius=line_radius, cross_samples=8,
            )
            seg_cols = np.tile(color, (len(seg_pts), 1))
            all_pts.append(seg_pts)
            all_cols.append(seg_cols)

    # ---- 路径点"小球" ----
    t_pts = np.linspace(0.0, 1.0, n_pts) if n_pts > 1 else np.array([0.0])
    pt_colors = _viridis_like_colormap(t_pts)
    for i, p in enumerate(path_xyz):
        if i == 0:
            r = start_end_radius
            c = np.array(start_color, dtype=np.uint8)
        elif i == n_pts - 1:
            r = start_end_radius
            c = np.array(end_color, dtype=np.uint8)
        else:
            r = point_radius
            c = pt_colors[i]
        sph = _sphere_points(p, r, num=120)
        all_pts.append(sph)
        all_cols.append(np.tile(c, (len(sph), 1)))

    # ---- 合并全部 vertex ----
    if all_pts:
        pts = np.concatenate(all_pts, axis=0)
        cols = np.concatenate(all_cols, axis=0).astype(np.uint8)
    else:
        pts = np.empty((0, 3)); cols = np.empty((0, 3), dtype=np.uint8)

    # ---- 在 vertex 数组末尾再追加"纯路径点"专用索引, 供 edge 元素引用 ----
    edge_vertex_start = len(pts)
    if n_pts >= 2:
        extra = path_xyz.astype(np.float32)
        extra_cols = _viridis_like_colormap(np.linspace(0, 1, n_pts))
        pts = np.concatenate([pts, extra], axis=0)
        cols = np.concatenate([cols, extra_cols], axis=0)

    vertex_array = np.empty(len(pts), dtype=[
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
    ])
    vertex_array['x'] = pts[:, 0]
    vertex_array['y'] = pts[:, 1]
    vertex_array['z'] = pts[:, 2]
    vertex_array['red'] = cols[:, 0]
    vertex_array['green'] = cols[:, 1]
    vertex_array['blue'] = cols[:, 2]

    elements = [PlyElement.describe(vertex_array, 'vertex')]

    if n_pts >= 2:
        edges = np.empty(n_pts - 1, dtype=[
            ('vertex1', 'i4'), ('vertex2', 'i4'),
            ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
        ])
        edge_cols = _viridis_like_colormap(np.linspace(0, 1, n_pts - 1))
        for i in range(n_pts - 1):
            edges[i] = (edge_vertex_start + i, edge_vertex_start + i + 1,
                        edge_cols[i, 0], edge_cols[i, 1], edge_cols[i, 2])
        elements.append(PlyElement.describe(edges, 'edge'))

    PlyData(elements, text=True).write(ply_path)


def save_filtered_points_ply(
    points_xyz: np.ndarray,
    ply_path: str,
    color: Tuple[int, int, int] = (100, 200, 100),
):
    """把过滤后的可行 free 点云保存为一个纯点云 PLY。"""
    os.makedirs(os.path.dirname(ply_path), exist_ok=True)
    if len(points_xyz) == 0:
        logger.warning(f"{ply_path}: 点数为 0,跳过写入")
        return
    verts = np.empty(len(points_xyz), dtype=[
        ('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
        ('red', 'u1'), ('green', 'u1'), ('blue', 'u1'),
    ])
    verts['x'] = points_xyz[:, 0]
    verts['y'] = points_xyz[:, 1]
    verts['z'] = points_xyz[:, 2]
    verts['red'] = color[0]
    verts['green'] = color[1]
    verts['blue'] = color[2]
    PlyData([PlyElement.describe(verts, 'vertex')], text=True).write(ply_path)


# ============================================================
# 路径空间范围检查(过滤角落蜷缩轨迹)
# ============================================================
def path_bounding_extents(path_xyz: np.ndarray) -> np.ndarray:
    """返回轨迹 xyz 各轴上的包围盒跨度 [dx, dy, dz]。"""
    mins = np.min(path_xyz, axis=0)
    maxs = np.max(path_xyz, axis=0)
    return maxs - mins


def is_path_too_compact(
    path_xyz: np.ndarray,
    min_path_extent: float,
    window_size: int = 40,
) -> Tuple[bool, int, np.ndarray]:
    """
    判断轨迹是否含有蜷缩在过小空间内的连续点段(常见于角落/无需采集区域)。

    对长度为 window_size 的滑动窗口检查 xyz 包围盒最大轴跨度:
      - 轨迹点数 >= window_size: 检查每一段连续 window_size 个点
      - 轨迹点数 < window_size: 用全部点作为一段检查

    任一段的最大轴跨度 < min_path_extent 则整条轨迹应丢弃。
    例如 window_size=40, min_path_extent=0.25 表示任意连续 40 点在任意方向上
    都没有走出 0.25m。

    Returns:
        (is_too_compact, window_start, extents)
        window_start: 触发丢弃的窗口起始索引; 未触发时为 -1
        extents: 触发窗口 (或全轨迹) 的 [dx, dy, dz]
    """
    if min_path_extent <= 0 or len(path_xyz) == 0:
        return False, -1, path_bounding_extents(path_xyz)

    window_size = max(1, int(window_size))
    n = len(path_xyz)
    actual_window = min(window_size, n)

    for start in range(n - actual_window + 1):
        segment = path_xyz[start:start + actual_window]
        extents = path_bounding_extents(segment)
        if float(np.max(extents)) < min_path_extent:
            return True, start, extents

    return False, -1, path_bounding_extents(path_xyz)


# ============================================================
# 入口:生成多条 3D 路径
# ============================================================
def gen_path_3d(
    free_position: np.ndarray,
    occupied_position: np.ndarray,
    output_dir: str,
    num_paths: int = 10,
    num_points: int = 30,
    resolution: float = 0.1,
    erode_iterations: int = 3,
    obstacle_dilate_iterations: int = 2,
    obstacle_envelope_iterations: int = 0,
    step_size_xy: float = 0.3,
    step_size_z: float = 0.1,
    max_angle_deviation: float = 10.0,
    max_dz_per_step: float = 0.1,
    min_path_extent: float = 0.25,
    min_path_compact_window: int = 40,
    max_path_generation_attempts: int = 100,
    save_filtered_ply: bool = True,
) -> np.ndarray:
    """
    生成若干条 3D 轨迹,保存轨迹 npy 和每条轨迹的 PLY 可视化。

    流程(室内室外通用):
      1) 3D 腐蚀 free_position, 让 free 点远离自身连通边界
      2) 3D 膨胀 occupied_position 得到"障碍禁区", 剔除落在禁区内的 free 点
         (防贴墙/穿模; obstacle_dilate_iterations)
      3) 取 occupied_position 的整体外接 shape (闭运算 + 3D 孔洞填充),
         仅保留落在外接 shape 内部的 free 点 (房间中心会被保留, 天空/旷野
         会被剔除; obstacle_envelope_iterations 为闭运算的膨胀层数, <=0 关闭)
      4) 在过滤后的点云上做 3D 随机游走
      5) 若轨迹中任意连续 min_path_compact_window 个点的 xyz 包围盒最大轴跨度
         < min_path_extent (不足 min_path_compact_window 点时检查全部点),
         视为角落蜷缩轨迹并重新生成
      6) 输出 paths.npy、filtered_free_positions.ply(可选)、以及每条路径 {idx:04d}.ply

    Returns:
        paths_xyz: (num_paths, num_points, 3) ndarray
    """
    os.makedirs(output_dir, exist_ok=True)

    free_xyz = free_position[:, :3] if free_position.shape[1] > 3 else free_position
    occupied_xyz = occupied_position[:, :3] if occupied_position.shape[1] > 3 else occupied_position

    eroded_xyz = erode_free_positions_3d(
        free_xyz, occupied_xyz, resolution, erode_iterations
    )

    filtered_xyz = filter_free_by_obstacle_dilation(
        eroded_xyz, occupied_xyz, resolution, obstacle_dilate_iterations
    )

    filtered_xyz = filter_free_by_obstacle_envelope(
        filtered_xyz, occupied_xyz, resolution, obstacle_envelope_iterations
    )

    if len(filtered_xyz) == 0:
        raise RuntimeError(
            "过滤后点数为 0,无法生成路径。请检查 erode_iterations / "
            "obstacle_dilate_iterations / obstacle_envelope_iterations / resolution 参数"
        )

    if save_filtered_ply:
        filtered_ply_path = os.path.join(output_dir, "filtered_free_positions.ply")
        save_filtered_points_ply(filtered_xyz, filtered_ply_path)
        logger.info(f"过滤后 free 点云已保存: {filtered_ply_path}, 点数: {len(filtered_xyz)}")

    paths_xyz: List[np.ndarray] = []
    for path_idx in range(num_paths):
        logger.info(f"生成 3D 路径 {path_idx + 1}/{num_paths}...")
        path_xyz = None
        for attempt in range(1, max_path_generation_attempts + 1):
            candidate = generate_random_path_3d(
                positions_xyz=filtered_xyz,
                num_points=num_points,
                step_size_xy=step_size_xy,
                step_size_z=step_size_z,
                max_angle_deviation=max_angle_deviation,
                max_dz_per_step=max_dz_per_step,
            )
            too_compact, window_start, extents = is_path_too_compact(
                candidate, min_path_extent, window_size=min_path_compact_window,
            )
            if too_compact:
                window_end = window_start + min(len(candidate), min_path_compact_window) - 1
                logger.warning(
                    f"路径 {path_idx + 1} 第 {attempt} 次尝试含过小空间段 "
                    f"(窗口 [{window_start}, {window_end}], "
                    f"extents=({extents[0]:.3f}, {extents[1]:.3f}, {extents[2]:.3f}), "
                    f"max={np.max(extents):.3f} < {min_path_extent}), 放弃并重试"
                )
                continue
            path_xyz = candidate
            if attempt > 1:
                logger.info(
                    f"路径 {path_idx + 1} 第 {attempt} 次尝试通过范围检查 "
                    f"(共 {len(candidate)} 点, 窗口={min_path_compact_window})"
                )
            break
        if path_xyz is None:
            raise RuntimeError(
                f"路径 {path_idx + 1} 在 {max_path_generation_attempts} 次尝试内"
                f"均未生成满足 min_path_extent={min_path_extent}, "
                f"window={min_path_compact_window} 的轨迹"
            )
        paths_xyz.append(path_xyz)

    paths_arr = np.stack(paths_xyz, axis=0)
    paths_npy_path = os.path.join(output_dir, "paths.npy")
    np.save(paths_npy_path, paths_arr)
    logger.info(f"路径已保存: {paths_npy_path}, 形状: {paths_arr.shape}")

    # 为了节约时间,注释掉
    for path_idx, path_xyz in enumerate(paths_xyz):
        ply_path = os.path.join(output_dir, f"{path_idx:04d}.ply")
        save_path_ply(path_xyz, filtered_xyz, ply_path)
    logger.info(f"已保存 {len(paths_xyz)} 条路径的 PLY 可视化到 {output_dir}")

    return paths_arr
