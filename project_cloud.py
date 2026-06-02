"""将 Isaac Sim 采集的 RGB + Depth 数据反投影为 3D 点云, 用于跨相机几何一致性验证。

支持两种相机模型, 自动根据 `common/<frame_id>.npy` 里每个相机的 `intrinsics_full`
字段判断:
  - LUT 鱼眼相机 (intrinsics_full["calibration"] 存在): 用 MEI 统一全向模型反投影
      内参 + 畸变直接从 common/*.npy 里 bake 过来的标定取(与相机 USD 中 bake 的
      omni:calibration:* 一致)。
  - 针孔相机 (没有 calibration 字段): 用标准 pinhole 反投影
      内参直接取 common/*.npy 中保存的 K 矩阵 (gen_data.py 运行时由 IsaacCamera 算出)。

所有相机的点云都用 common/*.npy 里 `extrinsics_world`(OpenCV 相机系 → 世界系)
变换到世界坐标系并合并, 验证相机外参 + 内参的几何一致性: 不同相机的点云
应该在重叠区域对齐, 不会出现明显偏移/双层。

参考:
- UniK3D MEI camera: https://github.com/lpiccinelli-eth/UniK3D/blob/main/unik3d/utils/camera.py#L1000
- docs/thinking.md: MEI 逆向投影数学推导

用法:
    python project_cloud.py \
        --data_dir /home/fufa/projects2026/DataGen_omni/workdir/home000 \
        --output_dir /home/fufa/projects2026/DataGen_omni/workdir/home000/vis
    未写 --frame_id 时按 rgb 排序处理前 --show_num 张。
"""

import os
import argparse
import numpy as np
from PIL import Image

from sdg_utils.projection_lut import (
    read_lut_enter_exr,
    depth_to_pointcloud_lut,
    depth_to_pointcloud_mei as depth_to_pointcloud_mei_shared,
)


# ===================== MEI 模型反投影 =====================

def undistort_radtan(mx_d: np.ndarray, my_d: np.ndarray,
                     k1: float, k2: float, p1: float, p2: float,
                     max_iters: int = 20) -> tuple:
    """迭代去 RadTan 畸变 (Newton 法)。

    输入: 带畸变的归一化坐标 (mx_d, my_d)
    输出: 无畸变的归一化坐标 (mx, my)
    """
    mx = mx_d.copy()
    my = my_d.copy()

    for _ in range(max_iters):
        r2 = mx * mx + my * my
        r4 = r2 * r2
        radial = 1.0 + k1 * r2 + k2 * r4

        # 正向畸变模型
        mx_est = mx * radial + 2.0 * p1 * mx * my + p2 * (r2 + 2.0 * mx * mx)
        my_est = my * radial + p1 * (r2 + 2.0 * my * my) + 2.0 * p2 * mx * my

        # Jacobian (2x2)
        dr2_dmx = 2.0 * mx
        dr2_dmy = 2.0 * my
        dradial_dmx = k1 * dr2_dmx + k2 * 2.0 * r2 * dr2_dmx
        dradial_dmy = k1 * dr2_dmy + k2 * 2.0 * r2 * dr2_dmy

        J00 = radial + mx * dradial_dmx + 2.0 * p1 * my + p2 * (dr2_dmx + 4.0 * mx)
        J01 = mx * dradial_dmy + 2.0 * p1 * mx + p2 * dr2_dmy
        J10 = my * dradial_dmx + p1 * dr2_dmx + 2.0 * p2 * my
        J11 = radial + my * dradial_dmy + p1 * (dr2_dmy + 4.0 * my) + 2.0 * p2 * mx

        # residual
        ex = mx_d - mx_est
        ey = my_d - my_est

        # 2x2 inverse
        det = J00 * J11 - J01 * J10
        det = np.where(np.abs(det) < 1e-12, 1e-12, det)
        inv_det = 1.0 / det

        dmx = inv_det * (J11 * ex - J01 * ey)
        dmy = inv_det * (-J10 * ex + J00 * ey)

        mx = mx + dmx
        my = my + dmy

    return mx, my


def mei_unproject(u: np.ndarray, v: np.ndarray,
                  fx: float, fy: float, cx: float, cy: float,
                  xi: float,
                  k1: float, k2: float, p1: float, p2: float,
                  undistort_iters: int = 20) -> np.ndarray:
    """MEI 统一全向模型: 像素 → 3D 射线方向。

    参考 UniK3D MEI.unproject 实现。

    Args:
        u, v: 像素坐标数组, shape (N,)
        fx, fy, cx, cy: pinhole 内参
        xi: MEI 模型参数
        k1, k2, p1, p2: RadTan 畸变系数
        undistort_iters: 迭代去畸变次数

    Returns:
        rays: shape (N, 3), 每行为一个 3D 射线方向 (未归一化)
    """
    # Step 1: 内参逆变换
    mx_d = (u - cx) / fx
    my_d = (v - cy) / fy

    # Step 2: 迭代去 RadTan 畸变
    has_distortion = abs(k1) + abs(k2) + abs(p1) + abs(p2) > 1e-10
    if has_distortion:
        mx, my = undistort_radtan(mx_d, my_d, k1, k2, p1, p2, undistort_iters)
    else:
        mx, my = mx_d, my_d

    # Step 3: MEI 逆投影 (参考 UniK3D MEI.unproject)
    rho2 = mx * mx + my * my

    if abs(xi - 1.0) < 1e-8:
        P_z = (1.0 - rho2) / 2.0
    else:
        sqrt_term = np.sqrt(np.maximum(1.0 + (1.0 - xi * xi) * rho2, 0.0))
        P_z = 1.0 - xi * (rho2 + 1.0) / (xi + sqrt_term)

    rays = np.stack([mx, my, P_z], axis=-1)

    # 处理 NaN
    nan_mask = np.isnan(rays).any(axis=-1)
    rays[nan_mask] = 0.0

    return rays


def depth_to_pointcloud_mei(rgb: np.ndarray, depth: np.ndarray,
                            mask: np.ndarray,
                            fx: float, fy: float, cx: float, cy: float,
                            xi: float,
                            k1: float, k2: float, p1: float, p2: float,
                            undistort_iters: int = 20,
                            min_pz: float = 0.05,
                            min_depth: float = 0.01,
                            max_depth: float = 100.0) -> tuple:
    """使用 MEI 模型将 RGB + depth 转为点云。

    对 LUT 鱼眼相机, Isaac Sim 的 distance_to_image_plane 是 3D 交点在
    相机坐标系中的 Z 坐标(沿光轴方向)。每个像素的光线方向由 MEI 模型决定,
    因此需要用 MEI 反投影得到射线方向 ray = (mx, my, P_z), 然后用
    scale = depth / P_z 缩放, 使得反投影后的 Z 坐标等于 depth 值。

    MEI 模型 xi > 1 时, 边缘像素的 P_z 可能接近 0 或为负, 导致 scale 发散或翻转。
    通过 min_pz 阈值过滤这些数值不稳定的边缘像素。
    """
    H, W = depth.shape

    valid = (mask > 0) & np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)

    v_grid, u_grid = np.mgrid[0:H, 0:W]
    u_valid = u_grid[valid].astype(np.float64)
    v_valid = v_grid[valid].astype(np.float64)
    depth_valid = depth[valid].astype(np.float64)

    rays = mei_unproject(u_valid, v_valid, fx, fy, cx, cy, xi,
                         k1, k2, p1, p2, undistort_iters)

    # scale = depth / P_z, 使得 Z_cam = P_z * scale = depth
    ray_z = rays[:, 2]
    stable = ray_z > min_pz
    scale = depth_valid[stable] / ray_z[stable]

    points = rays[stable] * scale[:, np.newaxis]
    colors = rgb[valid][stable]

    return points, colors


def depth_to_pointcloud_pinhole(rgb: np.ndarray, depth: np.ndarray,
                                mask: np.ndarray,
                                K: np.ndarray,
                                min_depth: float = 0.01,
                                max_depth: float = 100.0) -> tuple:
    """使用标准 Pinhole 模型将 RGB + depth 转为点云。"""
    H, W = depth.shape
    K_inv = np.linalg.inv(K)

    valid = (mask > 0) & np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)

    v_grid, u_grid = np.mgrid[0:H, 0:W]
    u_valid = u_grid[valid].astype(np.float64)
    v_valid = v_grid[valid].astype(np.float64)
    depth_valid = depth[valid].astype(np.float64)

    pixels = np.stack([u_valid, v_valid, np.ones_like(u_valid)], axis=-1)  # (N, 3)
    rays = (K_inv @ pixels.T).T  # (N, 3)
    points = rays * depth_valid[:, np.newaxis]

    colors = rgb[valid]

    return points, colors


# ===================== PLY 写入 =====================

def save_ply_binary(filepath: str, points: np.ndarray, colors: np.ndarray):
    """保存带颜色的点云为二进制 PLY (更快更小)。"""
    N = points.shape[0]
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {N}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )

    dtype = np.dtype([
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
        ('r', 'u1'), ('g', 'u1'), ('b', 'u1'),
    ])

    vertex_data = np.empty(N, dtype=dtype)
    vertex_data['x'] = points[:, 0].astype(np.float32)
    vertex_data['y'] = points[:, 1].astype(np.float32)
    vertex_data['z'] = points[:, 2].astype(np.float32)
    vertex_data['r'] = colors[:, 0].astype(np.uint8)
    vertex_data['g'] = colors[:, 1].astype(np.uint8)
    vertex_data['b'] = colors[:, 2].astype(np.uint8)

    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        vertex_data.tofile(f)


# ===================== 主流程辅助函数 =====================

def list_rgb_image_names_sorted(data_dir: str, ref_cam: str) -> list:
    """rgb/<ref_cam>/ 下所有 png/jpg/jpeg 文件名, 排序。"""
    rgb_dir = os.path.join(data_dir, 'rgb', ref_cam)
    if not os.path.isdir(rgb_dir):
        return []
    image_names = []
    for image_name in os.listdir(rgb_dir):
        if image_name.endswith('.png') or image_name.endswith('.jpg') or image_name.endswith('.jpeg'):
            image_names.append(image_name)
    image_names.sort()
    return image_names


def resolve_rgb_path(data_dir: str, cam_name: str, frame_id: str):
    """解析 rgb 路径: 按 jpg / jpeg / png 顺序找已存在的文件。"""
    d = os.path.join(data_dir, 'rgb', cam_name)
    if not os.path.isdir(d):
        return None
    for ext in ('.jpg', '.jpeg', '.png'):
        p = os.path.join(d, frame_id + ext)
        if os.path.isfile(p):
            return p
    return None


def transform_points(points: np.ndarray, T: np.ndarray) -> np.ndarray:
    """用 4x4 变换矩阵变换 3D 点。"""
    N = points.shape[0]
    ones = np.ones((N, 1), dtype=points.dtype)
    points_homo = np.hstack([points, ones])  # (N, 4)
    transformed = (T @ points_homo.T).T  # (N, 4)
    return transformed[:, :3]


def _resolve_lut_enter_path(intrinsics_full: dict) -> str | None:
    """从 common/*.npy 的 intrinsics_full 解析 LUT enter EXR 路径。"""
    if not isinstance(intrinsics_full, dict):
        return None
    raw = intrinsics_full.get("generalizedProjectionDirectionTexturePath")
    if not raw:
        return None
    path = os.path.normpath(str(raw))
    if os.path.isfile(path):
        return path
    return None


def _load_lut_enter_cached(cache: dict, exr_path: str) -> np.ndarray:
    if exr_path not in cache:
        cache[exr_path] = read_lut_enter_exr(exr_path)
    return cache[exr_path]


def _extract_mei_params(intrinsics_full: dict):
    """从 common/*.npy 里 `intrinsics_full` 字段提取 MEI 反投影所需参数。

    返回 (fx, fy, cx, cy, xi, k1, k2, p1, p2) 或 None (没有 omni calibration 时)。
    """
    if not isinstance(intrinsics_full, dict):
        return None
    cal = intrinsics_full.get("calibration")
    if cal is None:
        return None
    fx = float(cal["fx_px"])
    fy = float(cal["fy_px"])
    cx = float(cal["cx_px"])
    cy = float(cal["cy_px"])
    xi = float(cal.get("xi", 0.0))
    # distortion_coeffs 顺序: [k1, k2, p1, p2, ...]
    dist = list(cal.get("distortion_coeffs", []))
    while len(dist) < 4:
        dist.append(0.0)
    k1, k2, p1, p2 = float(dist[0]), float(dist[1]), float(dist[2]), float(dist[3])
    return fx, fy, cx, cy, xi, k1, k2, p1, p2


# ===================== 单相机处理 =====================

def process_single_camera(cam_name: str, data_dir: str, frame_id: str,
                          common_data: dict, output_dir: str,
                          undistort_iters: int, downsample: int = 1,
                          lut_cache: dict | None = None):
    """处理单个相机的 RGB + Depth → 点云 (世界坐标系)。

    投影模型由 common_data[cam_name]['intrinsics_full'] 自动决定:
      - 有 generalizedProjectionDirectionTexturePath: LUT 反投影 (与 RTX 一致)
      - 含 `calibration` 字段(omni 鱼眼标定): MEI 反投影
      - 否则: 标准 pinhole 反投影 (K 取 intrinsics)

    Returns:
        (points_world, colors) - 已变换到世界坐标系
    """
    print(f"\n{'='*60}")
    print(f"处理相机: {cam_name}")
    print(f"{'='*60}")

    if cam_name not in common_data:
        print(f"  [跳过] {cam_name} 不在 common/{frame_id}.npy 中")
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)

    rgb_path = resolve_rgb_path(data_dir, cam_name, frame_id)
    if rgb_path is None:
        print(f"  [跳过] 未找到 RGB: rgb/{cam_name}/{frame_id}.(jpg|jpeg|png)")
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)
    depth_path = os.path.join(data_dir, 'depth', cam_name, f'{frame_id}.npy')
    mask_path = os.path.join(data_dir, 'mask', f'{cam_name}_mask.png')

    if not os.path.isfile(depth_path):
        print(f"  [跳过] 未找到 depth: {depth_path}")
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)

    rgb = np.array(Image.open(rgb_path))
    depth = np.load(depth_path)
    if os.path.isfile(mask_path):
        mask = np.array(Image.open(mask_path))
    else:
        # 没有 mask 文件(例如针孔相机没保存): 用全 255
        mask = np.full(depth.shape, 255, dtype=np.uint8)

    print(f"  RGB shape:   {rgb.shape}")
    finite_mask = np.isfinite(depth)
    if finite_mask.any():
        print(f"  Depth shape: {depth.shape}, "
              f"finite range: [{depth[finite_mask].min():.3f}, {depth[finite_mask].max():.3f}]")
    else:
        print(f"  Depth shape: {depth.shape}, 全部为 inf/NaN (跳过)")
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)
    print(f"  Mask valid:  {np.count_nonzero(mask)}/{mask.size} "
          f"({np.count_nonzero(mask)/mask.size*100:.1f}%)")

    if downsample > 1:
        rgb = rgb[::downsample, ::downsample]
        depth = depth[::downsample, ::downsample]
        mask = mask[::downsample, ::downsample]
        print(f"  降采样 {downsample}x → RGB {rgb.shape}, Depth {depth.shape}")

    cam_common = common_data[cam_name]
    T_cam_to_world = np.asarray(cam_common['extrinsics_world'], dtype=np.float64)
    intrinsics_full = cam_common.get('intrinsics_full')

    if lut_cache is None:
        lut_cache = {}

    lut_path = _resolve_lut_enter_path(intrinsics_full)
    if lut_path is not None:
        lut_enter = _load_lut_enter_cached(lut_cache, lut_path)
        print(f"  LUT 反投影 (distance_to_image_plane, enter EXR): {lut_path}")
        pts_cam, colors_cam = depth_to_pointcloud_lut(
            rgb, depth, mask, lut_enter,
            min_depth=0.01, max_depth=100.0,
        )
        suffix = "lut_world"
    else:
        mei_params = _extract_mei_params(intrinsics_full)
        if mei_params is not None:
            fx, fy, cx, cy, xi, k1, k2, p1, p2 = mei_params
            if downsample > 1:
                fx /= downsample
                fy /= downsample
                cx /= downsample
                cy /= downsample
            print(f"  MEI 反投影 (xi={xi:.3f}, fx={fx:.1f}, fy={fy:.1f}, "
                  f"cx={cx:.1f}, cy={cy:.1f}, iters={undistort_iters})")
            dist = [k1, k2, p1, p2]
            pts_cam, colors_cam = depth_to_pointcloud_mei_shared(
                rgb, depth, mask,
                fx, fy, cx, cy, xi, dist,
                undistort_iters=undistort_iters,
                min_pz=0.05, min_depth=0.01, max_depth=100.0,
            )
            suffix = "mei_world"
        else:
            # 针孔: 用 common 里运行时算出的 K
            K = np.asarray(cam_common['intrinsics'], dtype=np.float64).copy()
            if downsample > 1:
                K[0, 0] /= downsample
                K[1, 1] /= downsample
                K[0, 2] /= downsample
                K[1, 2] /= downsample
            print(f"  Pinhole 反投影 (K from common):"
                  f" fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
            pts_cam, colors_cam = depth_to_pointcloud_pinhole(
                rgb, depth, mask, K,
                min_depth=0.01, max_depth=100.0,
            )
            suffix = "pinhole_world"

    print(f"  点云: {pts_cam.shape[0]} 点")
    if pts_cam.shape[0] == 0:
        return np.zeros((0, 3)), np.zeros((0, 3), dtype=np.uint8)

    pts_world = transform_points(pts_cam, T_cam_to_world)
    out_path = os.path.join(output_dir, f'{cam_name}_{suffix}_{frame_id}.ply')
    save_ply_binary(out_path, pts_world, colors_cam)
    print(f"  保存世界坐标系: {out_path}")
    return pts_world, colors_cam


# ===================== 主流程 =====================

def main():
    parser = argparse.ArgumentParser(description='RGB+Depth → 点云 (MEI / Pinhole 自动选择)')
    parser.add_argument('--data_dir', type=str, default=None, help='数据根目录 (含 rgb/ depth/ mask/ common/)')
    parser.add_argument('--output_dir', type=str, default=None, help='点云输出目录')
    parser.add_argument('--frame_id', type=str, default=None,
                        help='帧 ID(与 common/depth 的 npy stem 一致); '
                             '指定后只处理该帧, 忽略 --show_num')
    parser.add_argument('--show_num', type=int, default=4,
                        help='未指定 --frame_id 时, 只可视化排序后的前 N 张')
    parser.add_argument('--undistort_iters', type=int, default=20,
                        help='RadTan 去畸变迭代次数')
    parser.add_argument('--downsample', type=int, default=1,
                        help='降采样倍数 (1=不降采样, 2=2x降采样, ...)')
    parser.add_argument('--cameras', type=str, nargs='+', default=None,
                        help='要处理的相机列表 (None 时自动用 common/*.npy 里的全部相机)')
    parser.add_argument('--ref_cam', type=str, default=None,
                        help='扫描帧 ID 时用的参考相机 (None 时取 rgb/ 下任意子目录)')
    args = parser.parse_args()

    if args.output_dir is None:
        args.output_dir = os.path.join(args.data_dir, 'vis')
    os.makedirs(args.output_dir, exist_ok=True)

    # ref_cam: 用来扫描 rgb/<ref_cam>/ 目录找帧
    if args.ref_cam is not None:
        ref_cam = args.ref_cam
    else:
        rgb_root = os.path.join(args.data_dir, 'rgb')
        if not os.path.isdir(rgb_root):
            print(f"未找到 {rgb_root}, 跳过。")
            return
        subdirs = [d for d in os.listdir(rgb_root)
                   if os.path.isdir(os.path.join(rgb_root, d))]
        if not subdirs:
            print(f"{rgb_root} 下没有相机子目录, 跳过。")
            return
        subdirs.sort()
        ref_cam = subdirs[0]
        print(f"自动选择 ref_cam = {ref_cam}")

    image_names = list_rgb_image_names_sorted(args.data_dir, ref_cam)
    if not image_names:
        print(f"rgb/{ref_cam} 下无图像 (png/jpg/jpeg), 跳过。")
        return

    if args.frame_id is not None:
        frame_ids = [args.frame_id]
    else:
        n = max(1, args.show_num)
        frame_ids = [os.path.splitext(name)[0] for name in image_names[:n]]
        print(f"未指定 --frame_id, 处理排序后前 {len(frame_ids)}/{min(n, len(image_names))} 帧 "
              f"(show_num={args.show_num}, 目录共 {len(image_names)} 张)")

    os.makedirs(args.output_dir, exist_ok=True)

    for frame_idx, frame_id in enumerate(frame_ids):
        print(f"\n{'='*60}")
        print(f"====> 帧 {frame_idx + 1}/{len(frame_ids)}: {frame_id} <====")
        print(f"{'='*60}")

        common_path = os.path.join(args.data_dir, 'common', f'{frame_id}.npy')
        if not os.path.isfile(common_path):
            print(f"[跳过] 未找到 common: {common_path}")
            continue
        print(f"加载内外参数据: {common_path}")
        common_data = np.load(common_path, allow_pickle=True).item()

        cameras = args.cameras if args.cameras is not None else sorted(common_data.keys())
        print(f"本帧待处理相机: {cameras}")

        all_world_points = []
        all_world_colors = []
        lut_cache = {}

        for cam_name in cameras:
            pts_w, colors_w = process_single_camera(
                cam_name, args.data_dir, frame_id,
                common_data, args.output_dir,
                args.undistort_iters, args.downsample,
                lut_cache=lut_cache,
            )
            if pts_w.shape[0] == 0:
                continue
            all_world_points.append(pts_w)
            all_world_colors.append(colors_w)

        print(f"\n{'='*60}")
        print(f"合并所有相机点云 ({frame_id}) ...")
        print(f"{'='*60}")

        if not all_world_points:
            print("[警告] 本帧所有相机点云均为空, 跳过合并")
            continue

        merged_pts = np.vstack(all_world_points)
        merged_colors = np.vstack(all_world_colors)

        merged_path = os.path.join(args.output_dir, f'all_cameras_world_{frame_id}.ply')
        save_ply_binary(merged_path, merged_pts, merged_colors)

        print(f"合并点云: {merged_pts.shape[0]} 点 → {merged_path}")

    print(f"\n完成! 共处理 {len(frame_ids)} 帧。请用 MeshLab / CloudCompare 打开 PLY 文件查看。")


if __name__ == '__main__':
    main()
