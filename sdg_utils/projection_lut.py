"""LUT / Mei 深度反投影(无 Isaac Sim 依赖,供 project_cloud 等离线工具使用)。"""

from __future__ import annotations

from typing import List

import numpy as np

# rayEnterDirection.exr 有效像素判定阈值(与 RTX 渲染 FOV 对齐)。
# EXR 每像素存 RTX 相机系 (X右,Y上,Z后) 单位射线方向;无效 FOV 外像素为零向量。
#
# _RAY_NORM_MIN：过滤 FOV 外零向量。有效像素 norm≈1,无效≈0;0.25 留浮点容差。
#   调大 → mask 缩小,边界 norm 略低者可能被误删;
#   调小 → mask 扩大,可能纳入 FOV 外无效像素。
#
# _RENDERABLE_DIRZ_MAX：过滤 EXR 有值但 RTX 不渲染的黑角(dirZ 偏正,指向相机后方)。
#   调大 → mask 扩大,含黑角,与 RGB/depth 有效区错位;
#   调小 → mask 缩小,边缘有效像素可能被裁掉。
#
# 仅影响 Python 侧 mask 判定(save_cameras_mask、depth 反投影等),不改变 RTX 渲染。
# tools/cameras/compute_mask_radius.py 有同名常量,修改时须同步。
_RAY_NORM_MIN = 0.25
_RENDERABLE_DIRZ_MAX = 0.2


def read_lut_enter_exr(path: str) -> np.ndarray:
    """读取 rayEnterDirection.exr -> (H,W,3) float32。"""
    import OpenEXR
    import Imath

    f = OpenEXR.InputFile(path)
    dw = f.header()["dataWindow"]
    w = dw.max.x - dw.min.x + 1
    h = dw.max.y - dw.min.y + 1
    chans = [
        np.frombuffer(
            f.channel(c, Imath.PixelType(Imath.PixelType.FLOAT)),
            dtype=np.float32,
        ).reshape(h, w)
        for c in "RGB"
    ]
    return np.stack(chans, axis=-1)


def mask_from_lut_enter_exr(data: np.ndarray) -> np.ndarray:
    """由 LUT enter 纹理生成与 RTX 渲染一致的有效像素 mask(255=有效, 0=无效)。"""
    norm = np.linalg.norm(data, axis=2)
    valid = (norm > _RAY_NORM_MIN) & (data[:, :, 2] < _RENDERABLE_DIRZ_MAX)
    return valid.astype(np.uint8) * 255


def sample_lut_rays_at_pixels(
    lut_enter: np.ndarray, u: np.ndarray, v: np.ndarray, img_w: int, img_h: int,
) -> np.ndarray:
    """在 LUT enter 纹理上按像素坐标采样 RTX 射线方向 (Hlut,Wlut,3)。"""
    lut_h, lut_w = lut_enter.shape[:2]
    tu = np.clip(np.round(u / img_w * lut_w).astype(np.int64), 0, lut_w - 1)
    tv = np.clip(np.round(v / img_h * lut_h).astype(np.int64), 0, lut_h - 1)
    return lut_enter[tv, tu].astype(np.float64)


def depth_to_pointcloud_lut(
    rgb: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    lut_enter: np.ndarray,
    min_depth: float = 0.01,
    max_depth: float = 100.0,
    min_forward_z: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """用 LUT enter 方向 + distance_to_image_plane 深度反投影到 OpenCV 相机系。

    LUT 存的是 RTX 相机系 (X右,Y上,Z后) 单位方向;depth 为沿 -Z 到成像面的距离。
    """
    img_h, img_w = depth.shape
    valid = (
        (mask > 0)
        & np.isfinite(depth)
        & (depth > min_depth)
        & (depth < max_depth)
    )
    v_idx, u_idx = np.where(valid)
    if u_idx.size == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)

    u = u_idx.astype(np.float64)
    v = v_idx.astype(np.float64)
    d = depth[v_idx, u_idx].astype(np.float64)
    rays_rtx = sample_lut_rays_at_pixels(lut_enter, u, v, img_w, img_h)
    norm = np.linalg.norm(rays_rtx, axis=1, keepdims=True)
    rays_rtx = rays_rtx / np.maximum(norm, 1e-12)

    # distance_to_image_plane：沿 RTX -Z(相机前方)的距离
    denom = np.maximum(-rays_rtx[:, 2], 1e-6)
    t = d / denom
    pts_rtx = rays_rtx * t[:, None]
    pts_cv = pts_rtx * np.array([1.0, -1.0, -1.0], dtype=np.float64)

    stable = pts_cv[:, 2] > min_forward_z
    return pts_cv[stable], rgb[v_idx, u_idx][stable]


def depth_to_pointcloud_mei(
    rgb: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    xi: float,
    dist_coeffs: List[float],
    min_depth: float = 0.01,
    max_depth: float = 100.0,
    min_pz: float = 0.05,
    undistort_iters: int = 20,
) -> tuple[np.ndarray, np.ndarray]:
    """Mei omni 解析反投影到 OpenCV 相机系(无 LUT 时回退)。"""
    k1, k2, p1, p2 = (list(dist_coeffs) + [0.0, 0.0, 0.0, 0.0])[:4]
    valid = (
        (mask > 0)
        & np.isfinite(depth)
        & (depth > min_depth)
        & (depth < max_depth)
    )
    v_idx, u_idx = np.where(valid)
    if u_idx.size == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)

    u = u_idx.astype(np.float64)
    v = v_idx.astype(np.float64)
    d = depth[v_idx, u_idx].astype(np.float64)
    mx_d = (u - cx) / fx
    my_d = (v - cy) / fy
    mx, my = mx_d.copy(), my_d.copy()
    for _ in range(undistort_iters):
        r2 = mx * mx + my * my
        radial = 1.0 + k1 * r2 + k2 * r2 * r2
        mx_est = mx * radial + 2.0 * p1 * mx * my + p2 * (r2 + 2.0 * mx * mx)
        my_est = my * radial + p1 * (r2 + 2.0 * my * my) + 2.0 * p2 * mx * my
        mx = mx + (mx_d - mx_est) / radial
        my = my + (my_d - my_est) / radial
    rho2 = mx * mx + my * my
    sqrt_t = np.sqrt(np.maximum(1.0 + (1.0 - xi * xi) * rho2, 0.0))
    pz = 1.0 - xi * (rho2 + 1.0) / (xi + sqrt_t)
    rays = np.stack([mx, my, pz], axis=-1)
    rays = rays / np.maximum(np.linalg.norm(rays, axis=1, keepdims=True), 1e-12)
    ray_z = rays[:, 2]
    stable = ray_z > min_pz
    pts = rays[stable] * (d[stable] / ray_z[stable])[:, None]
    return pts, rgb[v_idx, u_idx][stable]
