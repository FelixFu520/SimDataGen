"""LUT / Mei 深度反投影(无 Isaac Sim 依赖,供 project_cloud 等离线工具使用)。"""

from __future__ import annotations

from typing import List

import numpy as np

# rayEnterDirection.exr 有效像素判定阈值(与 RTX 渲染 FOV 对齐)。
# EXR 每像素存 RTX 相机系 (X右,Y上,Z后) 单位射线方向;无效 FOV 外像素为零向量。
#
# _RAY_NORM_MIN:过滤 FOV 外零向量。有效像素 norm≈1,无效≈0;0.25 留浮点容差。
#   调大 → mask 缩小,边界 norm 略低者可能被误删;
#   调小 → mask 扩大,可能纳入 FOV 外无效像素。
#
# _RENDERABLE_DIRZ_MAX:过滤 EXR 有值但 RTX 不渲染的黑角(dirZ 偏正,指向相机后方)。
#   调大 → mask 扩大,含黑角,与 RGB/depth 有效区错位;
#   调小 → mask 缩小,边缘有效像素可能被裁掉。
#
# 仅影响 Python 侧 mask 判定(save_cameras_mask、depth 反投影等),不改变 RTX 渲染。
# tools/cameras/oak_compute_mask_radius.py 须与此处阈值保持一致。
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


def compute_mask_radius_from_enter_data(
    data: np.ndarray,
    *,
    scale: float = 1.0,
    center_x: float | None = None,
    center_y: float | None = None,
) -> float:
    """从 rayEnterDirection 数组估计圆形 mask 半径(像素)。

    在圆心处取覆盖所有「可渲染」像素的最小外接圆半径(ceil)。
    center_x/y 为 None 时使用图像几何中心。
    """
    h, w = data.shape[:2]
    valid = mask_from_lut_enter_exr(data) > 0
    if not np.any(valid):
        raise RuntimeError("no renderable pixels in LUT enter data")
    cx = w / 2.0 if center_x is None else float(center_x)
    cy = h / 2.0 if center_y is None else float(center_y)
    ys, xs = np.where(valid)
    dist = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2)
    return float(np.ceil(dist.max())) * scale


def compute_mask_radius_from_enter_exr(
    exr_path: str,
    *,
    scale: float = 1.0,
    center_x: float | None = None,
    center_y: float | None = None,
) -> float:
    """从 rayEnterDirection.exr 估计圆形 mask 半径(像素)。"""
    data = read_lut_enter_exr(exr_path)
    return compute_mask_radius_from_enter_data(
        data, scale=scale, center_x=center_x, center_y=center_y
    )


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

    # distance_to_image_plane:沿 RTX -Z(相机前方)的距离
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


def equidistant_principal_domain(dist_coeffs: List[float]) -> tuple[float, float]:
    """Kannala-Brandt 等距模型主分支的 (theta_limit, theta_d_max)。

    多项式在更大角度上可能折叠。只保留导数首次变非正之前的主分支，
    与 OpenCV fisheye 从 theta_d 出发的牛顿迭代一致。
    """
    k1, k2, k3, k4 = (list(dist_coeffs) + [0.0, 0.0, 0.0, 0.0])[:4]
    thetas = np.linspace(0.0, np.pi, 20001)
    t2 = thetas * thetas
    t4 = t2 * t2
    t6 = t4 * t2
    t8 = t4 * t4
    theta_d = thetas * (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8)
    dg = 1.0 + 3.0 * k1 * t2 + 5.0 * k2 * t4 + 7.0 * k3 * t6 + 9.0 * k4 * t8
    neg = np.where(dg[1:] <= 0.0)[0]
    if neg.size == 0:
        return float(np.pi), float(theta_d[-1])
    end = int(neg[0]) + 2
    j = int(np.argmax(theta_d[:end]))
    return float(thetas[j]), float(theta_d[j])


def _equidistant_theta_d(theta: np.ndarray, k1: float, k2: float, k3: float, k4: float):
    t2 = theta * theta
    t4 = t2 * t2
    t6 = t4 * t2
    t8 = t4 * t4
    theta_d = theta * (1.0 + k1 * t2 + k2 * t4 + k3 * t6 + k4 * t8)
    dtheta_d = 1.0 + 3.0 * k1 * t2 + 5.0 * k2 * t4 + 7.0 * k3 * t6 + 9.0 * k4 * t8
    return theta_d, dtheta_d


def equidistant_unproject(
    u: np.ndarray,
    v: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    dist_coeffs: List[float],
    *,
    theta_limit: float | None = None,
    theta_d_max: float | None = None,
    iters: int = 20,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """OpenCV / Kalibr equidistant：像素 -> OpenCV 单位射线。无效像素为零向量。"""
    k1, k2, k3, k4 = (list(dist_coeffs) + [0.0, 0.0, 0.0, 0.0])[:4]
    if theta_limit is None or theta_d_max is None:
        theta_limit, theta_d_max = equidistant_principal_domain(dist_coeffs)

    mx = (u - cx) / fx
    my = (v - cy) / fy
    theta_d = np.sqrt(mx * mx + my * my)
    theta = np.clip(theta_d, 0.0, theta_limit)
    for _ in range(iters):
        theta_d_hat, dtheta_d = _equidistant_theta_d(theta, k1, k2, k3, k4)
        denom = np.where(np.abs(dtheta_d) < 1e-8, 1e-8, dtheta_d)
        theta = np.clip(theta - (theta_d_hat - theta_d) / denom, 0.0, theta_limit)

    theta_d_hat, _ = _equidistant_theta_d(theta, k1, k2, k3, k4)
    valid = (
        (theta_d <= theta_d_max + 1e-4)
        & (np.abs(theta_d_hat - theta_d) < 1e-4)
        & np.isfinite(theta)
    )
    small = theta_d < 1e-8
    scale = np.sin(theta) / np.maximum(theta_d, 1e-12)
    dx = np.where(small, 0.0, mx * scale)
    dy = np.where(small, 0.0, my * scale)
    dz = np.where(small, 1.0, np.cos(theta))
    norm = np.maximum(np.sqrt(dx * dx + dy * dy + dz * dz), 1e-12)
    dx = np.where(valid, dx / norm, 0.0)
    dy = np.where(valid, dy / norm, 0.0)
    dz = np.where(valid, dz / norm, 0.0)
    return dx, dy, dz, valid


def equidistant_project(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    dist_coeffs: List[float],
    *,
    theta_limit: float | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """OpenCV fisheye 正向投影。theta 超过主分支的方向标为无效。"""
    k1, k2, k3, k4 = (list(dist_coeffs) + [0.0, 0.0, 0.0, 0.0])[:4]
    if theta_limit is None:
        theta_limit, _ = equidistant_principal_domain(dist_coeffs)
    radius = np.sqrt(x * x + y * y)
    theta = np.arctan2(radius, z)
    theta_d, _ = _equidistant_theta_d(theta, k1, k2, k3, k4)
    scale = np.where(radius < 1e-12, 0.0, theta_d / np.maximum(radius, 1e-12))
    u = fx * (x * scale) + cx
    v = fy * (y * scale) + cy
    valid = (theta >= 0.0) & (theta <= theta_limit + 1e-6) & np.isfinite(u) & np.isfinite(v)
    return u, v, valid


def depth_to_pointcloud_equidistant(
    rgb: np.ndarray,
    depth: np.ndarray,
    mask: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    dist_coeffs: List[float],
    min_depth: float = 0.01,
    max_depth: float = 100.0,
    min_pz: float = 0.05,
) -> tuple[np.ndarray, np.ndarray]:
    """等距鱼眼解析反投影到 OpenCV 相机系(无 LUT 时回退)。"""
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
    dx, dy, dz, ray_ok = equidistant_unproject(u, v, fx, fy, cx, cy, dist_coeffs)
    rays = np.stack([dx, dy, dz], axis=-1)
    ray_z = rays[:, 2]
    stable = ray_ok & (ray_z > min_pz)
    pts = rays[stable] * (d[stable] / ray_z[stable])[:, None]
    return pts, rgb[v_idx, u_idx][stable]
