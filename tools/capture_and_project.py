"""最小端到端测试：在 (1,1,1) 采集一组相机的 RGB+Depth,反投影成世界系点云。

Usage:
    ./app/python.sh tools/capture_and_project.py --scene_usd /path/to/scene.usd --camera_usd assets/cameras/oak_camera_4lut_2H30YA.usd --output_dir workdir/capture_project_test

完整流程：
  1. 启动 SimulationApp (RTX Real-Time, 与 gen_data 一致)
  2. 加载场景 + 相机组 USD
  3. CameraRig 设置位姿并渲染
  4. 保存 RGB、Depth、Mask、内外参
  5. 调用纯 numpy 反投影函数,把 RGB+Depth 转世界系点云,写 PLY
"""

# ------------------------- Isaac Sim 启动 -------------------------
from isaacsim import SimulationApp

launch_config = {
    "headless": True,
    "renderer": "PathTracing",
    "rt_subframes": 8,
}
simulation_app = SimulationApp(launch_config=launch_config)

import carb
settings = carb.settings.get_settings()
settings.set("/rtx/verifyDriverVersion/enabled", False)

# ------------------------- 常规依赖 -------------------------
import os
import sys
import argparse
import numpy as np
from loguru import logger

logger.remove()
logger.add(sys.stdout, format="{message}", level="INFO", colorize=True)

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from sdg_utils.camera import CameraRig
from sdg_utils.usd import load_usd_file


# ------------------------- 反投影 -------------------------
def undistort_radtan(mx_d, my_d, k1, k2, p1, p2, max_iters=20):
    mx, my = mx_d.copy(), my_d.copy()
    for _ in range(max_iters):
        r2 = mx * mx + my * my
        radial = 1.0 + k1 * r2 + k2 * r2 * r2
        mx_est = mx * radial + 2.0 * p1 * mx * my + p2 * (r2 + 2.0 * mx * mx)
        my_est = my * radial + p1 * (r2 + 2.0 * my * my) + 2.0 * p2 * mx * my
        dr2_dmx, dr2_dmy = 2.0 * mx, 2.0 * my
        dradial_dmx = k1 * dr2_dmx + k2 * 2.0 * r2 * dr2_dmx
        dradial_dmy = k1 * dr2_dmy + k2 * 2.0 * r2 * dr2_dmy
        J00 = radial + mx * dradial_dmx + 2.0 * p1 * my + p2 * (dr2_dmx + 4.0 * mx)
        J01 = mx * dradial_dmy + 2.0 * p1 * mx + p2 * dr2_dmy
        J10 = my * dradial_dmx + p1 * dr2_dmx + 2.0 * p2 * my
        J11 = radial + my * dradial_dmy + p1 * (dr2_dmy + 4.0 * my) + 2.0 * p2 * mx
        ex, ey = mx_d - mx_est, my_d - my_est
        det = J00 * J11 - J01 * J10
        det = np.where(np.abs(det) < 1e-12, 1e-12, det)
        inv = 1.0 / det
        mx = mx + inv * (J11 * ex - J01 * ey)
        my = my + inv * (-J10 * ex + J00 * ey)
    return mx, my


def mei_unproject(u, v, fx, fy, cx, cy, xi, k1, k2, p1, p2, iters=20):
    mx_d = (u - cx) / fx
    my_d = (v - cy) / fy
    if abs(k1) + abs(k2) + abs(p1) + abs(p2) > 1e-10:
        mx, my = undistort_radtan(mx_d, my_d, k1, k2, p1, p2, iters)
    else:
        mx, my = mx_d, my_d
    rho2 = mx * mx + my * my
    if abs(xi - 1.0) < 1e-8:
        Pz = (1.0 - rho2) / 2.0
    else:
        sqrt_t = np.sqrt(np.maximum(1.0 + (1.0 - xi * xi) * rho2, 0.0))
        Pz = 1.0 - xi * (rho2 + 1.0) / (xi + sqrt_t)
    rays = np.stack([mx, my, Pz], axis=-1)
    rays[np.isnan(rays).any(axis=-1)] = 0.0
    return rays


def depth_to_pointcloud_mei(rgb, depth, mask, fx, fy, cx, cy, xi,
                            k1, k2, p1, p2,
                            min_pz=0.05, min_depth=0.01, max_depth=100.0):
    H, W = depth.shape
    valid = (mask > 0) & np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)
    v_g, u_g = np.mgrid[0:H, 0:W]
    u, v = u_g[valid].astype(np.float64), v_g[valid].astype(np.float64)
    d = depth[valid].astype(np.float64)
    rays = mei_unproject(u, v, fx, fy, cx, cy, xi, k1, k2, p1, p2)
    ray_z = rays[:, 2]
    stable = ray_z > min_pz
    pts = rays[stable] * (d[stable] / ray_z[stable])[:, None]
    cols = rgb[valid][stable]
    return pts, cols


def depth_to_pointcloud_pinhole(rgb, depth, mask, K,
                                min_depth=0.01, max_depth=100.0):
    H, W = depth.shape
    K_inv = np.linalg.inv(K)
    valid = (mask > 0) & np.isfinite(depth) & (depth > min_depth) & (depth < max_depth)
    v_g, u_g = np.mgrid[0:H, 0:W]
    u, v = u_g[valid].astype(np.float64), v_g[valid].astype(np.float64)
    d = depth[valid].astype(np.float64)
    pix = np.stack([u, v, np.ones_like(u)], axis=-1)  # (N,3)
    rays = (K_inv @ pix.T).T
    pts = rays * d[:, None]
    cols = rgb[valid]
    return pts, cols


def transform_points(pts, T):
    N = pts.shape[0]
    homo = np.hstack([pts, np.ones((N, 1), dtype=pts.dtype)])
    return (T @ homo.T).T[:, :3]


def save_ply_binary(filepath, pts, cols):
    N = pts.shape[0]
    header = (
        "ply\nformat binary_little_endian 1.0\n"
        f"element vertex {N}\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
    )
    dtype = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                      ('r', 'u1'), ('g', 'u1'), ('b', 'u1')])
    out = np.empty(N, dtype=dtype)
    out['x'] = pts[:, 0].astype(np.float32)
    out['y'] = pts[:, 1].astype(np.float32)
    out['z'] = pts[:, 2].astype(np.float32)
    out['r'] = cols[:, 0].astype(np.uint8)
    out['g'] = cols[:, 1].astype(np.uint8)
    out['b'] = cols[:, 2].astype(np.uint8)
    with open(filepath, 'wb') as f:
        f.write(header.encode('ascii'))
        out.tofile(f)


# ------------------------- 主流程 -------------------------
RENDER_COUNT = 5  # 与 gen_data.py 一致


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scene_usd", type=str, required=True, help="场景 USD 路径")
    p.add_argument(
        "--camera_usd", type=str,
        default="assets/cameras/oak_camera_4lut_2H30YA.usd",
        help="相机组 USD(相对项目根目录或绝对路径)",
    )
    p.add_argument(
        "--output_dir", type=str,
        default="workdir/capture_project_test",
        help="数据输出目录",
    )
    p.add_argument("--xyz", type=float, nargs=3, default=[1.0, 1.0, 1.0],
                   help="相机组在世界系的位置")
    p.add_argument("--rpy", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                   help="相机组的 roll/pitch/yaw (degrees)")
    p.add_argument("--frame_id", type=str, default="0000")
    return p.parse_args()


def main():
    args = parse_args()

    camera_usd = args.camera_usd if os.path.isabs(args.camera_usd) \
        else os.path.join(ROOT_DIR, args.camera_usd)
    output_dir = args.output_dir if os.path.isabs(args.output_dir) \
        else os.path.join(ROOT_DIR, args.output_dir)
    os.makedirs(output_dir, exist_ok=True)

    # ---- 1. World + 场景 (与 gen_data 相同: open_stage 打开场景 USD) ----
    logger.info(f"[1/5] 创建 World 并加载场景: {args.scene_usd}")
    world, stage = load_usd_file(args.scene_usd)
    world.reset()
    for _ in range(RENDER_COUNT):
        simulation_app.update()

    # ---- 2. CameraRig ----
    logger.info(f"[2/5] 创建 CameraRig: {camera_usd}")
    rig = CameraRig(
        camera_usd_path=camera_usd,
        world=world,
        stage=stage,
        rig_prim_path="/World/camera_rig",
    )

    # ---- 3. 设置位姿 + reset/initialize ----
    x, y, z = args.xyz
    roll, pitch, yaw = args.rpy
    logger.info(f"[3/5] 设置相机组位姿: xyz=({x:.3f},{y:.3f},{z:.3f}) rpy=({roll:.1f},{pitch:.1f},{yaw:.1f})")
    rig.set_pose(x, y, z, roll, pitch, yaw)

    world.reset()
    rig.initialize(attach_depth=True)

    for _ in range(RENDER_COUNT):
        world.step(render=True)
        simulation_app.update()

    # ---- 4. 采集数据 + 保存 ----
    logger.info(f"[4/5] 采集 RGB + Depth + 内外参,输出到 {output_dir}")
    rgb_dir = os.path.join(output_dir, "rgb")
    depth_dir = os.path.join(output_dir, "depth")
    mask_dir = os.path.join(output_dir, "mask")
    common_dir = os.path.join(output_dir, "common")
    ply_dir = os.path.join(output_dir, "pointcloud")
    for d in (rgb_dir, depth_dir, mask_dir, common_dir, ply_dir):
        os.makedirs(d, exist_ok=True)

    rig.save_cameras_rgb(rgb_dir, frame_id=args.frame_id)
    rig.save_cameras_depth(depth_dir, frame_id=args.frame_id)
    rig.save_cameras_mask(mask_dir)

    # 内外参
    common = {}
    for name in rig.get_cameras_name():
        K = rig.get_intrinsics_matrix(name)
        T_cam_to_world = rig.get_camera_to_world_opencv(name)
        intr_full = rig.get_intrinsics(name)  # 完整 dict(含 omni/ftheta)
        common[name] = {
            "intrinsics": K,
            "extrinsics_world": T_cam_to_world,
            "intrinsics_full": intr_full,
        }
    np.save(os.path.join(common_dir, f"{args.frame_id}.npy"), common, allow_pickle=True)

    # ---- 5. 反投影成世界系点云 ----
    logger.info("[5/5] 反投影 RGB+Depth -> 世界系点云")
    rgb_list = rig.get_cameras_rgb()
    depth_list = rig.get_cameras_distance_to_image_plane()

    all_world_pts, all_world_cols = [], []
    for name, rgb, depth in zip(rig.get_cameras_name(), rgb_list, depth_list):
        if rgb is None or depth is None:
            logger.warning(f"{name}: rgb/depth 缺失,跳过")
            continue

        width, height = rig.resolutions[name]
        mask = rig._build_mask(name, width, height)

        omni_cal = rig.calibration.get(name)
        T = rig.get_camera_to_world_opencv(name)

        if omni_cal is not None:
            logger.info(
                f"  {name}: MEI 反投影 (xi={omni_cal['xi']:.3f}, "
                f"fx={omni_cal['fx']:.1f}, fy={omni_cal['fy']:.1f})"
            )
            pts_cam, cols = depth_to_pointcloud_mei(
                rgb, depth, mask,
                omni_cal["fx"], omni_cal["fy"], omni_cal["cx"], omni_cal["cy"],
                omni_cal["xi"],
                *(list(omni_cal["distortion_coeffs"]) + [0.0, 0.0, 0.0, 0.0])[:4],
            )
        else:
            K = rig.get_intrinsics_matrix(name)
            logger.info(
                f"  {name}: pinhole 反投影 (fx={K[0,0]:.1f}, fy={K[1,1]:.1f}, "
                f"cx={K[0,2]:.1f}, cy={K[1,2]:.1f})"
            )
            pts_cam, cols = depth_to_pointcloud_pinhole(rgb, depth, mask, K)

        if pts_cam.shape[0] == 0:
            logger.warning(f"  {name}: 无有效像素,跳过")
            continue

        pts_world = transform_points(pts_cam, T)
        ply_path = os.path.join(ply_dir, f"{name}_{args.frame_id}.ply")
        save_ply_binary(ply_path, pts_world, cols)
        logger.info(f"  {name}: {pts_world.shape[0]} 点 -> {ply_path}")

        all_world_pts.append(pts_world)
        all_world_cols.append(cols)

    if all_world_pts:
        merged_pts = np.vstack(all_world_pts)
        merged_cols = np.vstack(all_world_cols)
        merged_path = os.path.join(ply_dir, f"all_cameras_{args.frame_id}.ply")
        save_ply_binary(merged_path, merged_pts, merged_cols)
        logger.info(f"合并点云: {merged_pts.shape[0]} 点 -> {merged_path}")

    logger.info("Done. 用 MeshLab / CloudCompare 打开 PLY 查看点云。")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
