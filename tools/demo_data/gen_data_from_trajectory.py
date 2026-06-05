# 根据手动录制的 CameraRig 轨迹 (rig_poses_*.npy) 在场景中采数。
# 流程与 gen_data.py 的步骤 4–5 一致，跳过 occupancy 与自动路径生成。
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

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.asset.gen.omap")
simulation_app.update()

import os
import re
import shutil
import sys
import time
import random
import argparse
from typing import List, Optional, Tuple

import numpy as np
from loguru import logger

import omni.replicator.core as rep
from isaacsim.asset.gen.omap.bindings import _omap  # noqa: F401

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from sdg_utils.usd import load_usd_file
from sdg_utils.occupancy import (
    get_mesh_paths,
    build_mesh_id_map,
    apply_semantics_to_meshes,
)
from sdg_utils.camera import CameraRig
from sdg_utils.transparency import make_all_meshes_opaque, restore_meshes
from sdg_utils.misc import _fmt_duration, resolve_camera_usd_path

RENDER_COUNT = 5

parser = argparse.ArgumentParser(
    description=(
        "按录制轨迹 (rig_poses_*.npy) 采集 RGB / 深度 / 语义；"
        "输出含 path/paths.npy，与 gen_data.py / batch_vis_to_mcap.sh 目录布局一致。"
    )
)
parser.add_argument("--seed", type=int, default=4)
parser.add_argument("--scene_usd_url", type=str, required=True, help="场景 USD，须与录制轨迹时一致")
parser.add_argument(
    "--camera_usd_url",
    type=str,
    required=True,
    help="相机 USD，须与录制轨迹时一致",
)
parser.add_argument("--output_dir", type=str, required=True, help="采数输出目录")
parser.add_argument(
    "--trajectory_dir",
    type=str,
    required=True,
    help="录制轨迹目录，内含 rig_poses_XXXX.npy",
)
parser.add_argument(
    "--trajectory_tags",
    type=str,
    default=None,
    help='只采指定段，逗号分隔索引，如 "1" 或 "0,1"；默认采目录下全部 rig_poses_*.npy',
)
parser.add_argument(
    "--point_stride",
    type=int,
    default=1,
    help="轨迹点下采样步长，1 表示全量采数",
)
parser.add_argument("--point_start", type=int, default=0, help="起始点索引（含）")
parser.add_argument(
    "--point_end",
    type=int,
    default=None,
    help="结束点索引（不含）；默认到轨迹末尾",
)
parser.add_argument(
    "--max_retry_attempts",
    type=int,
    default=0,
    help="RGB 有效性失败时绕 yaw 重试次数；手动轨迹建议 0，避免偏离录制朝向",
)
parser.add_argument(
    "--yaw_increment",
    type=float,
    default=45.0,
    help="重试时每次增加的 yaw（度），仅 max_retry_attempts>0 时生效",
)
args = parser.parse_args()

logger.remove()
logger.add(sys.stdout, format="{message}", level="INFO", colorize=True)
os.makedirs(args.output_dir, exist_ok=True)
logger.add(
    os.path.join(args.output_dir, "gen_data_from_trajectory.log"),
    format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {level: <8} | {message}",
    level="DEBUG",
    rotation="10 MB",
    retention="7 days",
    compression="zip",
    encoding="utf-8",
    enqueue=True,
)

RIG_POSES_RE = re.compile(r"^rig_poses_(\d+)\.npy$")


def set_global_seed(seed: int) -> None:
    seed = int(seed)
    os.environ["PYTHONHASHSEED"] = str(seed)
    random.seed(seed)
    np.random.seed(seed)
    try:
        rep.set_global_seed(seed)
    except Exception as e:
        logger.debug(f"rep.set_global_seed 不可用: {e}")
    logger.info(f"随机种子已设置: {seed}")


def parse_trajectory_tags(tags_str: Optional[str]) -> Optional[List[int]]:
    if not tags_str:
        return None
    return [int(x.strip()) for x in tags_str.split(",") if x.strip() != ""]


def discover_rig_pose_files(trajectory_dir: str, tags: Optional[List[int]]) -> List[Tuple[int, str]]:
    found: List[Tuple[int, str]] = []
    for name in sorted(os.listdir(trajectory_dir)):
        m = RIG_POSES_RE.match(name)
        if not m:
            continue
        idx = int(m.group(1))
        if tags is not None and idx not in tags:
            continue
        found.append((idx, os.path.join(trajectory_dir, name)))
    if not found:
        hint = (
            f"未在 {trajectory_dir} 找到 rig_poses_*.npy"
            + (f"（筛选 tags={tags}）" if tags is not None else "")
        )
        raise FileNotFoundError(hint)
    return found


def load_and_slice_poses(path: str) -> np.ndarray:
    poses = np.load(path)
    if poses.ndim != 2 or poses.shape[1] != 6:
        raise ValueError(f"rig_poses 形状应为 (N, 6)，实际: {poses.shape} @ {path}")
    end = args.point_end if args.point_end is not None else len(poses)
    sliced = poses[args.point_start : end : args.point_stride]
    if len(sliced) == 0:
        raise ValueError(f"下采样后无轨迹点: {path} start={args.point_start} end={end} stride={args.point_stride}")
    return sliced.astype(np.float64)


def validate_rgb(cameras_rgb) -> bool:
    black_pixel_threshold = 5
    color_difference_threshold = 5
    black_pixel_ratio = 0.8
    valid_count = 0
    for camera_rgb in cameras_rgb:
        black_pixel_count = np.sum(camera_rgb < black_pixel_threshold)
        max_value = np.max(camera_rgb)
        min_value = np.min(camera_rgb)
        if black_pixel_count > black_pixel_ratio * camera_rgb.size or max_value - min_value < color_difference_threshold:
            continue
        valid_count += 1
    return valid_count >= len(cameras_rgb)


def copy_trajectory_sources(trajectory_dir: str, entries: List[Tuple[int, str]]) -> None:
    out_dir = os.path.join(args.output_dir, "trajectory")
    os.makedirs(out_dir, exist_ok=True)
    for tag, rig_path in entries:
        tag_s = f"{tag:04d}"
        shutil.copy2(rig_path, os.path.join(out_dir, f"rig_poses_{tag_s}.npy"))
        paths_src = os.path.join(trajectory_dir, f"paths_{tag_s}.npy")
        if os.path.isfile(paths_src):
            shutil.copy2(paths_src, os.path.join(out_dir, f"paths_{tag_s}.npy"))
        meta_src = os.path.join(trajectory_dir, f"trajectory_{tag_s}.json")
        if os.path.isfile(meta_src):
            shutil.copy2(meta_src, os.path.join(out_dir, f"trajectory_{tag_s}.json"))


def build_paths_npy(trajectories: List[Tuple[int, np.ndarray]]) -> np.ndarray:
    """构建 path/paths.npy: (num_paths, num_points, 3)，与 gen_data.py / batch_vis_to_mcap 一致。"""
    if not trajectories:
        raise ValueError("无轨迹段，无法生成 paths.npy")
    max_n = max(len(poses) for _, poses in trajectories)
    paths_arr = np.zeros((len(trajectories), max_n, 3), dtype=np.float64)
    for row, (_, poses) in enumerate(trajectories):
        n = len(poses)
        paths_arr[row, :n, :] = poses[:, :3]
        if n < max_n:
            paths_arr[row, n:, :] = poses[-1, :3]
    return paths_arr


def write_path_outputs(trajectories: List[Tuple[int, np.ndarray]]) -> str:
    """写入 path/paths.npy，供 project_cloud / path_vis_to_mcap / batch_vis_to_mcap 使用。"""
    path_dir = os.path.join(args.output_dir, "path")
    os.makedirs(path_dir, exist_ok=True)
    paths_arr = build_paths_npy(trajectories)
    paths_npy = os.path.join(path_dir, "paths.npy")
    np.save(paths_npy, paths_arr)
    logger.info(f"已写入 {paths_npy}, 形状 {paths_arr.shape}")
    return paths_npy


if __name__ == "__main__":
    logger.info(f"args: {args}")
    logger.info(f"RENDER_COUNT: {RENDER_COUNT}")

    overall_start = time.perf_counter()
    set_global_seed(args.seed)

    tags_filter = parse_trajectory_tags(args.trajectory_tags)
    rig_entries = discover_rig_pose_files(args.trajectory_dir, tags_filter)
    trajectories: List[Tuple[int, np.ndarray]] = []
    for tag, rig_path in rig_entries:
        poses = load_and_slice_poses(rig_path)
        trajectories.append((tag, poses))
        logger.info(f"轨迹段 {tag:04d}: {rig_path} -> {len(poses)} 点")

    copy_trajectory_sources(args.trajectory_dir, rig_entries)
    write_path_outputs(trajectories)

    # ============ 步骤 1: 加载场景 ============
    logger.info(f"[步骤1][开始] 加载场景 {args.scene_usd_url}")
    step1_start = time.perf_counter()
    world, stage = load_usd_file(args.scene_usd_url)
    world.reset()
    for _ in range(RENDER_COUNT):
        simulation_app.update()
    logger.info(f"[步骤1][结束] 耗时 {_fmt_duration(time.perf_counter() - step1_start)}")

    # ============ 步骤 2: 语义（采语义图所需，不算 occupancy） ============
    logger.info("[步骤2][开始] mesh 语义注入")
    step2_start = time.perf_counter()
    mesh_paths = get_mesh_paths(stage)
    mesh_path_to_id = build_mesh_id_map(mesh_paths)
    apply_semantics_to_meshes(stage, mesh_path_to_id)
    for _ in range(5):
        simulation_app.update()
    logger.info(f"[步骤2][结束] mesh={len(mesh_paths)}, 耗时 {_fmt_duration(time.perf_counter() - step2_start)}")

    # ============ 步骤 3: 相机 ============
    logger.info("[步骤3][开始] 创建 CameraRig")
    step3_start = time.perf_counter()
    camera_usd_path = resolve_camera_usd_path(args.camera_usd_url)
    logger.info(f"相机 USD: {camera_usd_path}")
    camera_rig = CameraRig(
        camera_usd_path=camera_usd_path,
        world=world,
        stage=stage,
        rig_prim_path="/World/camera_rig",
    )
    world.reset()
    camera_rig.initialize(attach_depth=True, attach_semantic=True)
    camera_rig.print_all()
    camera_rig.bind_semantic_id_map(mesh_path_to_id)
    for _ in range(RENDER_COUNT):
        world.step(render=True)
        simulation_app.update()

    save_mask_dir = os.path.join(args.output_dir, "mask")
    os.makedirs(save_mask_dir, exist_ok=True)
    camera_rig.save_cameras_mask(save_mask_dir)
    save_meta_dir = os.path.join(args.output_dir, "meta")
    os.makedirs(save_meta_dir, exist_ok=True)
    camera_rig.save_semantic_id_map(save_meta_dir)
    logger.info(f"[步骤3][结束] 耗时 {_fmt_duration(time.perf_counter() - step3_start)}")

    # ============ 步骤 4: 按轨迹渲染 ============
    logger.info("[步骤4][开始] 按录制轨迹渲染 (RGB / 深度+语义 两遍)")
    step4_start = time.perf_counter()
    save_rgb_dir = os.path.join(args.output_dir, "rgb")
    save_rgb_discard_dir = os.path.join(args.output_dir, "rgb_discard")
    save_depth_dir = os.path.join(args.output_dir, "depth")
    save_semantic_dir = os.path.join(args.output_dir, "semantic")
    save_common_dir = os.path.join(args.output_dir, "common")
    for d in (save_rgb_dir, save_rgb_discard_dir, save_depth_dir, save_semantic_dir, save_common_dir):
        os.makedirs(d, exist_ok=True)

    cameras_name = camera_rig.get_cameras_name()
    total_points = sum(len(p) for _, p in trajectories)
    # path_idx: 0 起标，与 gen_data.py / batch_vis_to_mcap 帧命名一致；path_tag 为源 rig_poses_XXXX 编号
    valid_points: List[Tuple[int, int, float, float, float, float, float, float]] = []

    logger.info("[步骤4][遍1][开始] RGB + 内外参")
    pass1_start = time.perf_counter()
    point_counter = 0
    for path_idx, (path_tag, rig_poses) in enumerate(trajectories):
        for point_idx, pose in enumerate(rig_poses):
            point_counter += 1
            x, y, z, roll, pitch, yaw = [float(v) for v in pose]
            point_start = time.perf_counter()
            logger.info(
                f"\n====> [遍1/RGB] {path_idx:04d}_{point_idx:04d} "
                f"(源 tag={path_tag:04d}) [{point_counter}/{total_points}] "
                f"pose=({x:.3f},{y:.3f},{z:.3f},{roll:.1f},{pitch:.1f},{yaw:.1f}) <===="
            )

            retry_count = 0
            valid_image = False
            cur_roll, cur_pitch, cur_yaw = roll, pitch, yaw
            max_attempts = max(1, args.max_retry_attempts + 1)

            while retry_count < max_attempts:
                camera_rig.set_pose(x, y, z, cur_roll, cur_pitch, cur_yaw)
                for _ in range(RENDER_COUNT):
                    world.step(render=True)
                    simulation_app.update()

                cameras_rgb = camera_rig.get_cameras_rgb()
                camera_rig.save_cameras_rgb(
                    save_rgb_discard_dir, path_idx=path_idx, point_idx=point_idx
                )

                if validate_rgb(cameras_rgb):
                    valid_image = True
                    break

                retry_count += 1
                if retry_count < max_attempts:
                    logger.warning(
                        f"有效性预检失败 {path_idx:04d}-{point_idx:04d} (源 tag={path_tag:04d}), "
                        f"yaw {cur_yaw:.1f} -> {cur_yaw + args.yaw_increment:.1f}"
                    )
                    cur_yaw += args.yaw_increment
                else:
                    break

            if not valid_image:
                logger.warning(
                    f"跳过 {path_idx:04d}-{point_idx:04d} (源 tag={path_tag:04d}), "
                    f"预检失败 (重试 {retry_count} 次)"
                )
                continue

            camera_rig.save_cameras_rgb(save_rgb_dir, path_idx=path_idx, point_idx=point_idx)

            common_dict = {}
            for camera_name in cameras_name:
                common_dict[camera_name] = {
                    "intrinsics": camera_rig.get_intrinsics_matrix(camera_name),
                    "extrinsics_world": camera_rig.get_camera_to_world_opencv(camera_name),
                    "intrinsics_full": camera_rig.get_intrinsics(camera_name),
                    "extrinsics_camera": {
                        other: camera_rig.get_transform_between_cameras_opencv(camera_name, other)
                        for other in cameras_name
                        if other != camera_name
                    },
                }
            np.save(
                os.path.join(save_common_dir, f"{path_idx:04d}_{point_idx:04d}.npy"),
                common_dict,
                allow_pickle=True,
            )
            valid_points.append(
                (path_idx, point_idx, x, y, z, cur_roll, cur_pitch, cur_yaw)
            )
            logger.info(
                f"[步骤4][遍1] 点结束 耗时 {_fmt_duration(time.perf_counter() - point_start)}"
            )

    logger.info(
        f"[步骤4][遍1][结束] 有效点 {len(valid_points)}/{total_points}, "
        f"耗时 {_fmt_duration(time.perf_counter() - pass1_start)}"
    )

    logger.info("[步骤4][全局不透明][开始]")
    opaque_start = time.perf_counter()
    override = make_all_meshes_opaque(stage, mesh_paths)
    for _ in range(RENDER_COUNT):
        world.step(render=True)
        simulation_app.update()
    logger.info(f"[步骤4][全局不透明][结束] 耗时 {_fmt_duration(time.perf_counter() - opaque_start)}")

    logger.info(f"[步骤4][遍2][开始] 深度+语义, {len(valid_points)} 点")
    pass2_start = time.perf_counter()
    try:
        for idx, (path_idx, point_idx, x, y, z, roll, pitch, yaw) in enumerate(
            valid_points, start=1
        ):
            logger.info(
                f"\n====> [遍2] {path_idx:04d}_{point_idx:04d} [{idx}/{len(valid_points)}] <===="
            )
            camera_rig.set_pose(x, y, z, roll, pitch, yaw)
            for _ in range(RENDER_COUNT):
                world.step(render=True)
                simulation_app.update()
            camera_rig.save_cameras_depth(
                save_depth_dir, path_idx=path_idx, point_idx=point_idx
            )
            camera_rig.save_cameras_semantic(
                save_semantic_dir, path_idx=path_idx, point_idx=point_idx
            )
    finally:
        logger.info("[步骤4][恢复材质][开始]")
        restore_start = time.perf_counter()
        restore_meshes(stage, override)
        for _ in range(RENDER_COUNT):
            world.step(render=True)
            simulation_app.update()
        logger.info(f"[步骤4][恢复材质][结束] 耗时 {_fmt_duration(time.perf_counter() - restore_start)}")

    logger.info(
        f"[步骤4][结束] 总耗时 {_fmt_duration(time.perf_counter() - step4_start)}; "
        f"输出: {args.output_dir}"
    )
    logger.info(f"[计时] 主流程 {_fmt_duration(time.perf_counter() - overall_start)}")
    simulation_app.close()
