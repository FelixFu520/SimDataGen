# 启动Isaac Sim应用
from isaacsim import SimulationApp
launch_config = {
    "headless": True,
    "renderer": "PathTracing",
    "rt_subframes": 8,
}
simulation_app = SimulationApp(launch_config=launch_config)

# 如果要实现 --/rtx/verifyDriverVersion/enabled=false 的效果：
import carb
settings = carb.settings.get_settings()
settings.set("/rtx/verifyDriverVersion/enabled", False)
# settings.set("/renderer/multiGpu/enabled", False)

# 启动扩展
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.gen.omap")
simulation_app.update()

# 导入必要的库
import os
import sys
import time
import random
import argparse
import numpy as np
from loguru import logger

# 导入Isaac Sim核心模块
import omni.replicator.core as rep
from isaacsim.asset.gen.omap.bindings import _omap  # 此文件不用, 但是别的文件要用, 这个要个 启动扩展 配合, 所以必须导入

# 自定义工具
from sdg_utils.usd import load_usd_file
from sdg_utils.occupancy import (
    get_mesh_paths,
    get_semantic_occupancy,
    save_semantic_occupancy_ply,
    build_mesh_id_map,
    apply_semantics_to_meshes,
)
from sdg_utils.trajectory import gen_path_3d
from sdg_utils.camera import CameraRig
from sdg_utils.transparency import (
    make_all_meshes_opaque,
    restore_meshes,
)
from sdg_utils.misc import _fmt_duration

RENDER_COUNT = 5
MAX_RETRY_ATTEMPTS = 1

# 解析命令行参数
parser = argparse.ArgumentParser()
# 环境参数
parser.add_argument("--seed", type=int, default=4)
parser.add_argument("--scene_usd_url", type=str, default=None, help='场景USD文件路径')
parser.add_argument("--camera_usd_url", type=str, default=None, help='相机USD文件路径')
parser.add_argument("--output_dir", type=str, default=None, help='输出目录')
# 生成occupancy所需参数
parser.add_argument("--occupancy_resolution", type=float, default=0.1, help='occupancy分辨率')
# 生成3D路径所需参数
parser.add_argument('--num_points', type=int, default=4, help='每条路径的路径点数量')
parser.add_argument('--num_paths', type=int, default=1, help='要生成的路径数量')
parser.add_argument('--max_angle_deviation', type=float, default=10.0, help='xy方向最大角度偏差(度),限制前进方向在前方左N度和右N度之间')
parser.add_argument('--erode_iterations', type=int, default=2, help='free positions 3D腐蚀迭代次数,越大过滤边缘越宽,设为0则不腐蚀')
parser.add_argument('--obstacle_dilate_iterations', type=int, default=2, help='occupied 障碍 3D 膨胀迭代次数(禁区),越大则 free 点离障碍越远(室内室外通用)')
parser.add_argument('--obstacle_envelope_iterations', type=int, default=10, help='取 occupied 障碍的整体外接 shape(闭运算+3D 孔洞填充)的闭运算膨胀层数,仅保留落在外接 shape 内部的 free 点(房间中心会保留,天空/旷野会剔除);设为0关闭此过滤,一般 2~5 即可封住墙缝/小开口')
parser.add_argument('--step_size_xy', type=float, default=0.3, help='3D 路径 xy 方向每步最大步长(米)')
parser.add_argument('--step_size_z', type=float, default=0.1, help='3D 路径 z 方向每步最大步长(米)')
parser.add_argument('--max_dz_per_step', type=float, default=0.1, help='3D 路径相邻两点 z 方向最大变化(米)')
parser.add_argument('--min_path_extent', type=float, default=0.5, help='连续窗口内 xyz 包围盒最大轴跨度下限(米); 小于此值视为角落蜷缩并重新生成, 设为0关闭')
parser.add_argument('--min_path_compact_window', type=int, default=60, help='检查角落蜷缩的连续点数窗口; 轨迹点数不足时用全部点')
parser.add_argument('--max_path_generation_attempts', type=int, default=1000000, help='单条轨迹因空间范围过小而重试的最大次数')
args = parser.parse_args()

# 配置日志
# 移除默认的日志处理器
logger.remove()  
# 配置日志格式
log_format = (
    "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
    "<level>{level: <8}</level> | "
    "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> | "
    "<level>{message}</level>"
)
# 添加控制台输出
logger.add(
    sys.stdout,
    format="{message}",
    level="INFO",
    colorize=True
)
# 添加文件输出
logger.add(
    args.output_dir + "/gen_data.log",
    format="{time:YYYY-MM-DD HH:mm:ss.SSS} | {level: <8} | {message}",
    level="DEBUG",
    rotation="10 MB",
    retention="7 days",
    compression="zip",
    encoding="utf-8",
    enqueue=True
)


def set_global_seed(seed: int) -> None:
    """统一设置本脚本中可能用到的随机源。

    当前代码路径中实际会影响结果的随机源:
    - Python `random`: `utils_/random_path_3d.py` 中生成路径方向、起点等;
    - NumPy `np.random`: `utils_/random_path_3d.py` 中随机采样候选点;
    - Replicator: 若后续加入 randomizer,由 `rep.set_global_seed` 控制。

    其他库:
    - SciPy ndimage 形态学操作是确定性的,不需要 seed;
    - CuPy / PyTorch 当前主流程没有用随机,但如果环境中存在也顺手设定,避免后续新增
      GPU 随机采样时忘记同步。
    """
    seed = int(seed)

    # 只对当前进程之后派生的子进程有效;Python hash 的完全确定性需在进程启动前设置。
    os.environ["PYTHONHASHSEED"] = str(seed)

    random.seed(seed)
    np.random.seed(seed)

    try:
        rep.set_global_seed(seed)
    except Exception as e:
        logger.debug(f"rep.set_global_seed 不可用或设置失败: {e}")

    try:
        import cupy as cp  # type: ignore
        cp.random.seed(seed)
    except Exception as e:
        logger.debug(f"CuPy 随机种子未设置: {e}")

    try:
        import torch  # type: ignore
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
        torch.backends.cudnn.deterministic = True
        torch.backends.cudnn.benchmark = False
    except Exception as e:
        logger.debug(f"PyTorch 随机种子未设置: {e}")

    logger.info(f"随机种子已设置: {seed}")


if __name__ == "__main__":
    logger.info(f"args: {args}")
    os.makedirs(args.output_dir, exist_ok=True)

    logger.info(f"RENDER_COUNT: {RENDER_COUNT}")
    
    overall_start = time.perf_counter()
    logger.info(f"[计时] 主流程开始")

    # 设置随机种子
    set_global_seed(args.seed)

    # ============ 步骤 1: 创建 World 和 Stage ============
    logger.info(f"[步骤1][开始] 加载场景, {args.scene_usd_url}")
    step1_start = time.perf_counter()
    world, stage = load_usd_file(args.scene_usd_url)
    world.reset()   # 重置物理世界以确保场景完全加载
    for _ in range(RENDER_COUNT):
        simulation_app.update()
    logger.info(f"[步骤1][结束] 加载场景, 耗时 {_fmt_duration(time.perf_counter() - step1_start)}")

    # ============ 步骤 2: 生成occupancy ============
    logger.info("[步骤2][开始] 生成 occupancy")
    step2_start = time.perf_counter()

    logger.info("[步骤2.1][开始] 获取物理碰撞体(Mesh)的 USD 路径列表")
    step2_1_start = time.perf_counter()
    mesh_paths = get_mesh_paths(stage)
    logger.info(f"[步骤2.1][结束] 共有 mesh(具有物理碰撞体(Mesh)的USD路径): {len(mesh_paths)}, 耗时 {_fmt_duration(time.perf_counter() - step2_1_start)}")

    logger.info("[步骤2.2][开始] 构建 mesh_path -> semantic_id 映射并写入 USD Semantics")
    step2_2_start = time.perf_counter()
    # 构建全局 mesh_path -> semantic_id 映射(occupancy 与相机语义图共用同一份 ID)
    mesh_path_to_id = build_mesh_id_map(mesh_paths)
    # 为每个 mesh 注入 USD Semantics class label,Replicator 的 semantic_segmentation
    # annotator 会据此输出语义图(包括玻璃等透明 mesh,从而便于后处理过滤透明像素)
    apply_semantics_to_meshes(stage, mesh_path_to_id)
    # 渲染前再多刷新几帧,确保 semantics schema 写入生效
    for _ in range(5):
        simulation_app.update()
    logger.info(f"[步骤2.2][结束] semantics 注入完成, 耗时 {_fmt_duration(time.perf_counter() - step2_2_start)}")

    logger.info("[步骤2.3][开始] 计算/读取带语义的 Occupancy")
    step2_3_start = time.perf_counter()
    save_occupancy_dir = os.path.join(args.output_dir, "occupancy")
    os.makedirs(save_occupancy_dir, exist_ok=True)

    save_occupancy_occupied_npy_path = os.path.join(save_occupancy_dir, "occupied_positions.npy")
    save_occupancy_free_npy_path = os.path.join(save_occupancy_dir, "free_positions.npy")
    save_occupancy_occupied_ply_path = os.path.join(save_occupancy_dir, "occupied_positions.ply")

    if not os.path.exists(save_occupancy_occupied_npy_path):
        logger.info("[步骤2.3] 未找到缓存, 实时计算 semantic occupancy")
        semantic_occupancy = get_semantic_occupancy(
            stage,
            resolution=args.occupancy_resolution,
            mesh_paths=mesh_paths,
            mesh_path_to_id=mesh_path_to_id,
        )
        occupied_data = semantic_occupancy[semantic_occupancy[:, 3] != 0]   # occupied positions
        free_data = semantic_occupancy[semantic_occupancy[:, 3] == 0]

        # save semantic_occupancy npy
        np.save(save_occupancy_occupied_npy_path, occupied_data)
        np.save(save_occupancy_free_npy_path, free_data)

        # save semantic_occupancy ply
        save_semantic_occupancy_ply(occupied_data, save_occupancy_occupied_ply_path)
    else:
        logger.info("[步骤2.3] 命中缓存, 直接加载 npy")
        occupied_data = np.load(save_occupancy_occupied_npy_path)
        free_data = np.load(save_occupancy_free_npy_path)

    logger.info(f"occupied_data: {occupied_data.shape}")
    logger.info(f"free_data: {free_data.shape}")
    logger.info(f"[步骤2.3][结束] occupancy 完成, 耗时 {_fmt_duration(time.perf_counter() - step2_3_start)}")
    logger.info(f"[步骤2][结束] 生成 occupancy, 累计耗时 {_fmt_duration(time.perf_counter() - step2_start)}")

    # ============ 步骤 3: 生成路径 ============
    logger.info("[步骤3][开始] 生成 3D 路径")
    step3_start = time.perf_counter()
    output_path_dir = os.path.join(args.output_dir, "path")
    os.makedirs(output_path_dir, exist_ok=True)
    paths_xyz = gen_path_3d(
        free_position=free_data,
        occupied_position=occupied_data,
        output_dir=output_path_dir,
        num_paths=args.num_paths,
        num_points=args.num_points,
        resolution=args.occupancy_resolution,
        erode_iterations=args.erode_iterations,
        obstacle_dilate_iterations=args.obstacle_dilate_iterations,
        obstacle_envelope_iterations=args.obstacle_envelope_iterations,
        step_size_xy=args.step_size_xy,
        step_size_z=args.step_size_z,
        max_angle_deviation=args.max_angle_deviation,
        max_dz_per_step=args.max_dz_per_step,
        min_path_extent=args.min_path_extent,
        min_path_compact_window=args.min_path_compact_window,
        max_path_generation_attempts=args.max_path_generation_attempts,
        save_filtered_ply=True,
    )
    logger.info(f"[步骤3][结束] 生成 3D 路径, 共 {len(paths_xyz)} 条, 耗时 {_fmt_duration(time.perf_counter() - step3_start)}")

    # ============ 步骤 4: 相机 ============
    logger.info("[步骤4][开始] 添加相机")
    step4_start = time.perf_counter()

    logger.info("[步骤4.1][开始] 创建 CameraRig 并绑定语义 ID")
    step4_1_start = time.perf_counter()
    # 新版 CameraRig: 接 camera USD 路径 (内参/外参已 bake 进 USD), 不再传 camera_name 字典。
    # 加载流程:
    #   1. 构造 CameraRig (此时仅 reference USD + 构建 IsaacCamera 包装器, 还未 initialize)
    #   2. world.reset()
    #   3. rig.initialize(attach_depth=True, attach_semantic=True)
    #      - attach distance_to_image_plane / semantic_segmentation / instance_id_segmentation
    camera_rig = CameraRig(
        camera_usd_path=os.path.join(os.path.dirname(__file__), "assets/cameras", f"{args.camera_usd_url}.usd"),
        world=world,
        stage=stage,
        rig_prim_path="/World/camera_rig",
    )
    world.reset()
    camera_rig.initialize(attach_depth=True, attach_semantic=True)
    camera_rig.print_all()

    # 把全局语义 ID 映射注入相机, 采集时输出的语义图像素值就是 mesh 的 semantic_id
    camera_rig.bind_semantic_id_map(mesh_path_to_id)
    # 等待渲染
    for i in range(RENDER_COUNT):
        world.step(render=True)
        # rep.orchestrator.step()
        simulation_app.update()
    logger.info(f"[步骤4.1][结束] 相机就绪, 耗时 {_fmt_duration(time.perf_counter() - step4_1_start)}")

    logger.info("[步骤4.2][开始] 保存相机有效像素 mask 与 semantic_id 映射")
    step4_2_start = time.perf_counter()
    # 保存相机有效像素 mask（LUT鱼眼的圆形视野 mask，只需保存一次）
    save_mask_dir = os.path.join(args.output_dir, "mask")
    os.makedirs(save_mask_dir, exist_ok=True)
    camera_rig.save_cameras_mask(save_mask_dir)

    # 保存全局 semantic_id <-> mesh_path 映射(只需写一次)
    save_meta_dir = os.path.join(args.output_dir, "meta")
    os.makedirs(save_meta_dir, exist_ok=True)
    camera_rig.save_semantic_id_map(save_meta_dir)
    logger.info(f"[步骤4.2][结束] mask 与 semantic_id 映射保存完成, 耗时 {_fmt_duration(time.perf_counter() - step4_2_start)}")
    logger.info(f"[步骤4][结束] 相机阶段, 累计耗时 {_fmt_duration(time.perf_counter() - step4_start)}")

    # ============ 步骤 5: 按照路径渲染, 并保存数据 ============
    # "两遍扫描"——两遍扫描,第一遍采集RGB图像,第二遍采集深度和语义图像
    # 第一遍: 保持原始材质(含透明), 遍历所有路径点完成相机位姿设置、有效性预检、RGB 采集 + 落盘、相机内外参落盘, 记录有效点。
    # 第二遍: 仅对第一遍标记为有效的点, 重设位姿 + 等渲染, 采集深度 + 语义并落盘。
    logger.info("[步骤5][开始] 按路径渲染并保存数据 (RGB / 深度+语义 两遍扫描)")
    step5_start = time.perf_counter()
    save_rgb_dir = os.path.join(args.output_dir, "rgb") # 相机RGB图像
    save_rbg_discard_dir = os.path.join(args.output_dir, "rgb_discard") # 相机RGB图像被丢弃
    save_depth_dir = os.path.join(args.output_dir, "depth") # 相机距离到图像平面图像
    save_semantic_dir = os.path.join(args.output_dir, "semantic") # 相机语义分割图(uint16, 值=mesh的semantic_id)
    save_common_dir = os.path.join(args.output_dir, "common") # 相机外参, 内参等
    os.makedirs(save_rgb_dir, exist_ok=True)
    os.makedirs(save_rbg_discard_dir, exist_ok=True)
    os.makedirs(save_depth_dir, exist_ok=True)
    os.makedirs(save_semantic_dir, exist_ok=True)
    os.makedirs(save_common_dir, exist_ok=True)

    cameras_name = camera_rig.get_cameras_name()

    total_points = sum(len(p) for p in paths_xyz)
    # 记录第一遍 RGB 阶段通过有效性预检的点, 第二遍仅对这些点采集深度+语义
    valid_points = []  # 每项: (path_idx, point_idx, x, y, z, roll, pitch, yaw)

    # ---------------- 遍 1: RGB + 内外参 (保留原始透明材质) ----------------
    logger.info("[步骤5][遍1][开始] 采集 RGB + 相机内外参 (保留透明效果)")
    pass1_start = time.perf_counter()
    point_counter = 0
    for path_idx, path_xyz in enumerate(paths_xyz):
        for point_idx, point_xyz in enumerate(path_xyz):
            point_counter += 1
            x, y, z = float(point_xyz[0]), float(point_xyz[1]), float(point_xyz[2])
            roll, pitch, yaw = 0, 0, 0

            point_start = time.perf_counter()
            logger.info(f"\n====> [遍1/RGB] 生成 {path_idx:04d}_{point_idx:04d}(路径_路径点) 数据 [{point_counter}/{total_points}] <====")

            # 有时相机拍摄出来的图像是黑色的, 进入了角落, 所以需要尝试多次, 直到获取到正常的图像
            # 每次尝试在yaw旋转45度, 直到获取到正常的图像, 如果尝试了8次, 则认为相机进入了角落, 跳过这个点
            yaw_increment = 45
            retry_count = 0
            valid_image = False  # 此camera rig是否获取到全部正常的图像
            validate_start = time.perf_counter()
            while retry_count < MAX_RETRY_ATTEMPTS:
                attempt_start = time.perf_counter()
                # 设置相机位置 (新 API 名 set_pose)
                camera_rig.set_pose(x, y, z, roll, pitch, yaw)

                # 等待渲染
                render_wait_start = time.perf_counter()
                for i in range(RENDER_COUNT):
                    world.step(render=True)
                    # rep.orchestrator.step()
                    simulation_app.update()
                logger.info(f"[步骤5][有效性预检-渲染等待] 耗时 {_fmt_duration(time.perf_counter() - render_wait_start)}")

                # 获取RGB图像
                grab_start = time.perf_counter()
                cameras_rgb = camera_rig.get_cameras_rgb()
                # 保存RGB图像全部数据 (含有效性预检失败的, 命名同正式落盘以便对照排查)
                camera_rig.save_cameras_rgb(save_rbg_discard_dir, path_idx=path_idx, point_idx=point_idx)
                logger.info(f"[步骤5][有效性预检-取RGB+落盘] 耗时 {_fmt_duration(time.perf_counter() - grab_start)}")

                # 用于判断相机是否在角落 是否保留
                valid_image_count = 0
                for camera_rgb in cameras_rgb:
                    # 排除整体偏暗的图片、排除色差小的图片
                    black_pixel_threshold = 5  # 整体偏暗的阈值
                    color_difference_threshold = 5  # 色差小的阈值
                    black_pixel_ratio = 0.8  # 整体偏暗的像素比例
                    black_pixel_count = np.sum(camera_rgb < black_pixel_threshold)
                    max_value = np.max(camera_rgb)  # 最大值
                    min_value = np.min(camera_rgb)  # 最小值
                    if black_pixel_count > black_pixel_ratio * camera_rgb.size or max_value - min_value < color_difference_threshold:
                        continue
                    else:
                        valid_image_count += 1

                # 根据camera rig中相机成像情况来判断是否旋转相机
                if valid_image_count < len(cameras_rgb):
                    logger.warning(f"采集 {path_idx:04d}-{point_idx:04d}-({x},{y},{z})-({roll},{pitch},{yaw}) 数据失败!!!!, 继续旋转相机, 格式:path-point-(xyz)-(roll,pitch,yaw) [本次尝试耗时 {_fmt_duration(time.perf_counter() - attempt_start)}]")
                    retry_count += 1
                    yaw += yaw_increment
                    continue
                else:
                    valid_image = True
                    logger.info(f"[步骤5][有效性预检] 单次尝试通过, 耗时 {_fmt_duration(time.perf_counter() - attempt_start)}")
                    break
            logger.info(f"[步骤5][有效性预检] 累计耗时 {_fmt_duration(time.perf_counter() - validate_start)}, 重试 {retry_count} 次")

            # 判断是否使用此点采集数据
            if not valid_image:
                logger.warning(f"采集 {path_idx:04d}-{point_idx:04d}-({x},{y},{z}) 数据失败!!!!, 重试 {retry_count} 次未成功, 跳过此点, 格式:path-point-(xyz) [本点耗时 {_fmt_duration(time.perf_counter() - point_start)}]")
                continue
            else:
                logger.info(f"采集{path_idx:04d}-{point_idx:04d}-({x},{y},{z})) 数据成功, 重试 {retry_count} 次成功, 格式:path-point-(xyz)")

            # ---- RGB(保留透明效果) ----
            # 复用上面有效性预检时已经渲染稳定的状态, 直接落盘保存即可 (不再重复 get/wait)
            logger.info("[步骤5][开始] 采集 RGB (保留透明)")
            phase1_start = time.perf_counter()
            camera_rig.save_cameras_rgb(save_rgb_dir, path_idx=path_idx, point_idx=point_idx)
            logger.info(f"[步骤5][结束] 采集 RGB, 耗时 {_fmt_duration(time.perf_counter() - phase1_start)}")

            # ---- 保存相机外参/内参 ----
            # 外参依赖于当前相机位姿 (此时位姿就是该点采集 RGB 时的位姿),
            # 所以放在遍 1 内、深度阶段重设位姿之前完成才是最稳妥的。
            #
            # 新 CameraRig API:
            #   - get_intrinsics_matrix(name): 3x3 像素内参 K(LUT 鱼眼相机用 bake 进 USD 的真实标定)
            #   - get_camera_to_world_opencv(name): 4x4, OpenCV 相机系 -> 世界系
            #   - get_transform_between_cameras_opencv(a, b): 4x4, OpenCV 系下 cam_a -> cam_b
            logger.info("[步骤5][开始] 保存相机内外参")
            phase4_start = time.perf_counter()
            common_dict = {}
            for camera_name in cameras_name:
                common_dict[camera_name] = {}

                # 内参 (像素 K)
                common_dict[camera_name]["intrinsics"] = camera_rig.get_intrinsics_matrix(camera_name)

                # 外参 (世界坐标系, OpenCV cam -> world)
                common_dict[camera_name]["extrinsics_world"] = camera_rig.get_camera_to_world_opencv(camera_name)

                # 完整内参 dict (含 omni/ftheta 标定, project_cloud 等下游用)
                common_dict[camera_name]["intrinsics_full"] = camera_rig.get_intrinsics(camera_name)

                # 外参 (相机间, OpenCV 坐标系)
                common_dict[camera_name]["extrinsics_camera"] = {}
                for camera_anthor in cameras_name:
                    if camera_anthor == camera_name:
                        continue
                    common_dict[camera_name]["extrinsics_camera"][camera_anthor] = (
                        camera_rig.get_transform_between_cameras_opencv(camera_name, camera_anthor)
                    )
            np.save(os.path.join(save_common_dir, f"{path_idx:04d}_{point_idx:04d}.npy"), common_dict, allow_pickle=True)
            logger.info(f"[步骤5][结束] 保存内外参, 耗时 {_fmt_duration(time.perf_counter() - phase4_start)}")

            valid_points.append((path_idx, point_idx, x, y, z, roll, pitch, yaw))

            point_elapsed = time.perf_counter() - point_start
            avg_per_point = (time.perf_counter() - pass1_start) / max(point_counter, 1)
            remaining = max(total_points - point_counter, 0) * avg_per_point
            logger.info(f"[步骤5][遍1/路径点结束] {path_idx:04d}_{point_idx:04d} 总耗时 {_fmt_duration(point_elapsed)}; 进度 {point_counter}/{total_points}, 平均/点 {_fmt_duration(avg_per_point)}, 预计剩余 {_fmt_duration(remaining)}")
    logger.info(f"[步骤5][遍1][结束] RGB + 内外参 完成, 有效点 {len(valid_points)}/{total_points}, 累计耗时 {_fmt_duration(time.perf_counter() - pass1_start)}")

    # ---------------- 全局切不透明 (只做 1 次) ----------------
    logger.info("[步骤5][全局切不透明][开始] make_all_meshes_opaque + 渲染等待")
    global_opaque_start = time.perf_counter()
    override = make_all_meshes_opaque(stage, mesh_paths)
    for _ in range(RENDER_COUNT):
        world.step(render=True)
        simulation_app.update()
    logger.info(f"[步骤5][全局切不透明][结束] 耗时 {_fmt_duration(time.perf_counter() - global_opaque_start)}")

    # ---------------- 遍 2: 深度 + 语义 (全场景不透明) ----------------
    logger.info(f"[步骤5][遍2][开始] 采集 深度 + 语义, 共 {len(valid_points)} 个有效点")
    pass2_start = time.perf_counter()
    try:
        for idx, (path_idx, point_idx, x, y, z, roll, pitch, yaw) in enumerate(valid_points, start=1):
            point_start = time.perf_counter()
            logger.info(f"\n====> [遍2/深度+语义] {path_idx:04d}_{point_idx:04d} [{idx}/{len(valid_points)}] <====")

            # 重设相机位姿 (用遍 1 里通过有效性预检时使用的最终 yaw, 保证两遍像素对齐)
            camera_rig.set_pose(x, y, z, roll, pitch, yaw)

            # 相机移动后渲染等待
            render_wait_start = time.perf_counter()
            for _ in range(RENDER_COUNT):
                world.step(render=True)
                simulation_app.update()
            logger.info(f"[步骤5][遍2][渲染等待] 耗时 {_fmt_duration(time.perf_counter() - render_wait_start)}")

            phase2_depth_start = time.perf_counter()
            # 新 API: save_cameras_depth 同时落 .npy(原始 float) + .png(归一化可视化)
            camera_rig.save_cameras_depth(save_depth_dir, path_idx=path_idx, point_idx=point_idx)
            logger.info(f"[步骤5][遍2][采集深度] 耗时 {_fmt_duration(time.perf_counter() - phase2_depth_start)}")

            phase2_sem_start = time.perf_counter()
            # 语义分割图(uint16, 像素值=mesh的semantic_id, 0=背景/天空)
            # 用途: 过滤天空/无穷远(semantic_id==0)、按 mesh ID 屏蔽训练黑名单。
            camera_rig.save_cameras_semantic(save_semantic_dir, path_idx=path_idx, point_idx=point_idx)
            logger.info(f"[步骤5][遍2][采集语义] 耗时 {_fmt_duration(time.perf_counter() - phase2_sem_start)}")

            point_elapsed = time.perf_counter() - point_start
            avg_per_point = (time.perf_counter() - pass2_start) / max(idx, 1)
            remaining = max(len(valid_points) - idx, 0) * avg_per_point
            logger.info(f"[步骤5][遍2/路径点结束] {path_idx:04d}_{point_idx:04d} 总耗时 {_fmt_duration(point_elapsed)}; 进度 {idx}/{len(valid_points)}, 平均/点 {_fmt_duration(avg_per_point)}, 预计剩余 {_fmt_duration(remaining)}")
    finally:
        # ---------------- 全局恢复材质 (只做 1 次) ----------------
        # 用 try/finally 确保即使遍 2 中途异常退出, 也能把场景材质恢复到原始状态,
        # 避免下次复用 stage 时 RGB 被污染。
        logger.info("[步骤5][全局恢复材质][开始] restore_meshes + 渲染等待")
        global_restore_start = time.perf_counter()
        restore_meshes(stage, override)
        for _ in range(RENDER_COUNT):
            world.step(render=True)
            simulation_app.update()
        logger.info(f"[步骤5][全局恢复材质][结束] 耗时 {_fmt_duration(time.perf_counter() - global_restore_start)}")
    logger.info(f"[步骤5][遍2][结束] 深度+语义 完成, 累计耗时 {_fmt_duration(time.perf_counter() - pass2_start)}")

    logger.info(f"[步骤5][结束] 渲染并保存数据完成, 累计耗时 {_fmt_duration(time.perf_counter() - step5_start)}")

    logger.info(f"数据生成完成, 保存路径: {args.output_dir}")
    logger.info(f"[计时] 主流程总耗时 {_fmt_duration(time.perf_counter() - overall_start)}")

    # 关闭软件
    simulation_app.close()