#!/usr/bin/env python3
"""按 vis 帧将点云写入 MCAP,供 Foxglove 播放。

输入:
  - path/paths.npy: (num_paths, num_points, 3) 轨迹路点
  - vis/{CAM}_{lut|mei|pinhole}_world_{path:04d}_{point:04d}.ply: 各相机世界系点云 (运行时按 --cameras 合并)
  - occupancy/occupied_positions.npy: 占据栅格 (相机拍不到的静态场景)

输出目录 (默认): <workdir>/mcap/
  每帧一个 MCAP: <name>_path_{path:04d}_pt_{point:04d}.mcap
  全局时间戳按帧序递增 (1/fps 间隔), 便于 Foxglove 同时打开多个文件连续播放。

输出 topic (protobuf,Foxglove 可直接解析):
  - /tf: 静态坐标系 foxglove.FrameTransform
  - /sim/occupancy: 占据场景点云 (每帧 MCAP 均写入,时间戳与当帧一致,灰白色)
  - /sim/pointcloud: 当前帧 foxglove.PointCloud
  - /sim/path: 对应轨迹 foxglove.PosesInFrame
  - /sim/map (仅 --accumulate): 额外写入 <name>_map.mcap

用法:
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/intime_home_000_100_1_30
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/intime_home_000_100_1_30 --downsample 20
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/intime_home_000_100_1_30 --output output/mcaps
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/foo --cameras CAM_A CAM_B CAM_C CAM_D
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/foo --ply-suffix mei_world

Foxglove: 打开 mcap/ 下全部文件; 固定参考系选「world」; 点云 Color mode 选「RGBA (separate fields)」。
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import time
from datetime import datetime, timezone

import numpy as np

try:
    from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
    from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField
    from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud
    from foxglove_schemas_protobuf.PosesInFrame_pb2 import PosesInFrame
    from google.protobuf.timestamp_pb2 import Timestamp
    from mcap_protobuf.writer import Writer as ProtobufWriter
except ImportError as exc:
    raise SystemExit(
        "缺少依赖,请执行:\n"
        "  ./app/python.sh -m pip install mcap mcap-protobuf-support foxglove-schemas-protobuf"
    ) from exc

PLY_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('r', 'u1'), ('g', 'u1'), ('b', 'u1'),
])
PACK_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('r', 'u1'), ('g', 'u1'), ('b', 'u1'), ('a', 'u1'),
])
KEY_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])

PER_CAM_PLY_PATTERN = re.compile(
    r'^(?P<cam>.+)_(?P<suffix>lut_world|mei_world|pinhole_world)_(?P<path>\d{4})_(?P<point>\d{4})\.ply$'
)
PLY_SUFFIXES = ("lut_world", "mei_world", "pinhole_world")
DEFAULT_CAMERAS = ("CAM_A", "CAM_B", "CAM_C", "CAM_D", "CAM_Front", "CAM_Back")

POINT_STRIDE = PACK_DTYPE.itemsize
DEFAULT_FRAME_ID = "world"
OCCUPANCY_GRAY_RGB = (210, 210, 210)
OCCUPANCY_NPY = "occupied_positions.npy"


def read_ply_binary(path: str, stride: int = 1) -> tuple[np.ndarray, int]:
    with open(path, 'rb') as f:
        n = None
        while True:
            line = f.readline().decode('ascii').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            if line == 'end_header':
                break
        if n is None:
            raise ValueError(f"no vertex count in {path}")
        offset = f.tell()
    if stride <= 1:
        data = np.memmap(path, dtype=PLY_DTYPE, mode='r', offset=offset, shape=(n,)).copy()
    else:
        mm = np.memmap(path, dtype=PLY_DTYPE, mode='r', offset=offset, shape=(n,))
        data = np.asarray(mm[::stride])
    return data, n


def cap_points(data: np.ndarray, max_points: int | None) -> np.ndarray:
    if max_points is not None and len(data) > max_points:
        idx = np.linspace(0, len(data) - 1, max_points, dtype=np.int64)
        data = data[idx]
    return data


def dedup_xyz(data: np.ndarray) -> np.ndarray:
    keys = np.empty(len(data), dtype=KEY_DTYPE)
    keys['x'] = data['x']
    keys['y'] = data['y']
    keys['z'] = data['z']
    _, unique_idx = np.unique(keys, return_index=True)
    unique_idx.sort()
    return data[unique_idx]


def pack_pointcloud_bytes(data: np.ndarray) -> bytes:
    packed = np.empty(len(data), dtype=PACK_DTYPE)
    packed['x'] = data['x']
    packed['y'] = data['y']
    packed['z'] = data['z']
    packed['r'] = data['r']
    packed['g'] = data['g']
    packed['b'] = data['b']
    packed['a'] = 255
    return packed.tobytes()


def to_timestamp(stamp: datetime) -> Timestamp:
    ts = Timestamp()
    ts.seconds = int(stamp.timestamp())
    ts.nanos = int((stamp.timestamp() % 1) * 1e9)
    return ts


def make_pointcloud_proto(
    data: np.ndarray,
    stamp: datetime,
    frame_id: str,
) -> PointCloud:
    msg = PointCloud()
    msg.timestamp.CopyFrom(to_timestamp(stamp))
    msg.frame_id = frame_id
    msg.pose.position.x = 0.0
    msg.pose.position.y = 0.0
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    msg.point_stride = POINT_STRIDE

    # Foxglove「RGBA (separate fields)」要求 red/green/blue/alpha 四字段齐全
    field_specs = [
        ("x", 0, PackedElementField.FLOAT32),
        ("y", 4, PackedElementField.FLOAT32),
        ("z", 8, PackedElementField.FLOAT32),
        ("red", 12, PackedElementField.UINT8),
        ("green", 13, PackedElementField.UINT8),
        ("blue", 14, PackedElementField.UINT8),
        ("alpha", 15, PackedElementField.UINT8),
    ]
    for name, offset, numeric_type in field_specs:
        field = msg.fields.add()
        field.name = name
        field.offset = offset
        field.type = numeric_type

    msg.data = pack_pointcloud_bytes(data)
    return msg


def make_frame_transform_proto(stamp: datetime, frame_id: str) -> FrameTransform:
    """发布根坐标系,让 Foxglove 固定参考系下拉框出现 frame_id。"""
    msg = FrameTransform()
    msg.timestamp.CopyFrom(to_timestamp(stamp))
    msg.parent_frame_id = ""
    msg.child_frame_id = frame_id
    msg.translation.x = 0.0
    msg.translation.y = 0.0
    msg.translation.z = 0.0
    msg.rotation.x = 0.0
    msg.rotation.y = 0.0
    msg.rotation.z = 0.0
    msg.rotation.w = 1.0
    return msg


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def make_poses_in_frame_proto(
    path_xyz: np.ndarray,
    stamp: datetime,
    frame_id: str,
) -> PosesInFrame:
    msg = PosesInFrame()
    msg.timestamp.CopyFrom(to_timestamp(stamp))
    msg.frame_id = frame_id
    n = len(path_xyz)
    for i in range(n):
        pose = msg.poses.add()
        pose.position.x = float(path_xyz[i, 0])
        pose.position.y = float(path_xyz[i, 1])
        pose.position.z = float(path_xyz[i, 2])
        if i < n - 1:
            dx = float(path_xyz[i + 1, 0] - path_xyz[i, 0])
            dy = float(path_xyz[i + 1, 1] - path_xyz[i, 1])
        elif i > 0:
            dx = float(path_xyz[i, 0] - path_xyz[i - 1, 0])
            dy = float(path_xyz[i, 1] - path_xyz[i - 1, 1])
        else:
            dx, dy = 1.0, 0.0
        yaw = math.atan2(dy, dx) if (dx != 0.0 or dy != 0.0) else 0.0
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
    return msg


def load_occupied_scene(
    npy_path: str,
    stride: int,
    max_points: int | None,
    gray: tuple[int, int, int] = OCCUPANCY_GRAY_RGB,
) -> tuple[np.ndarray, int]:
    """占据栅格 xyz -> 灰白色点云 (代表相机未直接观测到的静态场景)。"""
    raw = np.load(npy_path)
    if raw.ndim != 2 or raw.shape[1] < 3:
        raise ValueError(f"occupied_positions 形状异常: {raw.shape}")
    n_raw = raw.shape[0]
    xyz = raw[:, :3].astype(np.float32, copy=False)
    if stride > 1:
        xyz = xyz[::stride]
    n = len(xyz)
    data = np.empty(n, dtype=PLY_DTYPE)
    data['x'] = xyz[:, 0]
    data['y'] = xyz[:, 1]
    data['z'] = xyz[:, 2]
    data['r'] = gray[0]
    data['g'] = gray[1]
    data['b'] = gray[2]
    return cap_points(data, max_points), n_raw


def frame_stem(path_idx: int, point_idx: int) -> str:
    return f"{path_idx:04d}_{point_idx:04d}"


def resolve_per_cam_ply(
    vis_dir: str,
    cam: str,
    path_idx: int,
    point_idx: int,
    *,
    ply_suffix: str | None,
) -> str | None:
    stem = frame_stem(path_idx, point_idx)
    suffixes = (ply_suffix,) if ply_suffix else PLY_SUFFIXES
    for suffix in suffixes:
        path = os.path.join(vis_dir, f"{cam}_{suffix}_{stem}.ply")
        if os.path.isfile(path):
            return path
    return None


def merge_camera_plys(
    vis_dir: str,
    path_idx: int,
    point_idx: int,
    cameras: tuple[str, ...],
    *,
    ply_suffix: str | None,
) -> tuple[np.ndarray, list[str]]:
    """按 --cameras 拼接各路相机点云,等价于 all_cameras_world 合并结果。"""
    chunks: list[np.ndarray] = []
    sources: list[str] = []
    for cam in cameras:
        ply_path = resolve_per_cam_ply(
            vis_dir, cam, path_idx, point_idx, ply_suffix=ply_suffix,
        )
        if ply_path is None:
            stem = frame_stem(path_idx, point_idx)
            raise FileNotFoundError(
                f"缺少 {cam}_*_{stem}.ply "
                f"(vis_dir={vis_dir}, ply_suffix={ply_suffix or 'auto'})"
            )
        data, _ = read_ply_binary(ply_path, stride=1)
        chunks.append(data)
        sources.append(os.path.basename(ply_path))
    return np.concatenate(chunks), sources


def discover_vis_frames(
    vis_dir: str,
    cameras: tuple[str, ...],
    *,
    ply_suffix: str | None,
) -> list[tuple[int, int]]:
    """发现 vis/ 下指定相机均有点云的路径点 (path_idx, point_idx)。"""
    cam_set = set(cameras)
    candidates: set[tuple[int, int]] = set()
    for name in os.listdir(vis_dir):
        m = PER_CAM_PLY_PATTERN.match(name)
        if not m or m.group('cam') not in cam_set:
            continue
        if ply_suffix is not None and m.group('suffix') != ply_suffix:
            continue
        candidates.add((int(m.group('path')), int(m.group('point'))))

    entries: list[tuple[int, int]] = []
    for path_idx, point_idx in sorted(candidates):
        if all(
            resolve_per_cam_ply(
                vis_dir, cam, path_idx, point_idx, ply_suffix=ply_suffix,
            )
            for cam in cameras
        ):
            entries.append((path_idx, point_idx))
    return entries


def build_vis_tag_to_path_row(
    vis_path_tags: list[int],
    num_paths: int,
    selected_rows: set[int],
) -> dict[int, int]:
    """将 vis/ 文件名中的 path 编号映射到 paths.npy 行下标。

    gen_data.py 使用 0 起标; gen_data_from_trajectory 沿用 rig_poses_XXXX 的 XXXX
    (可为 0001 等), 此时 paths.npy 仍只有一行且下标为 0。
    """
    unique = sorted(set(vis_path_tags))
    if not unique or num_paths <= 0:
        return {}

    if unique == list(range(num_paths)):
        return {tag: tag for tag in unique if tag in selected_rows}

    if len(unique) == num_paths:
        base = unique[0]
        if unique == list(range(base, base + num_paths)):
            return {
                tag: row
                for tag, row in ((t, t - base) for t in unique)
                if row in selected_rows
            }

    return {tag: tag for tag in unique if tag in selected_rows and tag < num_paths}


def write_tf(writer: ProtobufWriter, stamp: datetime, frame_id: str, log_t: int) -> None:
    tf = make_frame_transform_proto(stamp, frame_id)
    writer.write_message("/tf", tf, log_time=log_t, publish_time=log_t)


def mcap_output_dir(workdir: str, output: str | None) -> str:
    if output is not None:
        return os.path.abspath(output)
    return os.path.join(workdir, 'mcap')


def mcap_frame_output_path(
    out_dir: str, workdir_name: str, path_idx: int, point_idx: int,
) -> str:
    return os.path.join(
        out_dir, f"{workdir_name}_path_{path_idx:04d}_pt_{point_idx:04d}.mcap",
    )


def frame_stamp(base_time: datetime, global_frame_i: int, fps: float) -> datetime:
    return datetime.fromtimestamp(
        base_time.timestamp() + global_frame_i / max(fps, 1e-6),
        tz=timezone.utc,
    )


def write_frame_mcap(
    out_path: str,
    path_idx: int,
    point_idx: int,
    path_xyz: np.ndarray,
    *,
    vis_dir: str,
    cameras: tuple[str, ...],
    ply_suffix: str | None,
    occ_data: np.ndarray | None,
    downsample: int,
    max_points: int | None,
    stamp: datetime,
    frame_id: str,
) -> np.ndarray | None:
    """将单帧 vis 点云写入一个 MCAP; 成功返回点云数组供 accumulate 复用。"""
    stem = frame_stem(path_idx, point_idx)
    if point_idx >= len(path_xyz):
        print(
            f"  [警告] {stem} 超出轨迹点数 ({point_idx} >= {len(path_xyz)})",
            file=sys.stderr,
        )

    try:
        merged, sources = merge_camera_plys(
            vis_dir, path_idx, point_idx, cameras, ply_suffix=ply_suffix,
        )
    except FileNotFoundError as exc:
        print(f"  [跳过] {exc}", file=sys.stderr)
        return None

    n_raw = len(merged)
    if downsample > 1:
        merged = np.asarray(merged[::downsample])
    data = cap_points(merged, max_points)
    print(f"    {' + '.join(sources)}")
    print(
        f"    点数 {n_raw:,} -> {len(data):,} "
        f"(downsample={downsample}, max_points={max_points})"
    )

    log_t = int(stamp.timestamp() * 1e9)
    with ProtobufWriter(out_path) as writer:
        write_tf(writer, stamp, frame_id, log_t)
        if occ_data is not None:
            occ_msg = make_pointcloud_proto(occ_data, stamp, frame_id)
            writer.write_message(
                "/sim/occupancy", occ_msg, log_time=log_t, publish_time=log_t,
            )
        path_msg = make_poses_in_frame_proto(path_xyz, stamp, frame_id)
        writer.write_message("/sim/path", path_msg, log_time=log_t, publish_time=log_t)
        pc_msg = make_pointcloud_proto(data, stamp, frame_id)
        writer.write_message(
            "/sim/pointcloud", pc_msg, log_time=log_t, publish_time=log_t,
        )
    return data


def write_map_mcap(
    out_path: str,
    chunks: list[np.ndarray],
    *,
    stamp: datetime,
    frame_id: str,
    max_points: int | None,
) -> None:
    print(f"\n拼接 {len(chunks)} 帧点云并去重 -> {out_path} ...")
    merged = np.concatenate(chunks)
    n_before = len(merged)
    merged = dedup_xyz(merged)
    print(f"  去重 {n_before:,} -> {len(merged):,}")
    if max_points is not None and len(merged) > max_points:
        merged = cap_points(merged, max_points)
        print(f"  地图点云裁剪至 {len(merged):,}")

    log_t = int(stamp.timestamp() * 1e9)
    with ProtobufWriter(out_path) as writer:
        write_tf(writer, stamp, frame_id, log_t)
        map_msg = make_pointcloud_proto(merged, stamp, frame_id)
        writer.write_message(
            "/sim/map", map_msg, log_time=log_t, publish_time=log_t,
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workdir', help='数据根目录 (含 path/ 与 vis/)')
    parser.add_argument(
        '--output', default=None,
        help='MCAP 输出目录 (默认 <workdir>/mcap/)',
    )
    parser.add_argument('--path-idx', type=int, default=None, help='只处理指定路径编号')
    parser.add_argument('--downsample', type=int, default=10, help='相机点云降采样步长')
    parser.add_argument('--max-points', type=int, default=500000, help='相机点云/地图点数上限,0=不限')
    parser.add_argument(
        '--occupancy-downsample', type=int, default=1,
        help='占据场景降采样步长 (1=全量,比相机点云更密, None使用downsample)',
    )
    parser.add_argument(
        '--occupancy-max-points', type=int, default=0,
        help='占据场景点数上限,0=不限',
    )
    parser.add_argument('--fps', type=float, default=6.0, help='时间轴帧率 Hz')
    parser.add_argument(
        '--accumulate', action='store_true',
        help='额外写入 <name>_map.mcap (拼接去重全局点云,默认关闭)',
    )
    parser.add_argument('--frame-id', default=DEFAULT_FRAME_ID, help=f'坐标系名称 (默认 {DEFAULT_FRAME_ID})')
    parser.add_argument(
        '--cameras', nargs='+', default=list(DEFAULT_CAMERAS),
        metavar='CAM',
        help='参与合并的相机名 (默认六路: CAM_A/B/C/D/Front/Back)',
    )
    parser.add_argument(
        '--ply-suffix', choices=PLY_SUFFIXES, default=None,
        help='单相机 PLY 后缀 (默认按 lut_world / mei_world / pinhole_world 自动选择)',
    )
    args = parser.parse_args()

    workdir = os.path.abspath(args.workdir)
    path_dir = os.path.join(workdir, 'path')
    vis_dir = os.path.join(workdir, 'vis')
    paths_npy = os.path.join(path_dir, 'paths.npy')

    if not os.path.isfile(paths_npy):
        print(f"未找到轨迹: {paths_npy}", file=sys.stderr)
        sys.exit(1)
    if not os.path.isdir(vis_dir):
        print(f"未找到点云目录: {vis_dir}", file=sys.stderr)
        sys.exit(1)

    paths_arr = np.load(paths_npy)
    if paths_arr.ndim != 3 or paths_arr.shape[2] != 3:
        print(f"paths.npy 形状异常: {paths_arr.shape}", file=sys.stderr)
        sys.exit(1)

    cameras = tuple(args.cameras)
    vis_frames = discover_vis_frames(vis_dir, cameras, ply_suffix=args.ply_suffix)
    if not vis_frames:
        cam_list = ', '.join(cameras)
        suffix_hint = args.ply_suffix or 'lut_world|mei_world|pinhole_world'
        print(
            f"vis/ 下无完整帧点云 ({cam_list}, 后缀 {suffix_hint}, "
            f"命名如 CAM_A_<suffix>_0000_0001.ply)",
            file=sys.stderr,
        )
        sys.exit(1)
    print(f"点云来源: 合并 {', '.join(cameras)} (不使用 all_cameras_world_*.ply)")

    out_dir = mcap_output_dir(workdir, args.output)
    os.makedirs(out_dir, exist_ok=True)
    workdir_name = os.path.basename(workdir)
    max_points = None if args.max_points <= 0 else args.max_points
    occ_max_points = None if args.occupancy_max_points <= 0 else args.occupancy_max_points
    base_time = datetime.now(timezone.utc)
    frame_id = args.frame_id

    path_indices = (
        [args.path_idx]
        if args.path_idx is not None
        else list(range(paths_arr.shape[0]))
    )
    path_idx_set = {
        i for i in path_indices if 0 <= i < paths_arr.shape[0]
    }
    for i in path_indices:
        if i not in path_idx_set:
            print(f"[跳过] 无效 path_idx={i}", file=sys.stderr)

    vis_tags = [p for p, _ in vis_frames]
    tag_to_row = build_vis_tag_to_path_row(
        vis_tags, paths_arr.shape[0], path_idx_set,
    )
    all_frames: list[tuple[int, int, int]] = []
    for vis_tag, point_idx in vis_frames:
        path_row = tag_to_row.get(vis_tag)
        if path_row is not None:
            all_frames.append((path_row, vis_tag, point_idx))

    if tag_to_row and any(t != r for t, r in tag_to_row.items()):
        pairs = ', '.join(f"vis {t:04d}->paths[{r}]" for t, r in sorted(tag_to_row.items()))
        print(f"路径编号映射: {pairs}")

    if not all_frames:
        vis_tag_set = sorted({p for p, _ in vis_frames})
        print("\n未找到可处理的 vis 帧", file=sys.stderr)
        if vis_frames and vis_tag_set:
            print(
                f"  vis 路径编号 {vis_tag_set}, paths.npy 行下标 {sorted(path_idx_set)}; "
                "手动轨迹采数时 vis 编号常与 paths.npy 行号不一致",
                file=sys.stderr,
            )
        sys.exit(1)

    occupied_npy = os.path.join(workdir, 'occupancy', OCCUPANCY_NPY)
    occ_data: np.ndarray | None = None
    if os.path.isfile(occupied_npy):
        if args.occupancy_downsample is not None:
            occ_stride = max(1, args.occupancy_downsample)
        else:
            occ_stride = max(1, args.downsample)
        print(f"加载占据场景: {occupied_npy} ...")
        occ_data, occ_n_raw = load_occupied_scene(
            occupied_npy, occ_stride, occ_max_points,
        )
        print(
            f"  占据点云 {occ_n_raw:,} -> {len(occ_data):,} "
            f"(灰 RGB{OCCUPANCY_GRAY_RGB}, downsample={occ_stride}, 每帧 MCAP 各写一份)"
        )
    else:
        print(f"[跳过] 未找到占据栅格: {occupied_npy}", file=sys.stderr)

    print(f"输出目录: {out_dir}")
    print(f"共 {len(all_frames)} 帧, 时间轴 {args.fps} Hz (Foxglove 可同时打开全部 MCAP)")

    t0 = time.time()
    written_count = 0
    total_bytes = 0
    accumulate_chunks: list[np.ndarray] = []

    for global_i, (path_row, vis_tag, point_idx) in enumerate(all_frames):
        stamp = frame_stamp(base_time, global_i, args.fps)
        stem = frame_stem(vis_tag, point_idx)
        out_path = mcap_frame_output_path(out_dir, workdir_name, vis_tag, point_idx)
        print(
            f"\n[{global_i + 1}/{len(all_frames)}] {stem} "
            f"t={stamp.isoformat()} -> {os.path.basename(out_path)}"
        )
        print(f"  合并 ({', '.join(cameras)}) ...")

        path_xyz = paths_arr[path_row]
        frame_data = write_frame_mcap(
            out_path,
            vis_tag,
            point_idx,
            path_xyz,
            vis_dir=vis_dir,
            cameras=cameras,
            ply_suffix=args.ply_suffix,
            occ_data=occ_data,
            downsample=args.downsample,
            max_points=max_points,
            stamp=stamp,
            frame_id=frame_id,
        )
        if frame_data is None:
            continue
        written_count += 1
        total_bytes += os.path.getsize(out_path)
        size_mb = os.path.getsize(out_path) / (1024 * 1024)
        print(f"  -> {size_mb:.2f} MB")

        if args.accumulate:
            accumulate_chunks.append(frame_data)

    elapsed = time.time() - t0
    if written_count == 0:
        print("\n未生成任何 MCAP", file=sys.stderr)
        sys.exit(1)

    if args.accumulate and accumulate_chunks:
        map_stamp = frame_stamp(base_time, len(all_frames), args.fps)
        map_path = os.path.join(out_dir, f"{workdir_name}_map.mcap")
        write_map_mcap(
            map_path, accumulate_chunks,
            stamp=map_stamp, frame_id=frame_id, max_points=max_points,
        )
        total_bytes += os.path.getsize(map_path)

    total_mb = total_bytes / (1024 * 1024)
    print(
        f"\n完成: {written_count} 个帧 MCAP -> {out_dir} "
        f"({total_mb:.1f} MB, {elapsed:.1f}s)"
    )
    print(
        f"Foxglove: 打开 {out_dir} 下全部 *.mcap; 固定参考系={frame_id}; "
        "点云 Color mode 选「RGBA (separate fields)」; "
        "订阅 /sim/occupancy(灰场景) + /sim/pointcloud(彩色观测)。"
    )


if __name__ == '__main__':
    main()
