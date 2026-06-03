#!/usr/bin/env python3
"""按轨迹将 vis/ 点云写入 MCAP,供 Foxglove 播放。

输入:
  - path/paths.npy: (num_paths, num_points, 3) 轨迹路点
  - vis/all_cameras_world_{path:04d}_{point:04d}.ply: 各路径点世界系点云
  - occupancy/occupied_positions.npy: 占据栅格 (相机拍不到的静态场景)

输出 topic (protobuf,Foxglove 可直接解析):
  - /tf: 静态坐标系 foxglove.FrameTransform
  - /sim/occupancy: 占据场景点云 (灰白色,默认全量 ~28 万点)
  - /sim/pointcloud: 每帧 foxglove.PointCloud
  - /sim/path: 轨迹 foxglove.PosesInFrame
  - /sim/map (仅 --accumulate): 拼接全局点云,体积大、生成慢,默认不写

用法:
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/intime_home_000_100_1_30
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/intime_home_000_100_1_30 --downsample 20
  ./app/python.sh tools/check_data/path_vis_to_mcap.py workdir/intime_home_000_100_1_30 --output output/mcaps

每条轨迹单独输出一个 MCAP: <name>_path_0000.mcap, <name>_path_0001.mcap, ...

Foxglove 3D 面板: 固定参考系选「world」; 点云 Color mode 选「RGBA (separate fields)」。
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

PLY_PATTERN = re.compile(
    r'^all_cameras_world_(?P<path>\d{4})_(?P<point>\d{4})\.ply$'
)

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


def discover_vis_frames(vis_dir: str) -> list[tuple[int, int, str]]:
    entries: list[tuple[int, int, str]] = []
    for name in os.listdir(vis_dir):
        m = PLY_PATTERN.match(name)
        if not m:
            continue
        entries.append((
            int(m.group('path')),
            int(m.group('point')),
            os.path.join(vis_dir, name),
        ))
    entries.sort(key=lambda x: (x[0], x[1]))
    return entries


def write_tf(writer: ProtobufWriter, stamp: datetime, frame_id: str, log_t: int) -> None:
    tf = make_frame_transform_proto(stamp, frame_id)
    writer.write_message("/tf", tf, log_time=log_t, publish_time=log_t)


def mcap_output_path(out_dir: str, workdir_name: str, path_idx: int) -> str:
    return os.path.join(out_dir, f"{workdir_name}_path_{path_idx:04d}.mcap")


def write_path_mcap(
    out_path: str,
    path_idx: int,
    path_xyz: np.ndarray,
    path_frames: list[tuple[int, int, str]],
    *,
    occupied_npy: str,
    occupancy_downsample: int | None,
    downsample: int,
    occ_max_points: int | None,
    max_points: int | None,
    fps: float,
    accumulate: bool,
    frame_id: str,
    base_time: datetime,
) -> tuple[int, float]:
    """将单条轨迹及其 vis 点云写入一个 MCAP, 返回 (帧数, 文件 MB)。"""
    accumulate_chunks: list[np.ndarray] = []
    written_frames = 0

    with ProtobufWriter(out_path) as writer:
        stamp0 = base_time
        log_t0 = int(stamp0.timestamp() * 1e9)
        write_tf(writer, stamp0, frame_id, log_t0)

        if os.path.isfile(occupied_npy):
            if occupancy_downsample is not None:
                occ_stride = max(1, occupancy_downsample)
            else:
                occ_stride = downsample
            occ_data, occ_n_raw = load_occupied_scene(
                occupied_npy, occ_stride, occ_max_points,
            )
            occ_msg = make_pointcloud_proto(occ_data, stamp0, frame_id)
            writer.write_message(
                "/sim/occupancy", occ_msg, log_time=log_t0, publish_time=log_t0,
            )
            print(
                f"  占据点云 {occ_n_raw:,} -> {len(occ_data):,} "
                f"(灰 RGB{OCCUPANCY_GRAY_RGB}, downsample={occ_stride})"
            )

        path_msg = make_poses_in_frame_proto(path_xyz, stamp0, frame_id)
        writer.write_message("/sim/path", path_msg, log_time=log_t0, publish_time=log_t0)

        print(f"path {path_idx:04d}: {len(path_xyz)} 路点, {len(path_frames)} 帧点云")

        for frame_i, (_p, point_idx, ply_path) in enumerate(path_frames):
            if point_idx >= len(path_xyz):
                print(
                    f"  [警告] {os.path.basename(ply_path)} 超出轨迹点数 "
                    f"({point_idx} >= {len(path_xyz)})",
                    file=sys.stderr,
                )

            stamp = datetime.fromtimestamp(
                stamp0.timestamp() + frame_i / max(fps, 1e-6),
                tz=timezone.utc,
            )
            log_t = int(stamp.timestamp() * 1e9)

            print(f"  [{frame_i + 1}/{len(path_frames)}] 读取 {os.path.basename(ply_path)} ...")
            data, n_raw = read_ply_binary(ply_path, stride=downsample)
            data = cap_points(data, max_points)
            print(
                f"    点数 {n_raw:,} -> {len(data):,} "
                f"(downsample={downsample}, max_points={max_points})"
            )

            if accumulate:
                accumulate_chunks.append(data)

            write_tf(writer, stamp, frame_id, log_t)
            pc_msg = make_pointcloud_proto(data, stamp, frame_id)
            writer.write_message(
                "/sim/pointcloud", pc_msg, log_time=log_t, publish_time=log_t,
            )
            written_frames += 1

        if accumulate and accumulate_chunks:
            print(f"\n拼接 {len(accumulate_chunks)} 帧点云并去重 ...")
            merged = np.concatenate(accumulate_chunks)
            del accumulate_chunks
            n_before = len(merged)
            merged = dedup_xyz(merged)
            print(f"  去重 {n_before:,} -> {len(merged):,}")
            if max_points is not None and len(merged) > max_points:
                merged = cap_points(merged, max_points)
                print(f"  地图点云裁剪至 {len(merged):,}")

            map_stamp = datetime.fromtimestamp(
                base_time.timestamp() + written_frames / max(fps, 1e-6),
                tz=timezone.utc,
            )
            map_log_t = int(map_stamp.timestamp() * 1e9)
            write_tf(writer, map_stamp, frame_id, map_log_t)
            map_msg = make_pointcloud_proto(merged, map_stamp, frame_id)
            writer.write_message(
                "/sim/map", map_msg, log_time=map_log_t, publish_time=map_log_t,
            )

    size_mb = os.path.getsize(out_path) / (1024 * 1024)
    return written_frames, size_mb


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('workdir', help='数据根目录 (含 path/ 与 vis/)')
    parser.add_argument(
        '--output', default=None,
        help='MCAP 输出目录 (默认写入 workdir, 即 <name>_path_0000.mcap 等)',
    )
    parser.add_argument('--path-idx', type=int, default=None, help='只处理指定路径编号')
    parser.add_argument('--downsample', type=int, default=10, help='相机点云降采样步长')
    parser.add_argument('--max-points', type=int, default=1000000, help='相机点云/地图点数上限,0=不限')
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
        help='写入 /sim/map (拼接去重,显著增大文件并变慢,默认关闭)',
    )
    parser.add_argument('--frame-id', default=DEFAULT_FRAME_ID, help=f'坐标系名称 (默认 {DEFAULT_FRAME_ID})')
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

    vis_frames = discover_vis_frames(vis_dir)
    if not vis_frames:
        print("vis/ 下无 all_cameras_world_*_*.ply", file=sys.stderr)
        sys.exit(1)

    out_dir = os.path.abspath(args.output) if args.output else workdir
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

    t0 = time.time()
    total_frames = 0
    written_mcaps: list[tuple[str, int, float]] = []

    occupied_npy = os.path.join(workdir, 'occupancy', OCCUPANCY_NPY)
    if not os.path.isfile(occupied_npy):
        print(f"[跳过] 未找到占据栅格: {occupied_npy}", file=sys.stderr)

    for path_idx in path_indices:
        if path_idx < 0 or path_idx >= paths_arr.shape[0]:
            print(f"[跳过] 无效 path_idx={path_idx}", file=sys.stderr)
            continue

        path_xyz = paths_arr[path_idx]
        path_frames = [(p, pt, fp) for p, pt, fp in vis_frames if p == path_idx]
        if not path_frames:
            print(f"[跳过] path {path_idx:04d}: vis 无对应点云", file=sys.stderr)
            continue

        out_path = mcap_output_path(out_dir, workdir_name, path_idx)
        print(f"\n写入 {out_path} ...")
        if os.path.isfile(occupied_npy):
            print(f"加载占据场景: {occupied_npy} ...")

        frames, size_mb = write_path_mcap(
            out_path,
            path_idx,
            path_xyz,
            path_frames,
            occupied_npy=occupied_npy,
            occupancy_downsample=args.occupancy_downsample,
            downsample=args.downsample,
            occ_max_points=occ_max_points,
            max_points=max_points,
            fps=args.fps,
            accumulate=args.accumulate,
            frame_id=frame_id,
            base_time=base_time,
        )
        total_frames += frames
        written_mcaps.append((out_path, frames, size_mb))
        print(f"  -> {frames} 帧, {size_mb:.1f} MB")

    elapsed = time.time() - t0
    if not written_mcaps:
        print("\n未生成任何 MCAP", file=sys.stderr)
        sys.exit(1)

    print(f"\n完成: {len(written_mcaps)} 个 MCAP, 共 {total_frames} 帧点云 ({elapsed:.1f}s)")
    for out_path, frames, size_mb in written_mcaps:
        print(f"  {out_path} ({frames} 帧, {size_mb:.1f} MB)")
    print(
        f"Foxglove: 固定参考系={frame_id}; "
        "点云 Color mode 选「RGBA (separate fields)」; "
        "订阅 /sim/occupancy(灰场景) + /sim/pointcloud(彩色观测)。"
    )


if __name__ == '__main__':
    main()
