#!/usr/bin/env python3
"""
将 workdir 中每个任务文件夹的多相机 RGB + Depth 拼接成预览视频。

每帧布局: CAM_A/B/C/D 俯视顺时针 2x2 排列 (左上→右上→右下→左下), 每格 [RGB | Depth]。
先按原始比例拼接, 再整体等比缩放至目标分辨率并居中 (不足处黑边填充)。
默认输出 4K (3840x3840)。
输出视频文件名 = 文件夹名 (如 intime_home_000_100_1_30.mp4)。
编码: H.264 (yuv420p), 可在 Cursor / 浏览器中直接预览。

用法示例:
  conda activate volc
  python tools/check_data/make_rgb_depth_video.py --input workdir02
  python tools/check_data/make_rgb_depth_video.py --input workdir02/intime_home_000_100_1_30
  python tools/check_data/make_rgb_depth_video.py --input workdir02 --workers 4

输出: 每个任务文件夹内生成 {文件夹名}.mp4
依赖: opencv-python-headless, imageio-ffmpeg
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, ThreadPoolExecutor
from dataclasses import dataclass
from functools import partial
from pathlib import Path

import cv2
import imageio_ffmpeg
import numpy as np

RGB_SUFFIXES = (".jpg", ".jpeg", ".png")
DEPTH_SUFFIXES = (".png",)
DEFAULT_SIZE = 3840

# 俯视顺时针: 左上 A → 右上 B → 右下 C → 左下 D
CAMERAS_CLOCKWISE = ("CAM_A", "CAM_B", "CAM_C", "CAM_D", "CAM_Front", "CAM_Back")
CAMERA_GRID_POS: dict[str, tuple[int, int]] = {
    "CAM_A": (0, 0),
    "CAM_B": (0, 1),
    "CAM_C": (1, 1),
    "CAM_D": (1, 0),
    "CAM_Front": (2, 0),
    "CAM_Back": (2, 1),
}
GRID_ROWS = 3
GRID_COLS = 2

_IMREAD_REDUCED = {
    2: cv2.IMREAD_REDUCED_COLOR_2,
    4: cv2.IMREAD_REDUCED_COLOR_4,
    8: cv2.IMREAD_REDUCED_COLOR_8,
}
_IMREAD_REDUCED_GRAY = {
    2: cv2.IMREAD_REDUCED_GRAYSCALE_2,
    4: cv2.IMREAD_REDUCED_GRAYSCALE_4,
    8: cv2.IMREAD_REDUCED_GRAYSCALE_8,
}


@dataclass(frozen=True)
class Layout:
    rows: int
    cols: int
    tile_h: int
    tile_w: int
    cell_h: int
    cell_w: int
    content_h: int
    content_w: int
    pad_y: int
    pad_x: int
    size: int
    read_divisor: int
    slots: dict[str, tuple[int, int]]


class H264VideoWriter:
    """通过 imageio-ffmpeg 的 libx264 写 MP4, 兼容性优于 OpenCV mp4v。"""

    def __init__(
        self,
        path: Path,
        fps: float,
        width: int,
        height: int,
        *,
        preset: str = "ultrafast",
        crf: int = 23,
    ) -> None:
        self._path = path
        self._proc = subprocess.Popen(
            [
                imageio_ffmpeg.get_ffmpeg_exe(),
                "-y",
                "-threads",
                "0",
                "-f",
                "rawvideo",
                "-vcodec",
                "rawvideo",
                "-s",
                f"{width}x{height}",
                "-pix_fmt",
                "bgr24",
                "-r",
                str(fps),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-preset",
                preset,
                "-crf",
                str(crf),
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(path),
            ],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=4 * 1024 * 1024,
        )
        if self._proc.stdin is None:
            raise RuntimeError(f"无法创建 ffmpeg 进程: {path}")

    def write(self, frame: np.ndarray) -> None:
        if not frame.flags["C_CONTIGUOUS"]:
            frame = np.ascontiguousarray(frame)
        self._proc.stdin.write(frame.tobytes())

    def release(self) -> None:
        if self._proc.stdin:
            self._proc.stdin.close()
        ret = self._proc.wait()
        if ret != 0:
            raise RuntimeError(f"ffmpeg 编码失败 (exit {ret}): {self._path}")


def _find_file(stem: Path, suffixes: tuple[str, ...]) -> Path | None:
    for suffix in suffixes:
        path = stem.with_suffix(suffix)
        if path.is_file():
            return path
    return None


def _pick_read_divisor(scale: float) -> int:
    if scale <= 0.125:
        return 8
    if scale <= 0.25:
        return 4
    if scale <= 0.5:
        return 2
    return 1


def _build_layout(src_h: int, src_w: int, size: int, cameras: list[str]) -> Layout:
    """先按源图比例拼网格, 再整体等比缩放到 size×size 内并居中。"""
    rows, cols = GRID_ROWS, GRID_COLS
    cell_h_src = src_h
    cell_w_src = src_w * 2
    content_h_src = rows * cell_h_src
    content_w_src = cols * cell_w_src
    scale = min(size / content_h_src, size / content_w_src)
    tile_h = max(1, int(round(src_h * scale)))
    tile_w = max(1, int(round(src_w * scale)))
    cell_h, cell_w = tile_h, tile_w * 2
    content_h, content_w = rows * cell_h, cols * cell_w
    pad_y = (size - content_h) // 2
    pad_x = (size - content_w) // 2
    slots: dict[str, tuple[int, int]] = {}
    for cam in cameras:
        if cam not in CAMERA_GRID_POS:
            continue
        row, col = CAMERA_GRID_POS[cam]
        slots[cam] = (pad_y + row * cell_h, pad_x + col * cell_w)
    return Layout(
        rows=rows,
        cols=cols,
        tile_h=tile_h,
        tile_w=tile_w,
        cell_h=cell_h,
        cell_w=cell_w,
        content_h=content_h,
        content_w=content_w,
        pad_y=pad_y,
        pad_x=pad_x,
        size=size,
        read_divisor=_pick_read_divisor(scale),
        slots=slots,
    )


def _resize_tile(img: np.ndarray, layout: Layout) -> np.ndarray:
    if img.shape[0] == layout.tile_h and img.shape[1] == layout.tile_w:
        return img
    return cv2.resize(img, (layout.tile_w, layout.tile_h), interpolation=cv2.INTER_LINEAR)


def _load_rgb(path: Path, layout: Layout) -> np.ndarray | None:
    flags = cv2.IMREAD_COLOR
    if layout.read_divisor > 1:
        flags = _IMREAD_REDUCED[layout.read_divisor]
    img = cv2.imread(str(path), flags)
    if img is None:
        return None
    return _resize_tile(img, layout)


def _load_depth_png(path: Path, layout: Layout) -> np.ndarray | None:
    flags = cv2.IMREAD_GRAYSCALE
    if layout.read_divisor > 1:
        flags = _IMREAD_REDUCED_GRAY[layout.read_divisor]
    gray = cv2.imread(str(path), flags)
    if gray is None:
        return None
    gray = _resize_tile(gray, layout)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def _load_depth_npy(path: Path, layout: Layout) -> np.ndarray | None:
    if not path.is_file():
        return None
    depth_arr = np.squeeze(np.load(path)).astype(np.float32)
    valid = depth_arr > 0
    if not np.any(valid):
        return np.zeros((layout.tile_h, layout.tile_w, 3), dtype=np.uint8)
    lo, hi = float(depth_arr[valid].min()), float(depth_arr[valid].max())
    if hi <= lo:
        hi = lo + 1.0
    dnorm = np.clip((depth_arr - lo) / (hi - lo) * 255.0, 0, 255).astype(np.uint8)
    dnorm[~valid] = 0
    vis = cv2.applyColorMap(dnorm, cv2.COLORMAP_TURBO)
    return _resize_tile(vis, layout)


def _put_label(img: np.ndarray, text: str, *, tile_h: int, tile_w: int) -> None:
    font_scale = max(0.9, min(tile_h, tile_w) / 380.0)
    x = max(8, int(tile_w * 0.012))
    y = max(28, int(tile_h * 0.075))
    thickness = max(2, int(round(font_scale * 2.0)))
    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 255, 255),
        thickness,
        cv2.LINE_8,
    )
    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (0, 0, 0),
        max(1, thickness - 1),
        cv2.LINE_8,
    )


def _list_cameras(task_dir: Path) -> list[str]:
    rgb_root = task_dir / "rgb"
    if not rgb_root.is_dir():
        return []
    return [cam for cam in CAMERAS_CLOCKWISE if (rgb_root / cam).is_dir()]


def _list_frames(task_dir: Path, camera: str) -> list[str]:
    cam_dir = task_dir / "rgb" / camera
    if not cam_dir.is_dir():
        return []
    return sorted(
        p.stem
        for p in cam_dir.iterdir()
        if p.is_file() and p.suffix.lower() in RGB_SUFFIXES
    )


def _infer_tile_size(task_dir: Path, cameras: list[str], frames: list[str]) -> tuple[int, int]:
    for frame_id in frames:
        for cam in cameras:
            rgb_path = _find_file(task_dir / "rgb" / cam / frame_id, RGB_SUFFIXES)
            if rgb_path is None:
                continue
            rgb = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
            if rgb is not None:
                return rgb.shape[:2]
    raise RuntimeError(f"无法从 {task_dir} 推断 RGB 尺寸")


def _index_modality_dir(mod_dir: Path, suffixes: tuple[str, ...]) -> dict[str, Path]:
    if not mod_dir.is_dir():
        return {}
    return {
        p.stem: p
        for p in mod_dir.iterdir()
        if p.is_file() and p.suffix.lower() in suffixes
    }


def _precompute_paths(
    task_dir: Path,
    cameras: list[str],
    frames: list[str],
    depth_source: str,
) -> list[list[tuple[Path | None, Path | None]]]:
    rgb_maps = {cam: _index_modality_dir(task_dir / "rgb" / cam, RGB_SUFFIXES) for cam in cameras}
    if depth_source == "png":
        depth_maps = {
            cam: _index_modality_dir(task_dir / "depth" / cam, DEPTH_SUFFIXES) for cam in cameras
        }
    else:
        depth_maps = {
            cam: _index_modality_dir(task_dir / "depth" / cam, (".npy",)) for cam in cameras
        }

    paths: list[list[tuple[Path | None, Path | None]]] = []
    for frame_id in frames:
        row = [(rgb_maps[cam].get(frame_id), depth_maps[cam].get(frame_id)) for cam in cameras]
        paths.append(row)
    return paths


def _load_cam_pair(
    cam: str,
    rgb_path: Path | None,
    depth_path: Path | None,
    layout: Layout,
    depth_source: str,
) -> tuple[str, np.ndarray | None, np.ndarray | None]:
    if rgb_path is None or depth_path is None:
        return cam, None, None
    rgb = _load_rgb(rgb_path, layout)
    if depth_source == "png":
        depth = _load_depth_png(depth_path, layout)
    else:
        depth = _load_depth_npy(depth_path, layout)
    return cam, rgb, depth


def _compose_frame(
    frame_paths: list[tuple[Path | None, Path | None]],
    cameras: list[str],
    layout: Layout,
    depth_source: str,
    square: np.ndarray,
    *,
    load_workers: int,
) -> bool:
    loaded_any = False
    loader = partial(_load_cam_pair, layout=layout, depth_source=depth_source)
    tasks = [
        (cam, rgb_path, depth_path)
        for cam, (rgb_path, depth_path) in zip(cameras, frame_paths)
    ]

    if load_workers > 1 and len(tasks) > 1:
        with ThreadPoolExecutor(max_workers=min(load_workers, len(tasks))) as pool:
            results = pool.map(lambda item: loader(*item), tasks)
    else:
        results = [loader(cam, rgb_path, depth_path) for cam, rgb_path, depth_path in tasks]

    tw, th = layout.tile_w, layout.tile_h
    for cam, rgb, depth in results:
        if rgb is None or depth is None or cam not in layout.slots:
            continue

        _put_label(rgb, cam, tile_h=th, tile_w=tw)
        _put_label(depth, cam, tile_h=th, tile_w=tw)

        y0, x0 = layout.slots[cam]
        cell = square[y0 : y0 + layout.cell_h, x0 : x0 + layout.cell_w]
        cell[:, :tw] = rgb
        cell[:, tw:] = depth
        loaded_any = True
    return loaded_any


def _render_frame_task(
    frame_paths: list[tuple[Path | None, Path | None]],
    cameras: tuple[str, ...],
    layout: Layout,
    depth_source: str,
    load_workers: int,
) -> np.ndarray | None:
    square = np.zeros((layout.size, layout.size, 3), dtype=np.uint8)
    if not _compose_frame(
        frame_paths,
        list(cameras),
        layout,
        depth_source,
        square,
        load_workers=load_workers,
    ):
        return None
    return square


def _make_video_for_task(
    task_dir: Path,
    task_idx: int,
    task_total: int,
    fps: float,
    depth_source: str,
    size: int,
    progress_interval: int,
    *,
    workers: int,
    load_workers: int,
    encode_preset: str,
    encode_crf: int,
) -> Path | None:
    cameras = _list_cameras(task_dir)
    if not cameras:
        print(f"[跳过] ({task_idx}/{task_total}) {task_dir.name}: 未找到 rgb/<CAM>/ 目录", file=sys.stderr)
        return None

    frames = _list_frames(task_dir, cameras[0])
    if not frames:
        print(f"[跳过] ({task_idx}/{task_total}) {task_dir.name}: 无帧数据", file=sys.stderr)
        return None

    total_frames = len(frames)
    src_h, src_w = _infer_tile_size(task_dir, cameras, frames)
    layout = _build_layout(src_h, src_w, size, cameras)
    frame_paths = _precompute_paths(task_dir, cameras, frames, depth_source)
    cameras_tuple = tuple(cameras)
    frame_workers = _effective_frame_workers(total_frames, workers)

    print(
        f"\n[{task_idx}/{task_total}] 处理文件夹: {task_dir.name} "
        f"({total_frames} 帧, {len(cameras)} 相机, 输出 {size}x{size}, "
        f"frame_workers={frame_workers}, load_workers={load_workers})"
    )

    out_path = task_dir / f"{task_dir.name}.mp4"
    writer = H264VideoWriter(
        out_path, fps, size, size, preset=encode_preset, crf=encode_crf
    )

    written = 0
    try:
        if frame_workers > 1:
            render = partial(
                _render_frame_task,
                cameras=cameras_tuple,
                layout=layout,
                depth_source=depth_source,
                load_workers=load_workers,
            )
            with ProcessPoolExecutor(max_workers=frame_workers) as pool:
                chunk = max(1, total_frames // (frame_workers * 4))
                for frame_idx, square in enumerate(pool.map(render, frame_paths, chunksize=chunk), start=1):
                    frame_id = frames[frame_idx - 1]
                    if square is None:
                        print(
                            f"  [{task_dir.name}] 帧 {frame_idx}/{total_frames} ({frame_id}): 跳过(缺少图像)",
                            file=sys.stderr,
                        )
                        continue
                    writer.write(square)
                    written += 1
                    if (
                        frame_idx == 1
                        or frame_idx == total_frames
                        or frame_idx % progress_interval == 0
                    ):
                        print(
                            f"  [{task_dir.name}] 帧 {frame_idx}/{total_frames} ({frame_id})",
                            flush=True,
                        )
        else:
            square = np.zeros((size, size, 3), dtype=np.uint8)
            for frame_idx, (frame_id, paths) in enumerate(zip(frames, frame_paths), start=1):
                square.fill(0)
                if not _compose_frame(
                    paths,
                    cameras,
                    layout,
                    depth_source,
                    square,
                    load_workers=load_workers,
                ):
                    print(
                        f"  [{task_dir.name}] 帧 {frame_idx}/{total_frames} ({frame_id}): 跳过(缺少图像)",
                        file=sys.stderr,
                    )
                    continue
                writer.write(square)
                written += 1
                if (
                    frame_idx == 1
                    or frame_idx == total_frames
                    or frame_idx % progress_interval == 0
                ):
                    print(
                        f"  [{task_dir.name}] 帧 {frame_idx}/{total_frames} ({frame_id})",
                        flush=True,
                    )
    finally:
        writer.release()

    if written == 0:
        out_path.unlink(missing_ok=True)
        print(f"[跳过] ({task_idx}/{task_total}) {task_dir.name}: 无有效帧", file=sys.stderr)
        return None

    print(f"[完成] ({task_idx}/{task_total}) {task_dir.name}: {written}/{total_frames} 帧 -> {out_path}")
    return out_path


def _iter_task_dirs(input_path: Path) -> list[Path]:
    if (input_path / "rgb").is_dir():
        return [input_path]
    if not input_path.is_dir():
        raise FileNotFoundError(f"输入路径不存在: {input_path}")
    return sorted(p for p in input_path.iterdir() if p.is_dir())


def _default_workers() -> int:
    return max(1, min(4, (os.cpu_count() or 4) // 2))


def _effective_frame_workers(total_frames: int, workers: int) -> int:
    if workers <= 1 or total_frames < max(32, workers * 8):
        return 1
    return workers


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="将多相机 RGB/Depth 拼接并导出为 H.264 视频")
    parser.add_argument("--input", type=Path, required=True, help="任务根目录或单个任务目录")
    parser.add_argument("--fps", type=float, default=2.0, help="视频帧率 (默认: 10)")
    parser.add_argument(
        "--depth-source",
        choices=("png", "npy"),
        default="png",
        help="深度可视化来源 (默认: png, 已是灰度可视化图)",
    )
    parser.add_argument(
        "--size",
        type=int,
        default=DEFAULT_SIZE,
        help=f"输出正方形边长 (默认: {DEFAULT_SIZE}, 4K)",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=_default_workers(),
        help=f"并行渲染帧的进程数 (默认: {_default_workers()})",
    )
    parser.add_argument(
        "--load-workers",
        type=int,
        default=6,
        help="单帧内并行读取相机的线程数 (默认: 6)",
    )
    parser.add_argument(
        "--encode-preset",
        default="ultrafast",
        help="x264 编码 preset (默认: ultrafast, 更快)",
    )
    parser.add_argument(
        "--encode-crf",
        type=int,
        default=23,
        help="x264 CRF 质量 (默认: 23, 越大越快/文件越小)",
    )
    parser.add_argument(
        "--progress-interval",
        type=int,
        default=10,
        help="每 N 帧打印一次进度 (默认: 10)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = args.input.resolve()
    cv2.setNumThreads(0)

    task_dirs = _iter_task_dirs(input_path)
    if not task_dirs:
        print(f"未找到任务目录: {input_path}", file=sys.stderr)
        return 1

    task_total = len(task_dirs)
    print(f"共 {task_total} 个文件夹待处理, 输入: {input_path}")

    ok = 0
    for task_idx, task_dir in enumerate(task_dirs, start=1):
        result = _make_video_for_task(
            task_dir,
            task_idx,
            task_total,
            args.fps,
            args.depth_source,
            args.size,
            max(1, args.progress_interval),
            workers=max(1, args.workers),
            load_workers=max(1, args.load_workers),
            encode_preset=args.encode_preset,
            encode_crf=args.encode_crf,
        )
        if result is not None:
            ok += 1

    print(f"\n全部完成: 共 {task_total} 个文件夹, 成功 {ok} 个")
    return 0 if ok > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
