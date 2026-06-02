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

输出: 每个任务文件夹内生成 {文件夹名}.mp4
依赖: opencv-python-headless, imageio-ffmpeg
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from dataclasses import dataclass
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
    canvas_h: int
    canvas_w: int
    content_h: int
    content_w: int
    pad_y: int
    pad_x: int
    size: int
    read_divisor: int


class H264VideoWriter:
    """通过 imageio-ffmpeg 的 libx264 写 MP4, 兼容性优于 OpenCV mp4v。"""

    def __init__(self, path: Path, fps: float, width: int, height: int) -> None:
        self._path = path
        self._proc = subprocess.Popen(
            [
                imageio_ffmpeg.get_ffmpeg_exe(),
                "-y",
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
                "veryfast",
                "-crf",
                "23",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(path),
            ],
            stdin=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        if self._proc.stdin is None:
            raise RuntimeError(f"无法创建 ffmpeg 进程: {path}")

    def write(self, frame: np.ndarray) -> None:
        self._proc.stdin.write(np.ascontiguousarray(frame).tobytes())

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


def _build_layout(src_h: int, src_w: int, size: int) -> Layout:
    """先按源图比例拼 2x2 网格, 再整体等比缩放到 size×size 内并居中。"""
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
    return Layout(
        rows=rows,
        cols=cols,
        tile_h=tile_h,
        tile_w=tile_w,
        cell_h=cell_h,
        cell_w=cell_w,
        canvas_h=content_h,
        canvas_w=content_w,
        content_h=content_h,
        content_w=content_w,
        pad_y=pad_y,
        pad_x=pad_x,
        size=size,
        read_divisor=_pick_read_divisor(scale),
    )


def _load_rgb(path: Path, layout: Layout) -> np.ndarray | None:
    flags = cv2.IMREAD_COLOR
    if layout.read_divisor > 1:
        flags = _IMREAD_REDUCED[layout.read_divisor]
    img = cv2.imread(str(path), flags)
    if img is None:
        return None
    if img.shape[0] != layout.tile_h or img.shape[1] != layout.tile_w:
        img = cv2.resize(img, (layout.tile_w, layout.tile_h), interpolation=cv2.INTER_LINEAR)
    return img


def _load_depth_png(path: Path, layout: Layout) -> np.ndarray | None:
    flags = cv2.IMREAD_GRAYSCALE
    if layout.read_divisor > 1:
        flags = _IMREAD_REDUCED_GRAY[layout.read_divisor]
    gray = cv2.imread(str(path), flags)
    if gray is None:
        return None
    if gray.shape[0] != layout.tile_h or gray.shape[1] != layout.tile_w:
        gray = cv2.resize(gray, (layout.tile_w, layout.tile_h), interpolation=cv2.INTER_LINEAR)
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
    if vis.shape[0] != layout.tile_h or vis.shape[1] != layout.tile_w:
        vis = cv2.resize(vis, (layout.tile_w, layout.tile_h), interpolation=cv2.INTER_LINEAR)
    return vis


def _put_label(img: np.ndarray, text: str, *, tile_h: int, tile_w: int) -> None:
    font_scale = max(0.9, min(tile_h, tile_w) / 380.0)
    x = max(8, int(tile_w * 0.012))
    y = max(28, int(tile_h * 0.075))
    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 255, 255),
        max(2, int(round(font_scale * 2.5))),
        cv2.LINE_AA,
    )
    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (0, 0, 0),
        max(1, int(round(font_scale * 1.2))),
        cv2.LINE_AA,
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


def _precompute_paths(
    task_dir: Path,
    cameras: list[str],
    frames: list[str],
    depth_source: str,
) -> list[list[tuple[Path | None, Path | None]]]:
    """每帧每相机的 (rgb_path, depth_path)。"""
    paths: list[list[tuple[Path | None, Path | None]]] = []
    for frame_id in frames:
        row: list[tuple[Path | None, Path | None]] = []
        for cam in cameras:
            rgb_path = _find_file(task_dir / "rgb" / cam / frame_id, RGB_SUFFIXES)
            if depth_source == "png":
                depth_path = _find_file(task_dir / "depth" / cam / frame_id, DEPTH_SUFFIXES)
            else:
                depth_path = task_dir / "depth" / cam / f"{frame_id}.npy"
                if not depth_path.is_file():
                    depth_path = None
            row.append((rgb_path, depth_path))
        paths.append(row)
    return paths


def _compose_frame(
    frame_paths: list[tuple[Path | None, Path | None]],
    cameras: list[str],
    layout: Layout,
    depth_source: str,
    square: np.ndarray,
) -> bool:
    loaded_any = False
    for cam, (rgb_path, depth_path) in zip(cameras, frame_paths):
        if rgb_path is None or depth_path is None:
            continue

        rgb = _load_rgb(rgb_path, layout)
        if depth_source == "png":
            depth = _load_depth_png(depth_path, layout)
        else:
            depth = _load_depth_npy(depth_path, layout)
        if rgb is None or depth is None:
            continue

        _put_label(rgb, cam, tile_h=layout.tile_h, tile_w=layout.tile_w)
        _put_label(depth, cam, tile_h=layout.tile_h, tile_w=layout.tile_w)
        tile = np.hstack((rgb, depth))

        row, col = CAMERA_GRID_POS[cam]
        y0 = layout.pad_y + row * layout.cell_h
        x0 = layout.pad_x + col * layout.cell_w
        square[y0 : y0 + layout.cell_h, x0 : x0 + layout.cell_w] = tile
        loaded_any = True
    return loaded_any


def _make_video_for_task(
    task_dir: Path,
    task_idx: int,
    task_total: int,
    fps: float,
    depth_source: str,
    size: int,
    progress_interval: int,
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
    layout = _build_layout(src_h, src_w, size)
    frame_paths = _precompute_paths(task_dir, cameras, frames, depth_source)

    print(
        f"\n[{task_idx}/{task_total}] 处理文件夹: {task_dir.name} "
        f"({total_frames} 帧, {len(cameras)} 相机 A→B→C→D 顺时针, 输出 {size}x{size} H.264)"
    )

    out_path = task_dir / f"{task_dir.name}.mp4"
    writer = H264VideoWriter(out_path, fps, size, size)
    square = np.zeros((size, size, 3), dtype=np.uint8)

    written = 0
    try:
        for frame_idx, (frame_id, paths) in enumerate(zip(frames, frame_paths), start=1):
            square.fill(0)
            if not _compose_frame(paths, cameras, layout, depth_source, square):
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="将多相机 RGB/Depth 拼接并导出为 H.264 视频")
    parser.add_argument("--input", type=Path, required=True, help="任务根目录或单个任务目录")
    parser.add_argument("--fps", type=float, default=10.0, help="视频帧率 (默认: 10)")
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
        "--progress-interval",
        type=int,
        default=10,
        help="每 N 帧打印一次进度 (默认: 10)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_path = args.input.resolve()

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
        )
        if result is not None:
            ok += 1

    print(f"\n全部完成: 共 {task_total} 个文件夹, 成功 {ok} 个")
    return 0 if ok > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
