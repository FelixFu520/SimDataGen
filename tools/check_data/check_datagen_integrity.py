#!/usr/bin/env python3
"""
检查数据生产目录完整性：根据 gen_data.log 是否包含成功标志，
并统计 rgb/CAM_A 下的图片数量作为帧数。

用法示例:
  python tools/check_data/check_datagen_integrity.py /root/vepfs/isaacsim/DataGen_omni/workdir_omni_20260512
  python tools/check_data/check_datagen_integrity.py /path/to/workdir --verbose
  python tools/check_data/check_datagen_integrity.py /path/to/workdir --json
  python tools/check_data/check_datagen_integrity.py /path/to/workdir --no-progress
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


DEFAULT_SUCCESS_MARKER = "数据生成完成, 保存路径"
DEFAULT_RGB_CAM = Path("rgb") / "CAM_A"
IMAGE_SUFFIXES = frozenset({".jpg", ".jpeg", ".png", ".webp", ".tif", ".tiff", ".bmp"})


@dataclass
class TaskReport:
    name: str
    log_exists: bool
    log_success: bool
    cam_a_exists: bool
    frame_count: int  # -1 表示 CAM_A 路径不是目录


def iter_task_dirs(root: Path) -> list[Path]:
    if not root.is_dir():
        return []
    dirs = [p for p in root.iterdir() if p.is_dir()]
    dirs.sort(key=lambda p: p.name)
    return dirs


def log_contains_success(log_path: Path, marker: str) -> bool:
    if not log_path.is_file():
        return False
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if marker in line:
                    return True
    except OSError:
        return False
    return False


def count_frames(cam_dir: Path) -> tuple[int, bool]:
    """返回 (帧数, cam_dir 是否为有效目录)。非目录时帧数为 -1。"""
    if not cam_dir.is_dir():
        return -1, False
    n = 0
    try:
        for p in cam_dir.iterdir():
            if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES:
                n += 1
    except OSError:
        return -1, False
    return n, True


def scan_root(
    root: Path,
    *,
    success_marker: str,
    rgb_cam: Path,
    progress: bool = False,
    progress_file=None,
) -> list[TaskReport]:
    if progress_file is None:
        progress_file = sys.stderr
    tasks = iter_task_dirs(root)
    total = len(tasks)
    if progress:
        print(f"共 {total} 个任务文件夹，开始检查…", file=progress_file, flush=True)
    reports: list[TaskReport] = []
    for i, task in enumerate(tasks, 1):
        if progress:
            print(f"[{i}/{total}] 当前文件夹: {task.name}", file=progress_file, flush=True)
        log_path = task / "gen_data.log"
        cam_a = task / rgb_cam
        log_exists = log_path.is_file()
        log_ok = log_contains_success(log_path, success_marker) if log_exists else False
        frames, cam_ok = count_frames(cam_a)
        reports.append(
            TaskReport(
                name=task.name,
                log_exists=log_exists,
                log_success=log_ok,
                cam_a_exists=cam_ok,
                frame_count=frames,
            )
        )
        if progress:
            log_st = "成功" if log_ok else ("无日志" if not log_exists else "未完成")
            fc = frames if frames >= 0 else "N/A"
            print(
                f"    -> 日志: {log_st}, CAM_A 帧数: {fc}  (累计已处理 {i}/{total})",
                file=progress_file,
                flush=True,
            )
    return reports


def summarize(reports: list[TaskReport]) -> dict:
    total = len(reports)
    with_log = sum(1 for r in reports if r.log_exists)
    success = sum(1 for r in reports if r.log_success)
    frames_if_cam = [r.frame_count for r in reports if r.cam_a_exists and r.frame_count >= 0]
    frames_total = sum(frames_if_cam)
    frames_success = sum(
        r.frame_count for r in reports if r.log_success and r.cam_a_exists and r.frame_count >= 0
    )
    failed = [r.name for r in reports if not r.log_success]
    return {
        "task_folder_count": total,
        "gen_data_log_present": with_log,
        "log_marked_success": success,
        "log_marked_failed_or_incomplete": total - success,
        "total_frames_rgb_cam_a": frames_total,
        "total_frames_among_log_success_only": frames_success,
        "tasks_missing_log": [r.name for r in reports if not r.log_exists],
        "tasks_log_not_success": [r.name for r in reports if r.log_exists and not r.log_success],
        "tasks_missing_cam_a": [r.name for r in reports if not r.cam_a_exists],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="检查 gen_data 产出目录完整性(日志成功标志 + CAM_A 帧数)")
    parser.add_argument(
        "root",
        type=Path,
        help="生产结果根目录(其下每个子文件夹视为一个任务)",
    )
    parser.add_argument(
        "--marker",
        default=DEFAULT_SUCCESS_MARKER,
        help=f'判定成功的日志子串(默认: "{DEFAULT_SUCCESS_MARKER}")',
    )
    parser.add_argument(
        "--rgb-cam",
        type=Path,
        default=DEFAULT_RGB_CAM,
        help=f"相对任务目录的 RGB 相机路径(默认: {DEFAULT_RGB_CAM.as_posix()})",
    )
    parser.add_argument("--verbose", "-v", action="store_true", help="逐任务打印")
    parser.add_argument("--json", action="store_true", help="输出 JSON(含每任务明细)")
    parser.add_argument(
        "--no-progress",
        action="store_true",
        help="不打印扫描进度(默认会打印当前文件夹与 序号/总数)",
    )
    args = parser.parse_args()

    root = args.root.expanduser().resolve()
    if not root.is_dir():
        print(f"错误: 根目录不存在或不是目录: {root}", file=sys.stderr)
        return 2

    show_progress = not args.no_progress
    # 进度走 stderr，便于 `脚本 > 报告.txt` 时报告里不含进度行
    reports = scan_root(
        root,
        success_marker=args.marker,
        rgb_cam=args.rgb_cam,
        progress=show_progress,
        progress_file=sys.stderr,
    )
    summary = summarize(reports)

    if args.json:
        out = {
            "root": str(root),
            "marker": args.marker,
            "rgb_cam_relative": args.rgb_cam.as_posix(),
            "summary": summary,
            "tasks": [asdict(r) for r in reports],
        }
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0 if summary["log_marked_failed_or_incomplete"] == 0 else 1

    print(f"根目录: {root}")
    print(f"成功标志: {args.marker!r}")
    print(f"帧数统计路径(相对任务): {args.rgb_cam.as_posix()}")
    print()
    print(f"任务文件夹总数: {summary['task_folder_count']}")
    print(f"存在 gen_data.log: {summary['gen_data_log_present']}")
    print(f"日志判定成功: {summary['log_marked_success']}")
    print(f"日志未成功或缺日志: {summary['log_marked_failed_or_incomplete']}")
    print(f"rgb/CAM_A 下图片总数(全部任务): {summary['total_frames_rgb_cam_a']}")
    print(f"rgb/CAM_A 下图片总数(仅日志成功任务): {summary['total_frames_among_log_success_only']}")
    print()

    if summary["tasks_missing_log"]:
        print("缺少 gen_data.log:")
        for n in summary["tasks_missing_log"]:
            print(f"  - {n}")
        print()
    if summary["tasks_log_not_success"]:
        print("有日志但未出现成功标志:")
        for n in summary["tasks_log_not_success"]:
            print(f"  - {n}")
        print()
    if summary["tasks_missing_cam_a"]:
        print("缺少 rgb/CAM_A 目录:")
        for n in summary["tasks_missing_cam_a"]:
            print(f"  - {n}")
        print()

    if args.verbose:
        print("逐任务:")
        for r in reports:
            status = "OK" if r.log_success else "FAIL"
            fc = r.frame_count if r.frame_count >= 0 else "N/A"
            print(f"  [{status}] {r.name}  frames={fc}  log={r.log_exists}  cam_a={r.cam_a_exists}")

    return 0 if summary["log_marked_failed_or_incomplete"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
