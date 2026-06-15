#!/usr/bin/env python3
"""
批量把 USD 资产目录下的超大纹理(8K/4K)降采样到指定上限(默认 2048),
用于显著降低 Isaac Sim / Omniverse 加载场景时的显存占用。

背景:
- 磁盘上的 PNG/JPG/EXR 是压缩格式,但 GPU 显存里存的是解压后的原始像素。
  一张 8K(8192x8192)RGBA 在显存里约 256MB,加 mipmap 约 340MB。几十张 8K
  纹理会让显存需求飙到几十 GB,导致拖入大型场景(如 AsianVillage)时爆显存。
- 把长边 > max-size 的纹理离线降采样到 max-size,可把显存占用降一个数量级,
  对场景预览/数据生成的视觉影响通常可忽略。

特点:
- 只处理"长边超过阈值"的图片;不达标的直接跳过。
- 覆盖原文件前自动把原图备份到同级的 <dir>/_orig_backup/(保留相对结构),
  便于一键回滚。可用 --no-backup 关闭。
- PNG 用 Pillow 处理(保留 RGB/RGBA 通道);EXR 用 imageio 处理并保留浮点精度
  (法线/位移/HDR 贴图绝不能压成 8bit)。
- 支持 --dry-run 预览将要修改的文件与预计节省空间。
- 多进程并行加速。

用法(在仓库根目录下):
    # 先预览
    ./app/python.sh tools/usd/downscale_textures.py \
        --root assets_extern/USD/AsianVillage \
        --max-size 2048 --dry-run

    # 实际执行(默认会备份原图到 _orig_backup/)
    ./app/python.sh tools/usd/downscale_textures.py \
        --root assets_extern/USD/AsianVillage \
        --max-size 2048

回滚:
    ./app/python.sh tools/usd/downscale_textures.py \
        --root assets_extern/USD/AsianVillage --restore
"""

from __future__ import annotations

import argparse
import os

# 必须在任何 `import cv2` 之前设置:本机 OpenCV 已编入 OpenEXR,但默认禁用,
# 需要该环境变量才能读写 .exr。子进程会继承本环境变量。
os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")

import shutil
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

IMG_EXTS = {".png", ".jpg", ".jpeg", ".tga", ".tif", ".tiff", ".bmp"}
EXR_EXTS = {".exr", ".hdr"}
ALL_EXTS = IMG_EXTS | EXR_EXTS

BACKUP_DIRNAME = "_orig_backup"


# ---------------------------------------------------------------------------
# 单文件处理(在子进程中运行)
# ---------------------------------------------------------------------------
def _downscale_one(
    path_str: str,
    root_str: str,
    max_size: int,
    do_backup: bool,
    dry_run: bool,
) -> dict:
    """处理单张纹理。返回结果字典(供主进程汇总)。"""
    path = Path(path_str)
    root = Path(root_str)
    ext = path.suffix.lower()
    res = {
        "path": path_str,
        "status": "skipped",
        "reason": "",
        "old_wh": None,
        "new_wh": None,
        "old_bytes": 0,
        "new_bytes": 0,
    }

    try:
        old_bytes = path.stat().st_size
        res["old_bytes"] = old_bytes

        if ext in EXR_EXTS:
            w, h, ok = _process_exr(path, max_size, do_backup, dry_run, root, res)
        else:
            w, h, ok = _process_pillow(path, max_size, do_backup, dry_run, root, res)

        if not ok:
            return res

        res["status"] = "would-resize" if dry_run else "resized"
        if not dry_run:
            res["new_bytes"] = path.stat().st_size
        return res
    except Exception as e:
        res["status"] = "error"
        res["reason"] = f"{type(e).__name__}: {e}"
        return res


def _backup_file(path: Path, root: Path) -> None:
    """把原文件复制到 root/_orig_backup/<相对路径>(若已存在则不覆盖,保护首次原图)。"""
    rel = path.relative_to(root)
    dst = root / BACKUP_DIRNAME / rel
    if dst.exists():
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(path, dst)


def _process_pillow(
    path: Path, max_size: int, do_backup: bool, dry_run: bool, root: Path, res: dict
) -> tuple[int, int, bool]:
    from PIL import Image

    Image.MAX_IMAGE_PIXELS = None  # 关闭"图像过大"的安全告警/异常

    with Image.open(path) as im:
        w, h = im.size
        res["old_wh"] = (w, h)
        long_edge = max(w, h)
        if long_edge <= max_size:
            res["reason"] = f"{w}x{h} <= {max_size}"
            return w, h, False

        scale = max_size / long_edge
        nw = max(1, round(w * scale))
        nh = max(1, round(h * scale))
        res["new_wh"] = (nw, nh)

        if dry_run:
            return nw, nh, True

        if do_backup:
            _backup_file(path, root)

        im = im.convert(im.mode)  # 触发加载,保留原 mode(RGB/RGBA/L 等)
        resized = im.resize((nw, nh), Image.LANCZOS)

        save_kwargs = {}
        if path.suffix.lower() == ".png":
            save_kwargs["optimize"] = True
            save_kwargs["compress_level"] = 6
        elif path.suffix.lower() in (".jpg", ".jpeg"):
            save_kwargs["quality"] = 95

        resized.save(path, **save_kwargs)
    return nw, nh, True


def _process_exr(
    path: Path, max_size: int, do_backup: bool, dry_run: bool, root: Path, res: dict
) -> tuple[int, int, bool]:
    """EXR/HDR:保留浮点精度。本机 OpenCV 已编入 OpenEXR(需 OPENCV_IO_ENABLE_OPENEXR=1,
    已在模块顶部设置),用 cv2 读写最稳;float32 全程保留,不降位深。"""
    import numpy as np
    import cv2

    # IMREAD_UNCHANGED 保留原始通道与浮点精度
    arr = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if arr is None:
        raise RuntimeError(
            "cv2 无法读取 EXR(检查文件是否存在/损坏,或 OpenEXR 是否启用)"
        )
    if arr.ndim == 2:
        h, w = arr.shape
    else:
        h, w = arr.shape[0], arr.shape[1]
    res["old_wh"] = (w, h)

    long_edge = max(w, h)
    if long_edge <= max_size:
        res["reason"] = f"{w}x{h} <= {max_size}"
        return w, h, False

    scale = max_size / long_edge
    nw = max(1, round(w * scale))
    nh = max(1, round(h * scale))
    res["new_wh"] = (nw, nh)

    if dry_run:
        return nw, nh, True

    if do_backup:
        _backup_file(path, root)

    # cv2.resize 用 INTER_AREA 适合降采样;保持 float32 精度。
    # cv2.imwrite 对 .exr 默认即写 half/float,通道顺序与读入一致,无需转换。
    arr32 = arr.astype(np.float32)
    resized = cv2.resize(arr32, (nw, nh), interpolation=cv2.INTER_AREA)
    ok = cv2.imwrite(str(path), resized)
    if not ok:
        raise RuntimeError("cv2.imwrite 写入 EXR 失败")
    return nw, nh, True


# ---------------------------------------------------------------------------
# 扫描 / 主流程
# ---------------------------------------------------------------------------
def find_textures(root: Path) -> list[Path]:
    files: list[Path] = []
    for p in root.rglob("*"):
        if not p.is_file():
            continue
        if BACKUP_DIRNAME in p.parts:
            continue
        if p.suffix.lower() in ALL_EXTS:
            files.append(p)
    return sorted(files)


def _fmt_mb(n: int) -> str:
    return f"{n / 1024 / 1024:.1f}MB"


def do_restore(root: Path) -> int:
    backup_root = root / BACKUP_DIRNAME
    if not backup_root.is_dir():
        print(f"[RESTORE] 未找到备份目录: {backup_root}")
        return 1
    count = 0
    for src in backup_root.rglob("*"):
        if not src.is_file():
            continue
        rel = src.relative_to(backup_root)
        dst = root / rel
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        count += 1
    print(f"[RESTORE] 已从 {backup_root} 恢复 {count} 个原始纹理。")
    print(f"[RESTORE] 如需删除备份: rm -rf '{backup_root}'")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="批量降采样 USD 资产目录下的超大纹理,降低显存占用。"
    )
    parser.add_argument(
        "--root", required=True, help="要处理的纹理/资产根目录(递归扫描)"
    )
    parser.add_argument(
        "--max-size", type=int, default=2048,
        help="纹理长边上限(像素),超过则降采样到该值(默认 2048)",
    )
    parser.add_argument(
        "--workers", type=int, default=max(1, (os.cpu_count() or 4) // 2),
        help="并行进程数(默认 CPU 核数的一半)",
    )
    parser.add_argument(
        "--no-backup", action="store_true",
        help="不备份原图(默认会备份到 <root>/_orig_backup/)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="只统计将要修改的文件和预计变化,不实际写入",
    )
    parser.add_argument(
        "--restore", action="store_true",
        help="从 <root>/_orig_backup/ 恢复所有原始纹理,然后退出",
    )
    args = parser.parse_args()

    root = Path(args.root).expanduser().resolve()
    if not root.is_dir():
        print(f"[ERROR] 目录不存在: {root}", file=sys.stderr)
        return 1

    if args.restore:
        return do_restore(root)

    files = find_textures(root)
    print(f"[SCAN] 在 {root} 下发现 {len(files)} 张纹理(扫描扩展名: {sorted(ALL_EXTS)})")
    print(
        f"[CFG] max-size={args.max_size} workers={args.workers} "
        f"backup={'OFF' if args.no_backup else 'ON'} dry_run={args.dry_run}"
    )
    if not files:
        print("[SCAN] 没有纹理可处理。")
        return 0

    t0 = time.time()
    results: list[dict] = []
    do_backup = not args.no_backup

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {
            ex.submit(
                _downscale_one,
                str(f), str(root), args.max_size, do_backup, args.dry_run,
            ): f
            for f in files
        }
        done = 0
        total = len(futs)
        for fut in as_completed(futs):
            res = fut.result()
            results.append(res)
            done += 1
            if res["status"] in ("resized", "would-resize"):
                ow, oh = res["old_wh"]
                nw, nh = res["new_wh"]
                tag = "[DRY]" if args.dry_run else "[OK ]"
                print(
                    f"{tag} ({done}/{total}) {ow}x{oh} -> {nw}x{nh}  "
                    f"{Path(res['path']).name}"
                )
            elif res["status"] == "error":
                print(f"[ERR] ({done}/{total}) {res['path']}: {res['reason']}")

    # 汇总
    resized = [r for r in results if r["status"] in ("resized", "would-resize")]
    errors = [r for r in results if r["status"] == "error"]
    skipped = [r for r in results if r["status"] == "skipped"]

    old_sum = sum(r["old_bytes"] for r in resized)
    new_sum = sum(r["new_bytes"] for r in resized) if not args.dry_run else 0

    dt = time.time() - t0
    print("\n" + "=" * 64)
    print(f"[DONE] 用时 {dt:.1f}s  共 {len(files)} 张")
    print(
        f"  - 处理(降采样): {len(resized)}  跳过(已达标): {len(skipped)}  "
        f"错误: {len(errors)}"
    )
    if resized:
        if args.dry_run:
            print(f"  - 这些文件当前磁盘占用合计: {_fmt_mb(old_sum)}(实际显存收益更大)")
            print("  - 这是预览模式,未写入。去掉 --dry-run 即可实际执行。")
        else:
            saved = old_sum - new_sum
            print(
                f"  - 磁盘: {_fmt_mb(old_sum)} -> {_fmt_mb(new_sum)} "
                f"(省 {_fmt_mb(saved)})"
            )
            if do_backup:
                print(f"  - 原图已备份到: {root / BACKUP_DIRNAME}")
                print(f"  - 回滚: ./app/python.sh tools/usd/downscale_textures.py --root '{root}' --restore")
    if errors:
        print("  - 出错文件:")
        for r in errors:
            print(f"      {r['path']}: {r['reason']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
