#!/usr/bin/env python3
"""
将深度 .npy 转为可视化灰度图：过滤非有限值与大于阈值的像素，对剩余像素做 min-max 归一化后保存。

用法示例:
  python tools/check_data/depth_npy_filter_normalize.py \
    --depth_path /root/vepfs/isaacsim/DataGen_omni/0000_0000.npy \
    --output_path /root/vepfs/isaacsim/DataGen_omni/0000_0000_filtered_norm.png
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def load_depth_2d(path: Path) -> tuple[np.ndarray, np.dtype]:
    d = np.load(path)
    orig_dtype = d.dtype
    if d.ndim == 3:
        d = d[..., 0]
    if d.ndim != 2:
        raise ValueError(f"期望 HxW 或 HxWxC 深度数组，当前 shape={d.shape}")
    return d.astype(np.float64, copy=False), orig_dtype


def filter_and_normalize(
    depth: np.ndarray,
    max_depth: float,
) -> tuple[np.ndarray, np.ndarray]:
    """有效掩码：有限且 depth <= max_depth;返回 (uint8 灰度图 HxW, valid_mask)。"""
    valid = np.isfinite(depth) & (depth <= max_depth)
    out = np.zeros(depth.shape, dtype=np.uint8)
    if not valid.any():
        return out, valid
    v = depth[valid]
    mn = float(v.min())
    mx = float(v.max())
    if mx > mn:
        out[valid] = ((depth[valid] - mn) / (mx - mn) * 255.0).astype(np.uint8)
    else:
        out[valid] = 128
    return out, valid


def save_gray_png(path: Path, img: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image

        Image.fromarray(img, mode="L").save(path)
    except ImportError:
        import cv2

        cv2.imwrite(str(path), img)


def main() -> None:
    parser = argparse.ArgumentParser(description="深度 npy：过滤 >阈值 与非有限值，归一化后存 PNG")
    parser.add_argument("--depth_path", type=Path, required=True, help="输入 .npy 路径")
    parser.add_argument(
        "--output_path",
        type=Path,
        default=None,
        help="输出 PNG;默认与输入同目录，文件名加 _filtered_norm.png",
    )
    parser.add_argument(
        "--max_depth",
        type=float,
        default=100.0,
        help="大于该值的像素视为无效(不参与归一化，输出为 0)",
    )
    args = parser.parse_args()

    depth_path: Path = args.depth_path
    if not depth_path.is_file():
        raise SystemExit(f"找不到文件: {depth_path}")

    out_path = args.output_path
    if out_path is None:
        out_path = depth_path.with_name(depth_path.stem + "_filtered_norm.png")

    depth, orig_dtype = load_depth_2d(depth_path)
    img, valid = filter_and_normalize(depth, args.max_depth)
    save_gray_png(out_path, img)

    n_valid = int(valid.sum())
    n_all = int(valid.size)
    print(f"读取 {depth_path} shape={depth.shape} dtype={orig_dtype}")
    print(f"有效像素 {n_valid} / {n_all}(max_depth={args.max_depth}，非有限或 >max_depth 已过滤为黑)")
    print(f"已保存 {out_path}")


if __name__ == "__main__":
    main()
