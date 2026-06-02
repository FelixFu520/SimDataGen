#!/usr/bin/env python3
"""
将各相机的 mask 叠加到 RGB 与深度可视化图上,用于检查 mask 对齐是否正确。

用法示例:
  ./app/python.sh tools/check_data/overlay_mask_verify.py \\
    --base /home/fufa/isaac_sim/workdir/omni/debug \\
    --frame 0000_0000

目录约定(与当前 debug 输出一致):
  {base}/mask/CAM_X_mask.png
  {base}/rgb/CAM_X/{frame}.jpg
  {base}/depth/CAM_X/{frame}.png  或 .npy(见 --depth-source)
"""

from __future__ import annotations

import argparse
import os
import sys

import cv2
import numpy as np

CAMERAS = ("CAM_A", "CAM_B", "CAM_C", "CAM_D")


def _read_mask(path: str) -> np.ndarray:
    m = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    if m is None:
        raise FileNotFoundError(f"无法读取 mask: {path}")
    if m.ndim == 3:
        m = cv2.cvtColor(m, cv2.COLOR_BGR2GRAY)
    return m.astype(np.float32)


def _to_binary_mask(m: np.ndarray, thr: float) -> np.ndarray:
    if m.max() <= 1.5:
        return (m >= thr).astype(np.float32)
    return (m >= thr).astype(np.float32)


def _resize_mask(mask_f: np.ndarray, h: int, w: int) -> np.ndarray:
    if mask_f.shape[0] == h and mask_f.shape[1] == w:
        return mask_f
    return cv2.resize(mask_f, (w, h), interpolation=cv2.INTER_NEAREST)


def _overlay_color_on_bgr(
    bgr: np.ndarray, mask01: np.ndarray, bgr_color: tuple[float, float, float], alpha: float
) -> np.ndarray:
    """mask 区域叠加上色;alpha 越小着色越透明,原图越清晰。"""
    out = bgr.astype(np.float32)
    color = np.array(bgr_color, dtype=np.float32).reshape(1, 1, 3)
    m = mask01[..., None]
    blend = alpha * m
    out = out * (1.0 - blend) + color * blend
    return np.clip(out, 0, 255).astype(np.uint8)


def _depth_to_vis_bgr(depth_u8: np.ndarray) -> np.ndarray:
    d = depth_u8.astype(np.float32)
    dnorm = cv2.normalize(d, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    vis = cv2.applyColorMap(dnorm, cv2.COLORMAP_TURBO)
    return vis


def _draw_mask_contour(bgr: np.ndarray, mask01: np.ndarray, bgr_color: tuple[int, int, int], thickness: int = 2) -> np.ndarray:
    out = bgr.copy()
    cnts, _ = cv2.findContours(
        (mask01 > 0.5).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if cnts:
        cv2.drawContours(out, cnts, -1, bgr_color, thickness)
    return out


def _rgb_valid(bgr: np.ndarray, thr: float = 5.0) -> np.ndarray:
    return (np.max(bgr, axis=2) > thr).astype(np.float32)


def _depth_valid_from_npy(arr: np.ndarray) -> np.ndarray:
    d = arr.astype(np.float32)
    return (np.isfinite(d) & (d > 0.01) & (d < 1000.0)).astype(np.float32)


def _alignment_stats(mask01: np.ndarray, valid01: np.ndarray) -> dict[str, int | float]:
    inter = int(np.sum(mask01 * valid01))
    mask_only = int(np.sum((mask01 > 0.5) & (valid01 < 0.5)))
    data_only = int(np.sum((mask01 < 0.5) & (valid01 > 0.5)))
    union = int(np.sum((mask01 > 0.5) | (valid01 > 0.5)))
    iou = inter / union if union else 1.0
    return {
        "iou": iou,
        "mask_only": mask_only,
        "data_only": data_only,
        "inter": inter,
    }


def _depth_npy_to_vis_bgr(arr: np.ndarray) -> np.ndarray:
    d = np.nan_to_num(arr.astype(np.float32), nan=0.0, posinf=0.0, neginf=0.0)
    if d.size == 0:
        return np.zeros((1, 1, 3), dtype=np.uint8)
    lo, hi = np.percentile(d[d > 0], [2, 98]) if np.any(d > 0) else (0.0, 1.0)
    if hi <= lo:
        hi = lo + 1e-6
    dn = np.clip((d - lo) / (hi - lo), 0.0, 1.0)
    u8 = (dn * 255.0).astype(np.uint8)
    return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)


def main() -> int:
    p = argparse.ArgumentParser(description="mask 与 RGB/深度叠加可视化")
    p.add_argument("--base", type=str, default=None, help="debug 根目录(其下含 mask/ rgb/ depth/)",)
    p.add_argument("--frame", type=str, default="", help="帧 stem,如 0000_0000;默认取 rgb 目录下第一张")
    p.add_argument("--out", type=str, default=None, help="输出目录,默认 {base}/overlay_verify",)
    p.add_argument(
        "--alpha",
        type=float,
        default=0.22,
        help="mask 区域着色不透明度 0~1,越小越透明、越容易看清底下 RGB/深度",
    )
    p.add_argument("--thr", type=float, default=127.0, help="mask 二值阈值(uint8 图常用 127)")
    p.add_argument(
        "--depth-source",
        choices=("png", "npy"),
        default="npy",
        help="深度可视化源:npy(推荐,与有效深度一致)或 png",
    )
    args = p.parse_args()
    base = os.path.abspath(args.base)
    out_dir = os.path.abspath(args.out) if args.out else os.path.join(base, "overlay_verify")
    os.makedirs(out_dir, exist_ok=True)

    # 推断默认 frame
    frame = args.frame.strip()
    if not frame:
        sample_rgb = os.path.join(base, "rgb", CAMERAS[0])
        if not os.path.isdir(sample_rgb):
            print(f"找不到 RGB 目录: {sample_rgb}", file=sys.stderr)
            return 1
        jpgs = sorted(f for f in os.listdir(sample_rgb) if f.lower().endswith((".jpg", ".jpeg")))
        if not jpgs:
            print(f"{sample_rgb} 下没有 jpg", file=sys.stderr)
            return 1
        frame = os.path.splitext(jpgs[0])[0]

    alpha = float(np.clip(args.alpha, 0.0, 1.0))

    for cam in CAMERAS:
        mask_path = os.path.join(base, "mask", f"{cam}_mask.png")
        rgb_path = os.path.join(base, "rgb", cam, f"{frame}.jpg")
        if not os.path.isfile(rgb_path):
            rgb_path = os.path.join(base, "rgb", cam, f"{frame}.jpeg")

        if args.depth_source == "png":
            depth_path = os.path.join(base, "depth", cam, f"{frame}.png")
        else:
            depth_path = os.path.join(base, "depth", cam, f"{frame}.npy")

        if not os.path.isfile(mask_path):
            print(f"跳过 {cam}: 缺少 mask {mask_path}", file=sys.stderr)
            continue
        if not os.path.isfile(rgb_path):
            print(f"跳过 {cam}: 缺少 RGB {rgb_path}", file=sys.stderr)
            continue
        if not os.path.isfile(depth_path):
            print(f"跳过 {cam}: 缺少 depth {depth_path}", file=sys.stderr)
            continue

        mask_f = _read_mask(mask_path)
        mask_bin = _to_binary_mask(mask_f, args.thr)

        bgr = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        if bgr is None:
            print(f"跳过 {cam}: 无法读 RGB {rgb_path}", file=sys.stderr)
            continue

        h, w = bgr.shape[:2]
        m = _resize_mask(mask_bin, h, w)
        rgb_valid = _rgb_valid(bgr)

        # 仅在有效 RGB/depth 区域着色,避免黑角被染色造成“假错位”
        rgb_overlay = _overlay_color_on_bgr(bgr, m * rgb_valid, (0, 255, 0), alpha)
        rgb_overlay = _draw_mask_contour(rgb_overlay, m, (0, 0, 255), thickness=2)
        cv2.imwrite(os.path.join(out_dir, f"{cam}_{frame}_rgb_mask.png"), rgb_overlay)

        depth_arr: np.ndarray | None = None
        if args.depth_source == "png":
            d = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
            if d is None:
                print(f"跳过 {cam}: 无法读 depth png {depth_path}", file=sys.stderr)
                continue
            if d.ndim == 3:
                d = cv2.cvtColor(d, cv2.COLOR_BGR2GRAY)
            d_vis = _depth_to_vis_bgr(d.astype(np.uint8))
            depth_valid = (d.astype(np.float32) > 0).astype(np.float32)
        else:
            depth_arr = np.load(depth_path)
            if depth_arr.ndim != 2:
                depth_arr = np.squeeze(depth_arr)
            depth_valid = _depth_valid_from_npy(depth_arr)
            d_vis = _depth_npy_to_vis_bgr(depth_arr)

        if d_vis.shape[0] != h or d_vis.shape[1] != w:
            d_vis = cv2.resize(d_vis, (w, h), interpolation=cv2.INTER_NEAREST)
        if depth_valid.shape[0] != h or depth_valid.shape[1] != w:
            depth_valid = cv2.resize(depth_valid, (w, h), interpolation=cv2.INTER_NEAREST)

        md = _resize_mask(mask_bin, h, w)
        depth_overlay = _overlay_color_on_bgr(d_vis, md * depth_valid, (0, 255, 255), alpha)
        depth_overlay = _draw_mask_contour(depth_overlay, md, (0, 0, 255), thickness=2)
        cv2.imwrite(os.path.join(out_dir, f"{cam}_{frame}_depth_mask.png"), depth_overlay)

        rgb_stats = _alignment_stats(m, rgb_valid)
        depth_stats = _alignment_stats(md, depth_valid)
        print(
            f"{cam}: RGB IoU={rgb_stats['iou']:.4f} "
            f"mask-only={rgb_stats['mask_only']} rgb-only={rgb_stats['data_only']} | "
            f"depth IoU={depth_stats['iou']:.4f} "
            f"mask-only={depth_stats['mask_only']} depth-only={depth_stats['data_only']}"
        )

        row = np.hstack([bgr, rgb_overlay, d_vis, depth_overlay])
        cv2.imwrite(os.path.join(out_dir, f"{cam}_{frame}_quad.png"), row)

    print(f"完成。输出目录: {out_dir}  frame={frame}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
