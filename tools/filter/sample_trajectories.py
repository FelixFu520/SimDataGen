#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从过滤结果 csv 或 txt 中的轨迹按帧号顺序每隔 step 帧抽帧, 把每条轨迹抽出的
若干帧拼接成一张网格大图(一条轨迹一张), 底部写上"场景文件夹名 + 轨迹序号";
可选地把所有轨迹的拼接图合并到一个视频, 每帧即一条轨迹的拼接图。

列表文件格式:
    csv: scene,traj_id,...,decision  (可用 --decision 按 save/discard 筛选)
    txt: 每行 "<场景文件夹名> <轨迹序号>"

数据组织:
    <root>/<场景文件夹>/rgb/<相机>/<轨迹序号>_<帧号>.jpg

用法示例:
    # 每条轨迹每隔10帧抽帧, 拼成一张网格大图
    python tools/filter/sample_trajectories.py \
        --list tools/filter/filter_discard.txt \
        --root workdir_taobao08_01 \
        --out-dir tools/filter/sample_discard \
        --step 10

    # 拼接图 + 把所有轨迹的拼接图合并到一个视频(每帧=一条轨迹拼接图)
    python tools/filter/sample_trajectories.py \
        --list tools/filter/filter_discard.txt \
        --root workdir_taobao08_01 \
        --out-dir tools/filter/sample_discard \
        --step 10 --video
"""

import os
import csv
import glob
import math
import argparse

import cv2
import numpy as np


def list_traj_images(root, scene, traj_id, camera):
    """返回某条轨迹按帧号排序的图片路径列表。"""
    cam_dir = os.path.join(root, scene, "rgb", camera)
    paths = glob.glob(os.path.join(cam_dir, f"{traj_id}_*.jpg"))
    paths += glob.glob(os.path.join(cam_dir, f"{traj_id}_*.jpeg"))
    paths += glob.glob(os.path.join(cam_dir, f"{traj_id}_*.png"))
    paths.sort()
    return paths


def read_list(list_path, decision=None):
    """读取 csv 或 txt, 返回 [(scene, traj_id), ...]"""
    items = []
    if list_path.lower().endswith(".csv"):
        with open(list_path, newline="") as f:
            for row in csv.DictReader(f):
                if decision and row.get("decision") != decision:
                    continue
                scene = row.get("scene", "").strip()
                traj_id = row.get("traj_id", "").strip()
                if scene and traj_id:
                    items.append((scene, traj_id))
        return items

    with open(list_path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 2:
                continue
            items.append((parts[0], parts[1]))
    return items


def draw_label(img, text):
    """在图片底部中间绘制带半透明背景的文字标签。"""
    h, w = img.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = max(0.6, w / 1280.0)
    thickness = max(1, int(round(scale * 2)))
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)

    pad = int(10 * scale)
    x = (w - tw) // 2
    y = h - pad - baseline

    # 半透明黑底
    overlay = img.copy()
    cv2.rectangle(
        overlay,
        (x - pad, y - th - pad),
        (x + tw + pad, y + baseline + pad),
        (0, 0, 0),
        -1,
    )
    cv2.addWeighted(overlay, 0.5, img, 0.5, 0, img)
    cv2.putText(img, text, (x, y), font, scale, (255, 255, 255),
                thickness, cv2.LINE_AA)
    return img


def concat_images(paths, cell=320, cols=None, pad=4, bg=(20, 20, 20)):
    """把多张图按网格拼成一张大图。

    paths: 图片路径列表(已按时间顺序)
    cell:  每个单元格(缩放后)的边长(像素)
    cols:  网格列数, None 时自动取接近正方形
    返回拼接后的 BGR 大图(np.ndarray), 失败返回 None。
    """
    imgs = []
    for p in paths:
        im = cv2.imread(p)
        if im is not None:
            imgs.append(im)
    if not imgs:
        return None

    n = len(imgs)
    if cols is None:
        cols = max(1, int(math.ceil(math.sqrt(n))))
    rows = int(math.ceil(n / cols))

    # 每张图等比缩放后居中放入 cell x cell 的格子
    canvas = np.full(
        (rows * cell + (rows + 1) * pad, cols * cell + (cols + 1) * pad, 3),
        bg, dtype=np.uint8,
    )
    for idx, im in enumerate(imgs):
        r, c = divmod(idx, cols)
        h, w = im.shape[:2]
        scale = min(cell / w, cell / h)
        nw, nh = max(1, int(w * scale)), max(1, int(h * scale))
        im_r = cv2.resize(im, (nw, nh))
        y0 = pad + r * (cell + pad) + (cell - nh) // 2
        x0 = pad + c * (cell + pad) + (cell - nw) // 2
        canvas[y0:y0 + nh, x0:x0 + nw] = im_r
    return canvas


def make_video(frame_paths, out_path, fps, video_size=None):
    """把已生成的成品帧(每帧=一条轨迹的拼接图)拼成一个视频。

    frame_paths: [图片路径, ...] —— 顺序即播放顺序, 图片已带标注。
    各帧尺寸以第一帧为准, 不一致的统一 resize。
    """
    if not frame_paths:
        return False

    first = None
    for p in frame_paths:
        first = cv2.imread(p)
        if first is not None:
            break
    if first is None:
        return False
    if video_size:
        w, h = video_size
    else:
        h, w = first.shape[:2]

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
    n = 0
    for p in frame_paths:
        img = cv2.imread(p)
        if img is None:
            continue
        if (img.shape[1], img.shape[0]) != (w, h):
            img = cv2.resize(img, (w, h))
        writer.write(img)
        n += 1
    writer.release()
    return n > 0


def main():
    ap = argparse.ArgumentParser(
        description="按轨迹每隔 step 帧抽帧拼成一张网格大图, 可选拼视频")
    ap.add_argument("--list", required=True,
                    help="过滤结果 csv 或 txt 列表文件")
    ap.add_argument("--decision", choices=["save", "discard"], default=None,
                    help="读取 csv 时按 decision 列筛选, 如 discard")
    ap.add_argument("--root", default="workdir_taobao08_01", help="数据根目录")
    ap.add_argument("--out-dir", required=True, help="输出目录")
    ap.add_argument("--camera", default="CAM_A", help="使用的相机")
    ap.add_argument("--step", type=int, default=10,
                    help="每隔多少帧取一帧(按帧号顺序), 默认10")
    ap.add_argument("--cell", type=int, default=320,
                    help="拼接图中每个单元格的边长(像素), 默认320")
    ap.add_argument("--cols", type=int, default=0,
                    help="拼接网格列数, 0 表示自动取接近正方形")
    ap.add_argument("--video", action="store_true",
                    help="是否把所有轨迹的拼接图合并到一个视频(每帧=一条轨迹拼接图)")
    ap.add_argument("--fps", type=float, default=2.0, help="视频帧率")
    ap.add_argument("--video-dir", default=None,
                    help="视频输出目录, 默认 <out-dir>/videos")
    ap.add_argument("--video-name", default="all.mp4",
                    help="合并视频的文件名, 默认 all.mp4")
    args = ap.parse_args()

    step = max(1, args.step)
    cols = args.cols if args.cols and args.cols > 0 else None

    img_out_dir = os.path.join(args.out_dir, "images")
    os.makedirs(img_out_dir, exist_ok=True)
    if args.video:
        video_dir = args.video_dir or os.path.join(args.out_dir, "videos")
        os.makedirs(video_dir, exist_ok=True)

    items = read_list(args.list, args.decision)
    print(f"[1/2] 读取轨迹列表: {args.list} -> {len(items)} 条轨迹")

    n_grid = 0
    n_skip = 0
    video_frames = []  # [拼接图路径, ...] 每条轨迹一张拼接图(作为视频一帧)
    for scene, traj_id in items:
        all_imgs = list_traj_images(args.root, scene, traj_id, args.camera)
        if not all_imgs:
            n_skip += 1
            print(f"  [跳过] 无图片: {scene} {traj_id}")
            continue

        # 每隔 step 帧取一帧(按帧号顺序), 拼成一张网格大图
        picked = all_imgs[::step]
        grid = concat_images(picked, cell=args.cell, cols=cols)
        if grid is None:
            n_skip += 1
            print(f"  [跳过] 无可读图片: {scene} {traj_id}")
            continue

        label = f"{scene}  traj {traj_id}  ({len(picked)} frames)"
        draw_label(grid, label)
        out_name = f"{scene}_{traj_id}.jpg"
        out_path = os.path.join(img_out_dir, out_name)
        cv2.imwrite(out_path, grid)
        n_grid += 1
        if args.video:
            video_frames.append(out_path)

    print(f"[2/2] 完成: 生成拼接图 {n_grid} 张 -> {img_out_dir}")
    if args.video:
        out_video = os.path.join(video_dir, args.video_name)
        if make_video(video_frames, out_video, args.fps):
            print(f"      合并视频 {len(video_frames)} 帧 -> {out_video}")
        else:
            print(f"      未生成视频(无可用帧)")
    if n_skip:
        print(f"      跳过(无图片) {n_skip} 条")


if __name__ == "__main__":
    main()
