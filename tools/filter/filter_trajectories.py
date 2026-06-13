#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
过滤采集数据中不可用的轨迹。

数据组织:
    <root>/<场景文件夹>/rgb/<相机>/xxxx_yyyy.jpg
        xxxx -> 轨迹序号 (trajectory id)
        yyyy -> 该轨迹中的采集点序号 (frame id)

判定规则:
    对每张图, 只统计鱼眼有效圆形区域(去掉四角固定背景), 计算:
        - 对比度(灰度标准差)
        - 平均饱和度
        - 近白像素占比 / 近黑像素占比
    若一张图 "色彩单一 / 信息量低", 则记为坏图, 包含以下任一情况:
        1) 近黑占比 >= black_hard(硬阈值): 纯黑死区占主导, 直接判坏
           (不要求低对比/低饱和, 避免"大面积纯黑+一道过曝高对比/带色条带"
            把 std/sat 抬高从而绕过判定)
        2) 低对比 且 低饱和 (灰蒙蒙、单色一片)
        3) 大面积近白 且 低饱和 (基本全白)
        4) 大面积近黑 且 低对比 且 低饱和 (偏黑且无内容)
        5) 近黑+近白占比极高 且 低饱和 (黑白两极化, 如黑底+过曝白物体)
    一条轨迹中坏图比例 >= bad_ratio(默认0.4) 则判为不可用 -> discard, 否则 -> save。

    另外, 一条轨迹中点(帧)数量 < min_points(默认30) 也直接判为不可用 -> discard。

输出:
    <root目录名>.csv  每条轨迹一行, 含 scene/traj_id/total/bad/bad_ratio/decision/reason
        reason: discard 原因, few_points(点数过少) / bad_ratio(坏图过多), save 时为空

用法:
    python tools/filter/filter_trajectories.py \
        --root workdir_taobao08_01 \
        --out-dir workdir_filter \
        --cameras CAM_A \
        --workers 16
"""

import os
import re
import glob
import argparse
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None

IMG_NAME_RE = re.compile(r"^(\d+)_(\d+)\.(jpg|jpeg|png)$", re.IGNORECASE)


# ----------------------------- 单图特征/判定 ----------------------------- #
def _circular_mask(h, w, ratio=0.97):
    """鱼眼有效区域圆形掩码, 去掉四角固定背景。"""
    yy, xx = np.ogrid[:h, :w]
    cy, cx = (h - 1) / 2.0, (w - 1) / 2.0
    r = min(h, w) / 2.0 * ratio
    return (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r


def image_features(path, size=128):
    """返回单张图的颜色统计特征, 失败返回 None。"""
    try:
        im = Image.open(path)
        rgb = np.asarray(im.convert("RGB").resize((size, size))).astype(np.float32)
        hsv = np.asarray(im.convert("HSV").resize((size, size))).astype(np.float32)
    except Exception:
        return None

    mask = _circular_mask(size, size)
    rgb_v = rgb[mask]              # (N, 3)
    sat_v = hsv[mask][:, 1]        # (N,)
    gray = rgb_v.mean(axis=1)      # (N,)

    return {
        "std": float(gray.std()),                 # 灰度对比度
        "sat": float(sat_v.mean()),               # 平均饱和度
        "white": float((gray > 235).mean()),      # 近白占比
        "black": float((gray < 18).mean()),       # 近黑占比
    }


def is_bad_image(f, args):
    """根据特征判断单张图是否为坏图(色彩单一/信息量低)。"""
    if f is None:
        # 读取失败的图视为坏图
        return True
    low_contrast = f["std"] < args.std_thresh
    low_sat = f["sat"] < args.sat_thresh

    # 1) 大面积近黑死区(硬性): 纯黑占主导 -> 必然无有效内容, 直接判坏
    #    不再要求同时低对比/低饱和: 否则"大面积纯黑 + 一条过曝高对比/带色条带"
    #    会把 std/sat 抬高从而绕过判定(如黑墙 + 一道强光缝隙的空洞画面)
    if f["black"] >= args.black_hard:
        return True
    # 2) 低对比 且 低饱和: 灰蒙蒙 / 单色一片
    if low_contrast and low_sat:
        return True
    # 3) 大面积近白 且 低饱和: 基本全白
    if f["white"] >= args.white_ratio and low_sat:
        return True
    # 4) 大面积近黑 且 低对比 且 低饱和: 基本全黑、无内容
    #    (低于 black_hard 但仍偏黑, 配合低对比/低饱和才判坏,
    #     避免误杀"暗调但有丰富色彩/灯光内容"的图, 如暖光服装店)
    if f["black"] >= args.black_ratio and low_contrast and low_sat:
        return True
    # 5) 近黑+近白占比极高 且 低饱和: 黑白两极化、几乎无中间调与色彩
    #    (如黑底背景 + 过曝纯白植被/物体, 整图被纯黑/纯白瓜分, 细节丢失)
    if (f["white"] + f["black"]) >= args.bw_ratio and low_sat:
        return True
    return False


# ----------------------------- 扫描数据 ----------------------------- #
def collect_trajectories(root, cameras):
    """
    返回: { (scene, traj_id): [图片路径, ...] }
    合并指定相机下同一轨迹序号的所有帧。
    """
    traj_imgs = defaultdict(list)
    scenes = sorted(
        d for d in os.listdir(root)
        if os.path.isdir(os.path.join(root, d))
    )
    for scene in scenes:
        rgb_dir = os.path.join(root, scene, "rgb")
        if not os.path.isdir(rgb_dir):
            continue
        if cameras:
            cam_dirs = [os.path.join(rgb_dir, c) for c in cameras]
        else:
            cam_dirs = [
                os.path.join(rgb_dir, c) for c in sorted(os.listdir(rgb_dir))
                if os.path.isdir(os.path.join(rgb_dir, c))
            ]
        for cam_dir in cam_dirs:
            if not os.path.isdir(cam_dir):
                continue
            for name in os.listdir(cam_dir):
                m = IMG_NAME_RE.match(name)
                if not m:
                    continue
                traj_id = m.group(1)
                traj_imgs[(scene, traj_id)].append(os.path.join(cam_dir, name))
    return traj_imgs


# ----------------------------- 并行处理一条轨迹 ----------------------------- #
def _process_traj(payload):
    scene, traj_id, paths, args_dict = payload
    args = argparse.Namespace(**args_dict)
    total = 0
    bad = 0
    for p in paths:
        f = image_features(p, size=args.size)
        if is_bad_image(f, args):
            bad += 1
        total += 1
    bad_ratio = bad / total if total else 1.0
    # 1) 点数过少 -> 丢弃; 2) 坏图比例过高 -> 丢弃
    too_few = total < args.min_points
    discard = too_few or bad_ratio >= args.bad_ratio
    if not discard:
        reason = ""
    elif too_few:
        reason = "few_points"
    else:
        reason = "bad_ratio"
    return scene, traj_id, total, bad, bad_ratio, discard, reason


def main():
    ap = argparse.ArgumentParser(description="过滤不可用轨迹(色彩单一/全白/全黑)")
    ap.add_argument("--root", default="workdir_taobao08_01", help="数据根目录")
    ap.add_argument("--out-dir", default="tools/filter", help="输出csv目录")
    ap.add_argument("--cameras", nargs="*", default=["CAM_A"],
                    help="参与统计的相机, 留空表示该轨迹下所有相机")
    ap.add_argument("--bad-ratio", type=float, default=0.4,
                    help="轨迹中坏图比例阈值, >= 则丢弃")
    ap.add_argument("--min-points", type=int, default=40,
                    help="轨迹中点(帧)数量阈值, < 则整条轨迹丢弃")
    ap.add_argument("--std-thresh", type=float, default=20.0, help="对比度阈值")
    ap.add_argument("--sat-thresh", type=float, default=8.0, help="饱和度阈值")
    ap.add_argument("--white-ratio", type=float, default=0.55, help="近白占比阈值")
    ap.add_argument("--black-ratio", type=float, default=0.55, help="近黑占比阈值")
    ap.add_argument("--black-hard", type=float, default=0.60,
                    help="近黑占比硬阈值, >= 则直接判坏(纯黑死区占主导, 无需低对比/低饱和)")
    ap.add_argument("--bw-ratio", type=float, default=0.85,
                    help="近黑+近白占比阈值(黑白两极化), >= 且低饱和则丢弃")
    ap.add_argument("--size", type=int, default=128, help="缩放后边长(加速统计)")
    ap.add_argument("--workers", type=int, default=16, help="并行进程数")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    print(f"[1/3] 扫描数据: root={args.root} cameras={args.cameras or '全部'}")
    traj_imgs = collect_trajectories(args.root, args.cameras)
    print(f"      共找到 {len(traj_imgs)} 条轨迹")

    args_dict = vars(args)
    payloads = [
        (scene, tid, paths, args_dict)
        for (scene, tid), paths in sorted(traj_imgs.items())
    ]

    print(f"[2/3] 并行分析 (workers={args.workers}) ...")
    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(_process_traj, p) for p in payloads]
        done = 0
        for fut in as_completed(futs):
            results.append(fut.result())
            done += 1
            if done % 50 == 0 or done == len(futs):
                print(f"      {done}/{len(futs)}")

    results.sort(key=lambda r: (r[0], r[1]))

    root_name = os.path.basename(os.path.normpath(args.root))
    csv_path = os.path.join(args.out_dir, f"{root_name}.csv")
    n_save = n_discard = 0
    with open(csv_path, "w") as f:
        f.write("scene,traj_id,total,bad,bad_ratio,decision,reason\n")
        for scene, tid, total, bad, ratio, discard, reason in results:
            decision = "discard" if discard else "save"
            f.write(f"{scene},{tid},{total},{bad},{ratio:.4f},{decision},{reason}\n")
            if discard:
                n_discard += 1
            else:
                n_save += 1

    print(f"[3/3] 完成: 保留 {n_save} 条, 丢弃 {n_discard} 条")
    print(f"      -> {csv_path}")


if __name__ == "__main__":
    main()
