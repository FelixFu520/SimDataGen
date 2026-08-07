#!/usr/bin/env python3
"""对比采集数据 common/*.npy 中的相机内外参与 Kalibr 标定 yaml 是否一致。

用途:验证 bake 进 USD 的内参、以及 USD 相机组的外参,是否与重新标定的
`fisheye_cams.yaml` 保持一致。

坐标系说明(重要):
  - Kalibr yaml 的 `T_cam_imu` 是 imu->cam,相机系为 OpenCV 约定 (X右 Y下 Z前)。
  - common/*.npy 的 `extrinsics_camera[A][B]` 是 OpenCV 约定下 camA->camB。
  - 因此二者的"相机间相对位姿"可直接比较:
        T_camj_cami(yaml) = T_cam_imu[j] @ inv(T_cam_imu[i])
  - 不直接比较 `extrinsics_world`(它随相机组每帧世界位姿变化,与标定无关)。

Usage:
    ./app/python.sh tools/check_data/compare_common_vs_calibration.py \\
        --data_dir workdir/home_000_oak_camera_4lut_2H110SA_20260806 \\
        --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml
"""

from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import yaml

CAM_KEY_TO_NAME = {
    "cam0": "CAM_A",
    "cam1": "CAM_B",
    "cam2": "CAM_C",
    "cam3": "CAM_D",
}

# 一致性判定阈值
TOL_FOCAL_PX = 0.1      # fx/fy/cx/cy 像素
TOL_XI = 1e-4
TOL_DIST = 1e-4
TOL_ROT_DEG = 0.2       # 相机间相对旋转
TOL_TRANS_M = 0.002     # 相机间相对平移 (米)


def load_yaml(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    cams = {}
    for key, cam in data.items():
        if cam.get("camera_model") != "omni":
            continue
        name = CAM_KEY_TO_NAME.get(key, cam.get("rostopic", key))
        intr = cam["intrinsics"]
        cams[name] = {
            "xi": float(intr[0]),
            "fx": float(intr[1]),
            "fy": float(intr[2]),
            "cx": float(intr[3]),
            "cy": float(intr[4]),
            "dist": [float(v) for v in cam["distortion_coeffs"]],
            "T_cam_imu": np.array(cam["T_cam_imu"], dtype=np.float64),
            "res": (int(cam["resolution"][0]), int(cam["resolution"][1])),
        }
    return cams


def find_first_common(data_dir):
    common_dir = os.path.join(data_dir, "common")
    if not os.path.isdir(common_dir):
        raise FileNotFoundError(f"未找到 common 目录: {common_dir}")
    files = sorted(f for f in os.listdir(common_dir) if f.endswith(".npy"))
    if not files:
        raise FileNotFoundError(f"common 目录内没有 .npy: {common_dir}")
    return os.path.join(common_dir, files[0])


def rot_angle_deg(Ra, Rb):
    R = Ra @ Rb.T
    c = (np.trace(R) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return float(np.degrees(np.arccos(c)))


def fmt_ok(ok):
    return "一致 ✓" if ok else "不一致 ✗"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--data_dir", required=True, help="采集数据根目录(含 common/)")
    ap.add_argument("--yaml", required=True, help="Kalibr omni 标定 yaml")
    ap.add_argument("--frame", default=None, help="指定 common 帧名(默认取第一帧)")
    args = ap.parse_args()

    yaml_cams = load_yaml(args.yaml)
    if args.frame:
        npy_path = os.path.join(args.data_dir, "common", args.frame)
        if not npy_path.endswith(".npy"):
            npy_path += ".npy"
    else:
        npy_path = find_first_common(args.data_dir)

    print(f"标定 yaml : {os.path.abspath(args.yaml)}")
    print(f"common npy: {os.path.abspath(npy_path)}")
    common = np.load(npy_path, allow_pickle=True).item()

    fisheye = [c for c in ("CAM_A", "CAM_B", "CAM_C", "CAM_D") if c in yaml_cams]

    # ---------------- 内参对比 ----------------
    print("\n" + "=" * 78)
    print("内参对比 (common intrinsics_full.calibration  vs  yaml intrinsics)")
    print("=" * 78)
    intr_all_ok = True
    for name in fisheye:
        y = yaml_cams[name]
        if name not in common:
            print(f"[{name}] common 中缺失,跳过")
            intr_all_ok = False
            continue
        cal = common[name].get("intrinsics_full", {}).get("calibration")
        if cal is None:
            print(f"[{name}] common 无 calibration 内参(未 bake?),跳过")
            intr_all_ok = False
            continue
        rows = [
            ("xi", cal["xi"], y["xi"], TOL_XI),
            ("fx", cal["fx_px"], y["fx"], TOL_FOCAL_PX),
            ("fy", cal["fy_px"], y["fy"], TOL_FOCAL_PX),
            ("cx", cal["cx_px"], y["cx"], TOL_FOCAL_PX),
            ("cy", cal["cy_px"], y["cy"], TOL_FOCAL_PX),
        ]
        dist_c = list(cal.get("distortion_coeffs", []))
        for i, dn in enumerate(("k1", "k2", "p1", "p2")):
            cv = dist_c[i] if i < len(dist_c) else 0.0
            rows.append((dn, cv, y["dist"][i], TOL_DIST))

        cam_ok = True
        print(f"\n[{name}]")
        print(f"  {'param':<6}{'common':>18}{'yaml':>18}{'diff':>14}   判定")
        for pn, cv, yv, tol in rows:
            diff = abs(float(cv) - float(yv))
            ok = diff <= tol
            cam_ok = cam_ok and ok
            print(f"  {pn:<6}{float(cv):>18.6f}{float(yv):>18.6f}{diff:>14.6e}   {fmt_ok(ok)}")
        intr_all_ok = intr_all_ok and cam_ok
        print(f"  => {name} 内参 {fmt_ok(cam_ok)}")

    # ---------------- 外参对比 (相机间相对位姿) ----------------
    print("\n" + "=" * 78)
    print("外参对比 (相机间相对位姿, OpenCV 约定)")
    print("  common: extrinsics_camera[A][B] (camA->camB)")
    print("  yaml  : T_cam_imu[B] @ inv(T_cam_imu[A])")
    print("=" * 78)
    extr_all_ok = True
    print(f"\n  {'pair':<14}{'旋转差(deg)':>14}{'平移差(m)':>14}   判定")
    for i in range(len(fisheye)):
        for j in range(len(fisheye)):
            if i == j:
                continue
            a, b = fisheye[i], fisheye[j]
            common_a = common.get(a, {})
            T_common = common_a.get("extrinsics_camera", {}).get(b)
            if T_common is None:
                print(f"  {a}->{b:<7} common 缺失该相对位姿")
                extr_all_ok = False
                continue
            T_common = np.array(T_common, dtype=np.float64)
            T_yaml = yaml_cams[b]["T_cam_imu"] @ np.linalg.inv(yaml_cams[a]["T_cam_imu"])
            drot = rot_angle_deg(T_common[:3, :3], T_yaml[:3, :3])
            dtrans = float(np.linalg.norm(T_common[:3, 3] - T_yaml[:3, 3]))
            ok = (drot <= TOL_ROT_DEG) and (dtrans <= TOL_TRANS_M)
            extr_all_ok = extr_all_ok and ok
            print(f"  {a}->{b:<7}{drot:>14.4f}{dtrans:>14.4f}   {fmt_ok(ok)}")

    # ---------------- 汇总 ----------------
    print("\n" + "=" * 78)
    print("汇总")
    print("=" * 78)
    print(f"  内参一致性: {fmt_ok(intr_all_ok)}")
    print(f"  外参一致性: {fmt_ok(extr_all_ok)}")
    print(
        "\n  说明: 内参由 bake 写入 USD,应与标定 yaml 完全一致;\n"
        "        外参由 USD 相机组几何决定,只有当相机组也按新标定重新摆位后才会一致。"
    )
    return 0 if (intr_all_ok and extr_all_ok) else 2


if __name__ == "__main__":
    sys.exit(main())
