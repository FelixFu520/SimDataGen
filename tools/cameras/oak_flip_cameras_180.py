"""将指定相机绕 base_link(工装中心)整体翻转 180 度。

用于修正鱼眼相机"图像上下颠倒 + 环绕方向反了(CCW 应为 CW)"的问题:
绕 base_link 的一条水平轴翻转 180°,可同时把 up 翻正、并把环绕方向反向,
且刚体旋转保持各相机之间的相对几何不变(common 的相机间外参与 yaml 仍一致)。

坐标系: 变换在 base_link(=IMU, 工装中心)坐标系下进行:
    T_world_cam_new = W_baselink @ FLIP @ inv(W_baselink) @ T_world_cam_old
FLIP 为绕 base_link 某轴的 180° 旋转(默认 x 轴)。

Usage:
    ./app/python.sh tools/cameras/oak_flip_cameras_180.py \\
        --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \\
        --cameras CAM_A CAM_B CAM_C CAM_D --axis x
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

import numpy as np
from pxr import Gf, Usd, UsdGeom


def gf_to_np(mat: Gf.Matrix4d) -> np.ndarray:
    arr = np.array([[mat[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
    return arr.T


def np_to_gf(m_np: np.ndarray) -> Gf.Matrix4d:
    mt = m_np.T
    return Gf.Matrix4d(*[float(mt[i][j]) for i in range(4) for j in range(4)])


def flip_matrix(axis: str) -> np.ndarray:
    """base_link 系下绕 axis 的 180° 旋转 (4x4)。"""
    m = np.eye(4)
    if axis == "x":
        m[1, 1] = -1; m[2, 2] = -1
    elif axis == "y":
        m[0, 0] = -1; m[2, 2] = -1
    elif axis == "z":
        m[0, 0] = -1; m[1, 1] = -1
    else:
        raise ValueError(f"axis must be x/y/z, got {axis}")
    return m


def find_base_link(stage):
    for prim in stage.Traverse():
        if prim.GetName() == "base_link":
            return prim
    return None


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--usd", required=True)
    ap.add_argument("--cameras", nargs="*", default=["CAM_A", "CAM_B", "CAM_C", "CAM_D"])
    ap.add_argument("--axis", choices=("x", "y", "z"), default="x",
                    help="绕 base_link 哪条轴翻转 180°(默认 x)")
    ap.add_argument("--dry_run", action="store_true")
    return ap.parse_args()


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"open failed: {usd_path}")

    base_link = find_base_link(stage)
    if base_link is None:
        raise RuntimeError("USD 中未找到 base_link prim")

    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    W_base = gf_to_np(xc.GetLocalToWorldTransform(base_link))
    FLIP = flip_matrix(args.axis)
    # 在 base_link 坐标系下做 FLIP: T_world_new = Wb @ FLIP @ inv(Wb) @ T_world_old
    G = W_base @ FLIP @ np.linalg.inv(W_base)

    targets = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera) and prim.GetName() in args.cameras:
            targets[prim.GetName()] = prim
    missing = set(args.cameras) - set(targets)
    if missing:
        raise RuntimeError(f"未找到相机 prim: {sorted(missing)}")

    print(f"base_link: {base_link.GetPath()}  翻转轴: base_link/{args.axis}  180°", flush=True)
    for name, prim in targets.items():
        W_cam = gf_to_np(xc.GetLocalToWorldTransform(prim))
        W_cam_new = G @ W_cam
        W_parent = gf_to_np(xc.GetLocalToWorldTransform(prim.GetParent()))
        local_new = np.linalg.inv(W_parent) @ W_cam_new
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
        op.Set(np_to_gf(local_new))
        print(f"  翻转 {name}", flush=True)

    if args.dry_run:
        print("[dry_run] 未写回", flush=True)
        return 0
    stage.GetRootLayer().Save()
    print(f"Saved USD: {usd_path}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        ret = main()
    finally:
        simulation_app.close()
    sys.exit(ret or 0)
