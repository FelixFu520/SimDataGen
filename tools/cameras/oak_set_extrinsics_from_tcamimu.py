"""根据 Kalibr 标定 yaml 的 T_cam_imu 重新摆放鱼眼相机在 rig 中的位姿(外参)。

背景:oak_bake_camera_intrinsics.py 只 bake 内参,不改相机在 USD rig 里的摆位。
若重新标定后想让 USD 外参也反映新标定,用本脚本按 yaml 的 T_cam_imu 重新摆位。

坐标系(关键):
  - Kalibr `T_cam_imu` 为 imu->cam,相机系是 OpenCV 约定 (X右 Y下 Z前)。
  - Isaac/USD 相机 prim 本地系是 RTX 约定 (X右 Y上 Z后)。
  - 二者转换: P_isaac = R_cv_to_isaac @ P_cv, R_cv_to_isaac = diag(1,-1,-1,1)。
  - 需要写到相机 prim 上、相对 base_link(=IMU) 的 RTX 变换:
        T_baselink_cam_rtx = inv(T_cam_imu) @ R_cv_to_isaac
    (右乘 R_cv_to_isaac 不影响平移,只把旋转从 cv 相机系换到 RTX 相机系)

只动 --cameras 指定的鱼眼相机(默认 CAM_A..D);针孔相机(如 CAM_Front/Back)不动。

Usage:
    ./app/python.sh tools/cameras/oak_set_extrinsics_from_tcamimu.py \\
        --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \\
        --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

import numpy as np
import yaml
from pxr import Gf, Usd, UsdGeom

CAM_KEY_TO_NAME = {"cam0": "CAM_A", "cam1": "CAM_B", "cam2": "CAM_C", "cam3": "CAM_D"}

R_CV_TO_ISAAC = np.diag([1.0, -1.0, -1.0, 1.0])


def gf_to_np(mat: Gf.Matrix4d) -> np.ndarray:
    """USD 行优先 Matrix4d -> numpy col-major (M_math = M_usd.T)。"""
    arr = np.array([[mat[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
    return arr.T


def np_to_gf(m_np: np.ndarray) -> Gf.Matrix4d:
    """numpy col-major (v'=M@v) -> USD 行优先 Matrix4d。M_usd = m_np.T。"""
    mt = m_np.T
    return Gf.Matrix4d(*[float(mt[i][j]) for i in range(4) for j in range(4)])


def load_tcamimu(path):
    data = yaml.safe_load(open(path))
    out = {}
    for key, cam in data.items():
        if cam.get("camera_model") != "omni":
            continue
        name = CAM_KEY_TO_NAME.get(key, cam.get("rostopic", key))
        out[name] = np.array(cam["T_cam_imu"], dtype=np.float64)
    return out


def rot_angle_deg(Ra, Rb):
    R = Ra @ Rb.T
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    return float(np.degrees(np.arccos(c)))


def find_base_link(stage):
    for prim in stage.Traverse():
        if prim.GetName() == "base_link":
            return prim
    return None


def parse_args():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--usd", required=True)
    ap.add_argument("--yaml", required=True)
    ap.add_argument("--cameras", nargs="*", default=["CAM_A", "CAM_B", "CAM_C", "CAM_D"])
    ap.add_argument("--dry_run", action="store_true", help="只打印,不写回 USD")
    ap.add_argument(
        "--no_cv_convert",
        action="store_true",
        help="复刻 oak_camera_extrinsics.py 的旧约定: 直接用 inv(T_cam_imu) 作为 RTX 位姿, "
             "不做 OpenCV->RTX 转换(用于对比测试)",
    )
    return ap.parse_args()


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    tcamimu = load_tcamimu(os.path.abspath(args.yaml))

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"open failed: {usd_path}")

    base_link = find_base_link(stage)
    if base_link is None:
        raise RuntimeError("USD 中未找到 base_link prim(IMU 参考系)")
    print(f"base_link: {base_link.GetPath()}", flush=True)

    cam_prims = {}
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera) and prim.GetName() in args.cameras:
            cam_prims[prim.GetName()] = prim

    missing = set(args.cameras) - set(cam_prims)
    if missing:
        raise RuntimeError(f"USD 中未找到相机 prim: {sorted(missing)}")

    # 摆位前的世界变换(base_link 与各相机父级),用于把"相对 base_link"换成"相对父级"
    xc = UsdGeom.XformCache(Usd.TimeCode.Default())
    W_base = gf_to_np(xc.GetLocalToWorldTransform(base_link))

    print("\n重新摆位(按 T_baselink_cam = inv(T_cam_imu) @ R_cv_to_isaac):", flush=True)
    for name in args.cameras:
        prim = cam_prims[name]
        if name not in tcamimu:
            print(f"  [跳过] {name}: yaml 无 T_cam_imu")
            continue
        if args.no_cv_convert:
            T_bl_cam = np.linalg.inv(tcamimu[name])  # 旧约定: 不转换坐标系
        else:
            T_bl_cam = np.linalg.inv(tcamimu[name]) @ R_CV_TO_ISAAC  # RTX, 相对 base_link
        W_parent = gf_to_np(xc.GetLocalToWorldTransform(prim.GetParent()))
        W_cam_target = W_base @ T_bl_cam
        local = np.linalg.inv(W_parent) @ W_cam_target  # 相对父级的本地变换

        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        op = xf.AddTransformOp()
        op.Set(np_to_gf(local))

        pos = T_bl_cam[:3, 3]
        print(f"  {name}: baselink 位置(m) = ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f})", flush=True)

    # ---------- 自检:重算 base_link->cam 与相机间相对位姿,和 yaml 对比 ----------
    xc2 = UsdGeom.XformCache(Usd.TimeCode.Default())
    W_base2 = gf_to_np(xc2.GetLocalToWorldTransform(base_link))
    T_bl_cam_cv = {}
    print("\n自检: USD(重摆后) vs yaml T_cam_imu", flush=True)
    print(f"  {'cam':<7}{'旋转差(deg)':>12}{'平移差(m)':>12}", flush=True)
    for name in args.cameras:
        if name not in tcamimu:
            continue
        W_cam = gf_to_np(xc2.GetLocalToWorldTransform(cam_prims[name]))
        T_bl_cam_rtx = np.linalg.inv(W_base2) @ W_cam
        T_cv = T_bl_cam_rtx @ R_CV_TO_ISAAC  # baselink->cam, cv 相机系
        T_bl_cam_cv[name] = T_cv
        T_yaml = np.linalg.inv(tcamimu[name])  # baselink->cam (cv) = inv(T_cam_imu)
        drot = rot_angle_deg(T_cv[:3, :3], T_yaml[:3, :3])
        dtrans = float(np.linalg.norm(T_cv[:3, 3] - T_yaml[:3, 3]))
        print(f"  {name:<7}{drot:>12.4f}{dtrans:>12.4f}", flush=True)

    if args.dry_run:
        print("\n[dry_run] 未写回 USD", flush=True)
        return 0

    stage.GetRootLayer().Save()
    print(f"\nSaved USD: {usd_path}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        ret = main()
    finally:
        simulation_app.close()
    sys.exit(ret or 0)
