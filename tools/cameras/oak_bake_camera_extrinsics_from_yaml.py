"""把 Kalibr yaml 中的 T_cam_imu 绝对外参 bake 进相机 USD。

本仓库 fisheye_cams.yaml 的 T_cam_imu 与 Kalibr results 中的 T_ci 一致
（imu → cam，OpenCV 相机系：X 右、Y 下、Z 前）。

Isaac Sim / USD 相机系：X 右、Y 上、Z 后（沿 -Z 观察）。因此：
  - translate: T_ic = inv(T_ci) 的平移（相机原点在 IMU/base_link 下）
  - orient:    R_usd = R_ic @ diag(-1, 1, -1)（相对 OpenCV 绕相机 Y 转 180°：
               光轴由 +Z 对齐到 USD -Z，且保持 up 朝上；若用 diag(1,-1,-1)
               则光轴对了但画面上下颠倒）

Usage:
    ./app/python.sh tools/cameras/oak_bake_camera_extrinsics_from_yaml.py \\
        --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \\
        --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml
"""

from __future__ import annotations

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

import numpy as np
import yaml
from pxr import Gf, Usd, UsdGeom
from scipy.spatial.transform import Rotation

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

CAM_KEY_TO_NAME = {
    "cam0": "CAM_A",
    "cam1": "CAM_B",
    "cam2": "CAM_C",
    "cam3": "CAM_D",
}

# OpenCV → USD：绕相机 Y 转 180°，使 look(+Z→-Z) 朝外且 up 不颠倒
R_OPENCV_TO_USD = np.diag([-1.0, 1.0, -1.0])


def load_T_ci_from_yaml(yaml_path: str) -> dict[str, np.ndarray]:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    out: dict[str, np.ndarray] = {}
    for cam_key, cam in data.items():
        if not isinstance(cam, dict) or cam.get("T_cam_imu") is None:
            continue
        name = CAM_KEY_TO_NAME.get(cam_key, cam.get("rostopic", cam_key))
        # 本仓库约定：yaml T_cam_imu == Kalibr T_ci (imu→cam)
        out[name] = np.asarray(cam["T_cam_imu"], dtype=np.float64).reshape(4, 4)
    return out


def t_ci_to_usd_pose(T_ci: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """T_ci (imu→cam, OpenCV) → Isaac translate + R_usd + XYZ 欧拉角（度）。"""
    T_ic = np.linalg.inv(np.asarray(T_ci, dtype=np.float64).reshape(4, 4))
    translate = T_ic[:3, 3].copy()
    R_usd = T_ic[:3, :3] @ R_OPENCV_TO_USD
    euler_xyz_deg = Rotation.from_matrix(R_usd).as_euler("XYZ", degrees=True)
    return translate, R_usd, euler_xyz_deg


def quat_wxyz_from_R(R: np.ndarray) -> Gf.Quatd:
    """旋转矩阵 → USD Gf.Quatd(real, i, j, k) = (w, x, y, z)。"""
    q = Rotation.from_matrix(R).as_quat()  # scipy: x, y, z, w
    return Gf.Quatd(float(q[3]), Gf.Vec3d(float(q[0]), float(q[1]), float(q[2])))


def apply_pose_to_prim(prim: Usd.Prim, translate: np.ndarray, R: np.ndarray) -> None:
    """更新 translate + orient；若无 orient 则写 rotateXYZ（针孔相机）。"""
    xform = UsdGeom.Xformable(prim)
    ops = list(xform.GetOrderedXformOps())
    has_translate = False
    has_orient = False
    has_rotate_xyz = False

    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            op.Set(
                Gf.Vec3d(
                    float(translate[0]),
                    float(translate[1]),
                    float(translate[2]),
                )
            )
            has_translate = True
        elif op.GetOpType() == UsdGeom.XformOp.TypeOrient:
            op.Set(quat_wxyz_from_R(R))
            has_orient = True
        elif op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            euler = Rotation.from_matrix(R).as_euler("XYZ", degrees=True)
            op.Set(Gf.Vec3f(float(euler[0]), float(euler[1]), float(euler[2])))
            has_rotate_xyz = True

    if not has_translate:
        raise RuntimeError(
            f"no translate xformOp on {prim.GetPath()}; cannot bake extrinsics"
        )
    if not has_orient and not has_rotate_xyz:
        raise RuntimeError(
            f"no orient/rotateXYZ xformOp on {prim.GetPath()}; cannot bake rotation"
        )


def bake_extrinsics_from_yaml(usd_path: str, yaml_path: str) -> None:
    cameras = load_T_ci_from_yaml(yaml_path)
    if not cameras:
        raise RuntimeError(f"no T_cam_imu entries in {yaml_path}")

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"failed to open USD: {usd_path}")

    updated = []
    missing = set(cameras.keys())

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Camera):
            continue
        name = prim.GetName()
        if name not in cameras:
            continue
        missing.discard(name)

        translate, R_usd, euler = t_ci_to_usd_pose(cameras[name])
        apply_pose_to_prim(prim, translate, R_usd)
        updated.append(name)
        print(
            f"  baked {name}: "
            f"T=({translate[0]:.6f}, {translate[1]:.6f}, {translate[2]:.6f}) "
            f"Rxyz=({euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f})",
            flush=True,
        )

    if missing:
        raise RuntimeError(f"USD 中未找到相机 prim: {sorted(missing)}")
    if not updated:
        raise RuntimeError("no camera extrinsics updated")

    stage.GetRootLayer().Save()
    print(f"Done. Updated {len(updated)} cameras in {usd_path}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, help="camera rig USD path")
    parser.add_argument(
        "--yaml",
        required=True,
        help="含 T_cam_imu 的 Kalibr fisheye_cams.yaml",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    bake_extrinsics_from_yaml(os.path.abspath(args.usd), os.path.abspath(args.yaml))


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
