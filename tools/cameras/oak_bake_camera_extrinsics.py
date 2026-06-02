"""把外参平移 xyz 扰动 bake 进相机 USD（只改 translate op，不碰旋转）。

原版 USD 旋转来自 Isaac Sim 手工配置（常为 orient 四元数）。任何矩阵→欧拉→写回
旋转 op 的操作都可能改变有效朝向（图像倾斜/倒立）。因此 bake 时仅在现有
translate 上叠加 xyz 偏移。

Usage:
    ./app/python.sh tools/cameras/oak_bake_camera_extrinsics.py \\
        --usd assets/cameras/perturbed_camera/oak_camera_4lut_2H30YA_perturbed.usd \\
        --yaml assets/cameras/perturbed_camera/fisheye_cams.yaml \\
        --perturb-pinholes \\
        --seed 0
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

import numpy as np
import yaml
from pxr import Gf, Usd, UsdGeom

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from tools.cameras.oak_extrinsics_perturb import (  # noqa: E402
    DEFAULT_TRANS_RANGE_M,
    perturb_translate,
    rng_for_camera,
)

PINHOLE_CAMERAS = ("CAM_Front", "CAM_Back")


def load_fisheye_names_from_yaml(yaml_path: str) -> set[str]:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    cam_key_to_name = {
        "cam0": "CAM_A",
        "cam1": "CAM_B",
        "cam2": "CAM_C",
        "cam3": "CAM_D",
    }
    names: set[str] = set()
    for cam_key, cam in data.items():
        if not isinstance(cam, dict):
            continue
        if cam.get("T_cam_imu") is None:
            continue
        names.add(cam_key_to_name.get(cam_key, cam.get("rostopic", cam_key)))
    return names


def apply_translate_perturb_to_prim(
    prim: Usd.Prim,
    rng: np.random.Generator,
    *,
    trans_range_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    """只更新 translate xformOp，旋转 op 保持原样。"""
    xform = UsdGeom.Xformable(prim)
    delta = perturb_translate(rng, trans_range_m)

    for op in xform.GetOrderedXformOps():
        if op.GetOpType() != UsdGeom.XformOp.TypeTranslate:
            continue
        v = op.Get()
        translate = np.array([float(v[0]), float(v[1]), float(v[2])], dtype=np.float64)
        translate += delta
        op.Set(
            Gf.Vec3d(
                float(translate[0]),
                float(translate[1]),
                float(translate[2]),
            )
        )
        return translate, delta

    raise RuntimeError(
        f"no translate xformOp on {prim.GetPath()}; cannot apply extrinsics perturbation"
    )


def bake_extrinsics(
    usd_path: str,
    yaml_path: str,
    *,
    perturb_pinholes: bool = False,
    seed: int = 0,
    trans_range_m: float = DEFAULT_TRANS_RANGE_M,
) -> None:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"failed to open USD: {usd_path}")

    fisheye_in_yaml = load_fisheye_names_from_yaml(yaml_path)
    if not fisheye_in_yaml:
        raise RuntimeError(f"no T_cam_imu entries in {yaml_path}")

    updated = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Camera):
            continue
        name = prim.GetName()

        should_perturb = name in fisheye_in_yaml or (
            perturb_pinholes and name in PINHOLE_CAMERAS
        )
        if not should_perturb:
            continue

        rng = rng_for_camera(seed, name)
        translate, delta = apply_translate_perturb_to_prim(
            prim, rng, trans_range_m=trans_range_m
        )
        updated.append(name)
        print(
            f"  baked {name} xyz delta (seed={seed}): "
            f"T=({translate[0]:.6f}, {translate[1]:.6f}, {translate[2]:.6f}) "
            f"d=({delta[0]*1e3:.3f}, {delta[1]*1e3:.3f}, {delta[2]*1e3:.3f}) mm",
            flush=True,
        )

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
        help="扰动 yaml（确认鱼眼条目；位姿 bake 以 USD 原 translate + seed 为准）",
    )
    parser.add_argument(
        "--perturb-pinholes",
        action="store_true",
        help="对 CAM_Front/CAM_Back 用 --seed 施加小幅 xyz 平移扰动",
    )
    parser.add_argument("--seed", type=int, default=0, help="与 yaml 生成时相同的 seed")
    parser.add_argument(
        "--trans-range-mm",
        type=float,
        default=None,
        help="平移 xyz 扰动半幅 (mm)，默认 1",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    yaml_path = os.path.abspath(args.yaml)
    trans_m = (
        args.trans_range_mm / 1000.0
        if args.trans_range_mm is not None
        else DEFAULT_TRANS_RANGE_M
    )

    bake_extrinsics(
        usd_path,
        yaml_path,
        perturb_pinholes=args.perturb_pinholes,
        seed=args.seed,
        trans_range_m=trans_m,
    )


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
