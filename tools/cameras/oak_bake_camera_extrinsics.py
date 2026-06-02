"""把 yaml 中 T_cam_imu 写入相机 USD 的局部位姿（相对 base_link）。

Usage:
    ./app/python.sh tools/cameras/oak_bake_camera_extrinsics.py \\
        --usd assets/cameras/oak_camera_4lut_2H30YA_perturbed_extrinsics_change.usd \\
        --yaml docs/oak_camera_perturbed/fisheye_cams_extrinsics_change.yaml

    # yaml 仅含 4 路鱼眼外参时，对 CAM_Front/CAM_Back 用同一 seed 施加小幅扰动
    ./app/python.sh tools/cameras/oak_bake_camera_extrinsics.py \\
        --usd ... --yaml ... --perturb-pinholes --seed 0
"""

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

from tools.cameras.oak_extrinsics_perturb import (  # noqa: E402
    perturb_local_transform,
    rng_for_camera,
    t_cam_imu_to_isaac_sim_pose,
)

CAM_KEY_TO_NAME = {
    "cam0": "CAM_A",
    "cam1": "CAM_B",
    "cam2": "CAM_C",
    "cam3": "CAM_D",
}
PINHOLE_CAMERAS = ("CAM_Front", "CAM_Back")


def _gf_matrix_to_np(m) -> np.ndarray:
    return np.array(m, dtype=np.float64).reshape(4, 4)


def load_T_cam_imu_from_yaml(yaml_path: str) -> dict[str, np.ndarray]:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    out: dict[str, np.ndarray] = {}
    for cam_key, cam in data.items():
        if not isinstance(cam, dict):
            continue
        T = cam.get("T_cam_imu")
        if T is None:
            continue
        name = CAM_KEY_TO_NAME.get(cam_key, cam.get("rostopic", cam_key))
        out[name] = np.asarray(T, dtype=np.float64)
    return out


def find_base_link(stage: Usd.Stage) -> Usd.Prim | None:
    for prim in stage.Traverse():
        if prim.GetName() == "base_link":
            return prim
    return None


def get_local_transform(prim: Usd.Prim) -> np.ndarray:
    xform = UsdGeom.Xformable(prim)
    return _gf_matrix_to_np(xform.GetLocalTransformation())


def set_prim_pose(prim: Usd.Prim, translate: np.ndarray, euler_xyz_deg: np.ndarray) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    t_op.Set(Gf.Vec3d(float(translate[0]), float(translate[1]), float(translate[2])))
    r_op = xform.AddRotateXYZOp()
    r_op.Set(
        Gf.Vec3f(
            float(euler_xyz_deg[0]),
            float(euler_xyz_deg[1]),
            float(euler_xyz_deg[2]),
        )
    )


def matrix_to_isaac_pose(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    translate = T[:3, 3].copy()
    euler = Rotation.from_matrix(T[:3, :3]).as_euler("XYZ", degrees=True)
    return translate, euler


def bake_extrinsics(
    usd_path: str,
    yaml_path: str,
    *,
    perturb_pinholes: bool = False,
    seed: int = 0,
    trans_range_m: float | None = None,
    rot_range_deg: float | None = None,
) -> None:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"failed to open USD: {usd_path}")

    base_link = find_base_link(stage)
    if base_link is None:
        raise RuntimeError(f"base_link not found in {usd_path}")

    T_from_yaml = load_T_cam_imu_from_yaml(yaml_path)
    kwargs = {}
    if trans_range_m is not None:
        kwargs["trans_range_m"] = trans_range_m
    if rot_range_deg is not None:
        kwargs["rot_range_deg"] = rot_range_deg

    updated = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Camera):
            continue
        name = prim.GetName()

        if name in T_from_yaml:
            translate, euler = t_cam_imu_to_isaac_sim_pose(T_from_yaml[name])
            set_prim_pose(prim, translate, euler)
            updated.append(name)
            print(
                f"  baked {name} from yaml: "
                f"T=({translate[0]:.6f}, {translate[1]:.6f}, {translate[2]:.6f}) "
                f"R=({euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f})",
                flush=True,
            )
        elif perturb_pinholes and name in PINHOLE_CAMERAS:
            T_local = get_local_transform(prim)
            rng = rng_for_camera(seed, name)
            T_new = perturb_local_transform(T_local, rng, **kwargs)
            translate, euler = matrix_to_isaac_pose(T_new)
            set_prim_pose(prim, translate, euler)
            updated.append(name)
            print(
                f"  baked {name} perturbed (seed={seed}): "
                f"T=({translate[0]:.6f}, {translate[1]:.6f}, {translate[2]:.6f})",
                flush=True,
            )

    if not updated:
        raise RuntimeError("no camera extrinsics updated")

    stage.GetRootLayer().Save()
    print(f"Done. Updated {len(updated)} cameras in {usd_path}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, help="camera rig USD path")
    parser.add_argument("--yaml", required=True, help="Kalibr yaml with T_cam_imu")
    parser.add_argument(
        "--perturb-pinholes",
        action="store_true",
        help="对 yaml 中无条目的 CAM_Front/CAM_Back 用 --seed 施加小幅随机外参扰动",
    )
    parser.add_argument("--seed", type=int, default=0, help="与 yaml 生成时相同的 seed")
    parser.add_argument(
        "--trans-range-mm",
        type=float,
        default=None,
        help="平移扰动半幅 (mm)，默认 3",
    )
    parser.add_argument(
        "--rot-range-deg",
        type=float,
        default=None,
        help="旋转扰动半幅 (度)，默认 1.5",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    yaml_path = os.path.abspath(args.yaml)
    trans_m = args.trans_range_mm / 1000.0 if args.trans_range_mm is not None else None

    bake_extrinsics(
        usd_path,
        yaml_path,
        perturb_pinholes=args.perturb_pinholes,
        seed=args.seed,
        trans_range_m=trans_m,
        rot_range_deg=args.rot_range_deg,
    )


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
