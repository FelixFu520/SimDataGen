#!/usr/bin/env python3
"""从原始 Kalibr omni 标定生成扰动版 fisheye_cams.yaml。

内参 profile（在 docs/oak_camera/calibration/fisheye_cams.yaml 上扰动）：
  - small_change   : ±2% 量级泛化漂移
  - pinhole_like   : 降低 xi、减弱畸变，更接近针孔
  - fisheye_like   : 提高 xi、增强畸变，更接近 omni/鱼眼
  - extrinsics_change : 仅外参小幅随机扰动（内参不变）

外参：--perturb-extrinsics 仅扰动 T_cam_imu 平移 xyz，默认 ±1 mm，不修改旋转。

Usage:
    # 仅内参
    ./app/python.sh tools/cameras/oak_generate_perturbed_yaml.py \\
        --profile small_change \\
        --output docs/oak_camera_perturbed/fisheye_cams_small_change.yaml

    # 仅外参
    ./app/python.sh tools/cameras/oak_generate_perturbed_yaml.py \\
        --profile extrinsics_change \\
        --seed 0 \\
        --output docs/oak_camera_perturbed/fisheye_cams_extrinsics_change.yaml

    # 内外参联合
    ./app/python.sh tools/cameras/oak_generate_perturbed_yaml.py \\
        --profile small_change --perturb-extrinsics --seed 0 \\
        --output docs/oak_camera_perturbed/fisheye_cams_small_change_extrinsics.yaml
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.cameras.oak_extrinsics_perturb import (  # noqa: E402
    CAM_NAMES,
    DEFAULT_TRANS_RANGE_M,
    matrix_to_yaml_nested,
    perturb_T_cam_imu_translate,
    rng_for_camera,
    t_cam_imu_to_isaac_sim_pose,
)

DEFAULT_BASE = ROOT / "docs/oak_camera/calibration/fisheye_cams.yaml"

CAM_KEYS = ("cam0", "cam1", "cam2", "cam3")

INTRINSICS_PROFILES = ("small_change", "pinhole_like", "fisheye_like")
ALL_PROFILES = INTRINSICS_PROFILES + ("extrinsics_change",)

PER_CAM = {
    "small_change": {
        "CAM_A": dict(xi=1.02, fx=1.02, fy=1.015, cx=5, cy=-5, k1=1.05, k2=0.95, p1=1e-4, p2=-1e-4),
        "CAM_B": dict(xi=0.98, fx=0.985, fy=0.98, cx=-5, cy=5, k1=0.95, k2=1.05, p1=-1e-4, p2=1e-4),
        "CAM_C": dict(xi=1.015, fx=0.98, fy=1.02, cx=-5, cy=-5, k1=1.05, k2=0.95, p1=1e-4, p2=1e-4),
        "CAM_D": dict(xi=0.985, fx=1.02, fy=0.985, cx=5, cy=5, k1=0.95, k2=1.05, p1=-1e-4, p2=-1e-4),
    },
    "pinhole_like": {
        "CAM_A": dict(xi=0.89, fx=1.04, fy=1.03, cx=4, cy=-4, k1=0.72, k2=0.72, p1=0.0, p2=0.0),
        "CAM_B": dict(xi=0.87, fx=1.035, fy=1.04, cx=-4, cy=4, k1=0.75, k2=0.68, p1=0.0, p2=0.0),
        "CAM_C": dict(xi=0.88, fx=1.03, fy=1.035, cx=-4, cy=-4, k1=0.70, k2=0.74, p1=0.0, p2=0.0),
        "CAM_D": dict(xi=0.90, fx=1.045, fy=1.03, cx=4, cy=4, k1=0.78, k2=0.70, p1=0.0, p2=0.0),
    },
    "fisheye_like": {
        "CAM_A": dict(xi=1.13, fx=0.96, fy=0.975, cx=5, cy=-5, k1=1.12, k2=1.22, p1=2e-4, p2=-2e-4),
        "CAM_B": dict(xi=1.11, fx=0.965, fy=0.97, cx=-5, cy=5, k1=1.08, k2=1.25, p1=-2e-4, p2=2e-4),
        "CAM_C": dict(xi=1.12, fx=0.97, fy=0.98, cx=-5, cy=-5, k1=1.15, k2=1.20, p1=2e-4, p2=2e-4),
        "CAM_D": dict(xi=1.14, fx=0.955, fy=0.968, cx=5, cy=5, k1=1.10, k2=1.28, p1=-2e-4, p2=-2e-4),
    },
}

PROFILE_COMMENTS = {
    "small_change": "Perturbed omni intrinsics (±2% generalization drift).",
    "pinhole_like": "Perturbed omni intrinsics biased toward pinhole.",
    "fisheye_like": "Perturbed omni intrinsics biased toward fisheye/omni.",
    "extrinsics_change": "Omni intrinsics unchanged; T_cam_imu translation xyz perturbed only.",
}


def apply_intrinsics_perturbation(cam: dict, spec: dict) -> dict:
    out = copy.deepcopy(cam)
    intr = list(out["intrinsics"])
    dist = list(out["distortion_coeffs"])

    intr[0] = float(intr[0]) * spec["xi"]
    intr[1] = float(intr[1]) * spec["fx"]
    intr[2] = float(intr[2]) * spec["fy"]
    intr[3] = float(intr[3]) + spec["cx"]
    intr[4] = float(intr[4]) + spec["cy"]

    dist[0] = float(dist[0]) * spec["k1"]
    dist[1] = float(dist[1]) * spec["k2"]
    dist[2] = float(dist[2]) + spec["p1"]
    dist[3] = float(dist[3]) + spec["p2"]

    out["intrinsics"] = intr
    out["distortion_coeffs"] = dist
    return out


def apply_extrinsics_perturbation(
    cam: dict,
    cam_name: str,
    seed: int,
    *,
    trans_range_m: float = DEFAULT_TRANS_RANGE_M,
) -> dict:
    out = copy.deepcopy(cam)
    T = np.asarray(out["T_cam_imu"], dtype=np.float64)
    rng = rng_for_camera(seed, cam_name)
    T_new = perturb_T_cam_imu_translate(T, rng, trans_range_m=trans_range_m)
    out["T_cam_imu"] = matrix_to_yaml_nested(T_new)
    return out


def generate(
    profile: str,
    base_path: Path,
    *,
    perturb_extrinsics: bool = False,
    seed: int = 0,
    trans_range_m: float = DEFAULT_TRANS_RANGE_M,
) -> dict:
    with open(base_path, "r") as f:
        data = yaml.safe_load(f)

    do_intrinsics = profile != "extrinsics_change"
    do_extrinsics = perturb_extrinsics or profile == "extrinsics_change"

    for cam_key, cam_name in zip(CAM_KEYS, CAM_NAMES):
        if cam_key not in data:
            raise KeyError(f"missing {cam_key} in {base_path}")
        cam = data[cam_key]
        if do_intrinsics:
            cam = apply_intrinsics_perturbation(cam, PER_CAM[profile][cam_name])
        if do_extrinsics:
            cam = apply_extrinsics_perturbation(
                cam,
                cam_name,
                seed,
                trans_range_m=trans_range_m,
            )
        data[cam_key] = cam
    return data


def _rel_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        required=True,
        choices=sorted(ALL_PROFILES),
        help="内参扰动配方；extrinsics_change 表示仅改外参",
    )
    parser.add_argument(
        "--perturb-extrinsics",
        action="store_true",
        help="在内参 profile 基础上额外扰动 T_cam_imu",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=0,
        help="外参随机扰动种子（可复现）",
    )
    parser.add_argument(
        "--trans-range-mm",
        type=float,
        default=DEFAULT_TRANS_RANGE_M * 1000.0,
        help="平移 xyz 扰动半幅 (mm)，默认 1",
    )
    parser.add_argument(
        "--base",
        default=str(DEFAULT_BASE),
        help="原始 fisheye_cams.yaml",
    )
    parser.add_argument("--output", required=True, help="输出 yaml 路径")
    args = parser.parse_args()

    if args.profile == "extrinsics_change" and args.perturb_extrinsics:
        parser.error("--profile extrinsics_change 已含外参扰动，无需再加 --perturb-extrinsics")

    base_path = Path(args.base).resolve()
    out_path = Path(args.output).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    trans_m = args.trans_range_mm / 1000.0
    data = generate(
        args.profile,
        base_path,
        perturb_extrinsics=args.perturb_extrinsics,
        seed=args.seed,
        trans_range_m=trans_m,
    )

    parts = [PROFILE_COMMENTS[args.profile]]
    if args.perturb_extrinsics and args.profile != "extrinsics_change":
        parts.append("Extrinsics perturbed with same seed.")
    if args.profile == "extrinsics_change" or args.perturb_extrinsics:
        parts.append(
            f"Extrinsics seed={args.seed}, xyz±{args.trans_range_mm:g}mm (rotation unchanged)."
        )

    header = (
        f"# {' '.join(parts)}\n"
        f"# Generated by tools/cameras/oak_generate_perturbed_yaml.py "
        f"--profile {args.profile}"
    )
    if args.perturb_extrinsics:
        header += " --perturb-extrinsics"
    if args.profile == "extrinsics_change" or args.perturb_extrinsics:
        header += f" --seed {args.seed}"
    header += f"\n# Base: {_rel_path(base_path)}\n"

    with open(out_path, "w") as f:
        f.write(header)
        yaml.dump(data, f, default_flow_style=None, sort_keys=False, allow_unicode=True)

    print(f"Wrote {out_path} (profile={args.profile})", flush=True)
    for cam_key, cam_name in zip(CAM_KEYS, CAM_NAMES):
        intr = data[cam_key]["intrinsics"]
        line = (
            f"  {cam_name}: xi={intr[0]:.6f} fx={intr[1]:.2f} fy={intr[2]:.2f} "
            f"cx={intr[3]:.2f} cy={intr[4]:.2f}"
        )
        if args.profile == "extrinsics_change" or args.perturb_extrinsics:
            t, r = t_cam_imu_to_isaac_sim_pose(data[cam_key]["T_cam_imu"])
            line += (
                f" | T=({t[0]:.6f},{t[1]:.6f},{t[2]:.6f}) "
                f"R=({r[0]:.1f},{r[1]:.1f},{r[2]:.1f})"
            )
        print(line, flush=True)


if __name__ == "__main__":
    main()
