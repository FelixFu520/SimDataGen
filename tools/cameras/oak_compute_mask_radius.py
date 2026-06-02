#!/usr/bin/env python3
"""估计鱼眼 LUT 相机的 maskRadius(像素), 用于预览或单独调试。

bake 时已内置自动估计，一般只需:
    ./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \\
        --usd ... --yaml ... --texture_dir assets/cameras/oak_camera_texture

本脚本在需要单独查看/导出半径时使用:
    ./app/python.sh tools/cameras/oak_compute_mask_radius.py \\
        --texture_dir assets/cameras/oak_camera_texture \\
        --yaml docs/oak_camera/calibration/fisheye_cams.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sdg_utils.projection_lut import (  # noqa: E402
    _RENDERABLE_DIRZ_MAX,
    compute_mask_radius_from_enter_exr,
)

CAM_KEY_TO_NAME = {
    "cam0": "CAM_A",
    "cam1": "CAM_B",
    "cam2": "CAM_C",
    "cam3": "CAM_D",
}

FISHEYE_CAMERAS = ("CAM_A", "CAM_B", "CAM_C", "CAM_D")


def _calibration_centers_from_yaml(yaml_path: str | Path) -> dict[str, tuple[float, float]]:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)
    out: dict[str, tuple[float, float]] = {}
    for cam_key, cam in data.items():
        if cam.get("camera_model") != "omni":
            continue
        name = CAM_KEY_TO_NAME.get(cam_key, cam.get("rostopic", cam_key))
        intr = cam["intrinsics"]
        out[name] = (float(intr[3]), float(intr[4]))
    return out


def compute_all_from_texture_dir(
    texture_dir: str | Path,
    scale: float = 1.0,
    cameras: tuple[str, ...] = FISHEYE_CAMERAS,
    mask_center: str = "calibration",
    yaml_path: str | Path | None = None,
) -> dict[str, float]:
    tex_dir = Path(texture_dir)
    cal_centers: dict[str, tuple[float, float]] | None = None
    if mask_center == "calibration":
        if yaml_path is None:
            raise ValueError("--mask_center calibration 需要同时提供 --yaml 以读取 cx/cy")
        cal_centers = _calibration_centers_from_yaml(yaml_path)

    out = {}
    for name in cameras:
        exr = tex_dir / f"{name}_rayEnterDirection.exr"
        if not exr.is_file():
            raise FileNotFoundError(f"missing {exr}")
        cx = cy = None
        if mask_center == "calibration":
            cx, cy = cal_centers[name]  # type: ignore[index]
        out[name] = compute_mask_radius_from_enter_exr(
            str(exr), scale=scale, center_x=cx, center_y=cy
        )
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--texture_dir",
        required=True,
        help="LUT 目录（含 CAM_*_rayEnterDirection.exr）",
    )
    parser.add_argument(
        "--yaml",
        required=True,
        help="Kalibr omni yaml（提供 cx/cy 作为 mask 圆心）",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="半径缩放，默认 1.0",
    )
    parser.add_argument(
        "--format",
        choices=("table", "bake_args"),
        default="table",
    )
    parser.add_argument("--output", default=None, help="写入 {CAM_X: radius} yaml")
    parser.add_argument(
        "--mask_center",
        choices=("calibration", "imageCenter"),
        default="calibration",
    )
    args = parser.parse_args()

    radii = compute_all_from_texture_dir(
        args.texture_dir,
        scale=args.scale,
        mask_center=args.mask_center,
        yaml_path=args.yaml,
    )
    source = f"EXR in {Path(args.texture_dir).resolve()}, center={args.mask_center}"

    center_note = (
        f"Kalibr cx/cy, renderable dirZ<{_RENDERABLE_DIRZ_MAX}"
        if args.mask_center == "calibration"
        else f"image center (w/2, h/2), renderable dirZ<{_RENDERABLE_DIRZ_MAX}"
    )
    print(f"# maskRadius @ {center_note}", flush=True)
    print(f"# source: {source}", flush=True)

    if args.format == "bake_args":
        for name in sorted(radii.keys()):
            print(f"--mask_radius {name}={int(round(radii[name]))}", flush=True)
    else:
        for name in sorted(radii.keys()):
            print(f"  {name}: maskRadius={radii[name]:.1f}px", flush=True)

    if args.output:
        out_path = Path(args.output)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        rounded = {k: int(round(v)) for k, v in radii.items()}
        with open(out_path, "w") as f:
            f.write(f"# maskRadius from {source}\n")
            yaml.dump(rounded, f, default_flow_style=False, sort_keys=True)
        print(f"Wrote {out_path}", flush=True)

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
