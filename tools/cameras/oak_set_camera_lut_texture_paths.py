#!/usr/bin/env python3
"""把鱼眼相机的 generalizedProjection LUT 纹理路径写入相机 USD（相对 USD 文件目录）。

对应 docs/camera_OAK_H30YA_intrinsics_extrinsics_perturbed.md 步骤 3.3，无需在 Isaac Sim 里手改。

Usage:
    ./app/python.sh tools/set_camera_lut_texture_paths.py \\
        --usd assets/cameras/oak_camera_4lut_2H30YA_intrinsics_change_pinhole.usd \\
        --texture_dir assets/cameras/oak_camera_texture_intrinsics_change_pinhole

    # 仅更新指定相机
    ./app/python.sh tools/set_camera_lut_texture_paths.py \\
        --usd ... --texture_dir ... --cameras CAM_A CAM_C
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Sdf, Usd, UsdGeom  # noqa: E402

FISHEYE_CAMERAS = ("CAM_A", "CAM_B", "CAM_C", "CAM_D")
ATTR_DIRECTION = "generalizedProjectionDirectionTexturePath"
ATTR_NDC = "generalizedProjectionNDCTexturePath"


def _rel_to_usd(usd_dir: Path, file_path: Path) -> str:
    """EXR 路径相对 USD 所在目录，使用 USD 习惯的 '/' 分隔。"""
    rel = os.path.relpath(file_path.resolve(), usd_dir.resolve())
    return rel.replace(os.sep, "/")


def texture_paths(usd_dir: Path, texture_dir: Path, cam_name: str) -> tuple[str, str]:
    enter = texture_dir / f"{cam_name}_rayEnterDirection.exr"
    exit_ = texture_dir / f"{cam_name}_rayExitPosition.exr"
    if not enter.is_file():
        raise FileNotFoundError(f"missing LUT: {enter}")
    if not exit_.is_file():
        raise FileNotFoundError(f"missing LUT: {exit_}")
    return _rel_to_usd(usd_dir, enter), _rel_to_usd(usd_dir, exit_)


def set_lut_paths(usd_path: str, texture_dir: str, cameras: tuple[str, ...]) -> int:
    usd_file = Path(usd_path).resolve()
    usd_dir = usd_file.parent
    tex_dir = Path(texture_dir).resolve()
    if not tex_dir.is_dir():
        raise FileNotFoundError(f"texture_dir not found: {tex_dir}")

    stage = Usd.Stage.Open(str(usd_file))
    if stage is None:
        raise RuntimeError(f"failed to open USD: {usd_path}")

    updated = []
    missing_cams = set(cameras)

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Camera):
            continue
        name = prim.GetName()
        if name not in cameras:
            continue
        missing_cams.discard(name)
        enter_path, exit_path = texture_paths(usd_dir, tex_dir, name)

        for attr_name, path in (
            (ATTR_DIRECTION, enter_path),
            (ATTR_NDC, exit_path),
        ):
            attr = prim.GetAttribute(attr_name)
            if not attr:
                attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Asset, False)
            attr.Set(Sdf.AssetPath(path))

        updated.append((name, enter_path, exit_path))

    if missing_cams:
        raise RuntimeError(
            f"USD 中未找到相机 prim: {sorted(missing_cams)} (usd={usd_path})"
        )
    if not updated:
        raise RuntimeError(f"no cameras updated in {usd_path}")

    stage.GetRootLayer().Save()
    print(f"Saved USD: {usd_path}", flush=True)
    for name, enter_path, exit_path in updated:
        print(f"  {name}:", flush=True)
        print(f"    {ATTR_DIRECTION}: {enter_path}", flush=True)
        print(f"    {ATTR_NDC}:       {exit_path}", flush=True)
    return 0


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, help="相机 rig USD（会被原地修改）")
    parser.add_argument(
        "--texture_dir",
        required=True,
        help="LUT EXR 目录（含 CAM_*_rayEnterDirection.exr / *_rayExitPosition.exr）",
    )
    parser.add_argument(
        "--cameras",
        nargs="*",
        default=list(FISHEYE_CAMERAS),
        help=f"要更新的鱼眼相机名，默认 {FISHEYE_CAMERAS}",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    return set_lut_paths(usd_path, args.texture_dir, tuple(args.cameras))


if __name__ == "__main__":
    try:
        ret = main()
    finally:
        simulation_app.close()
    sys.exit(ret or 0)
