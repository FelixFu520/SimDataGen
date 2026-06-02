#!/usr/bin/env python3
"""一键搭建一套内参扰动相机组（复制 USD → 生成 yaml → LUT → bake → 写 LUT 路径）。

Usage:
    ./app/python.sh tools/setup_intrinsics_change_variant.py --variant pinhole_like
    ./app/python.sh tools/setup_intrinsics_change_variant.py --variant fisheye_like

可选跳过耗时步骤：
    --skip-lut   已有 EXR 时跳过 generate_lut_textures
    --skip-bake  仅更新 LUT 路径
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

VARIANTS = {
    "pinhole_like": {
        "profile": "pinhole_like",
        "suffix": "intrinsics_change_pinhole",
        "camera_name": "4cam-lut-2H30YA-intrinsics_change_pinhole",
    },
    "fisheye_like": {
        "profile": "fisheye_like",
        "suffix": "intrinsics_change_fisheye",
        "camera_name": "4cam-lut-2H30YA-intrinsics_change_fisheye",
    },
}

def run(cmd: list[str], cwd: Path) -> None:
    print(f"\n>>> {' '.join(cmd)}", flush=True)
    subprocess.run(cmd, cwd=str(cwd), check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--variant",
        required=True,
        choices=sorted(VARIANTS.keys()),
    )
    parser.add_argument("--skip-lut", action="store_true")
    parser.add_argument("--skip-bake", action="store_true")
    args = parser.parse_args()

    cfg = VARIANTS[args.variant]
    suffix = cfg["suffix"]
    profile = cfg["profile"]

    base_usd = ROOT / "assets/cameras/oak_camera_4lut_2H30YA.usd"
    out_usd = ROOT / f"assets/cameras/oak_camera_4lut_2H30YA_{suffix}.usd"
    yaml_path = ROOT / f"docs/oak_camera_intrinsics_change/fisheye_cams_{profile}.yaml"
    tex_dir = ROOT / f"assets/cameras/oak_camera_texture_{suffix}"
    py = str(ROOT / "app/python.sh")

    if not yaml_path.is_file():
        run(
            [
                py,
                "tools/generate_perturbed_fisheye_yaml.py",
                "--profile",
                profile,
                "--output",
                str(yaml_path),
            ],
            ROOT,
        )

    if not out_usd.is_file():
        print(f"Copy {base_usd} -> {out_usd}", flush=True)
        shutil.copy2(base_usd, out_usd)

    if not args.skip_lut:
        run(
            [
                py,
                "tools/generate_lut_textures.py",
                "--yaml",
                str(yaml_path.relative_to(ROOT)),
                "--output_dir",
                str(tex_dir.relative_to(ROOT)),
            ],
            ROOT,
        )

    if not args.skip_bake:
        run(
            [
                py,
                "tools/cameras/oak_bake_camera_intrinsics.py",
                "--usd",
                str(out_usd.relative_to(ROOT)),
                "--yaml",
                str(yaml_path.relative_to(ROOT)),
                "--texture_dir",
                str(tex_dir.relative_to(ROOT)),
                "--mask_center",
                "calibration",
                "--resolution",
                "CAM_Front=1920x1200",
                "--resolution",
                "CAM_Back=1920x1200",
            ],
            ROOT,
        )

    run(
        [
            py,
            "tools/set_camera_lut_texture_paths.py",
            "--usd",
            str(out_usd.relative_to(ROOT)),
            "--texture_dir",
            str(tex_dir.relative_to(ROOT)),
        ],
        ROOT,
    )

    print(f"\nDone. camera_name={cfg['camera_name']}", flush=True)
    print(f"  USD:   {out_usd}", flush=True)
    print(f"  YAML:  {yaml_path}", flush=True)
    print(f"  LUT:   {tex_dir}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
