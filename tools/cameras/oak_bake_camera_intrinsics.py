"""把 Kalibr omni 标定 yaml 中的内参以及自定义分辨率 bake 进相机 USD。

USD 上每个 Camera prim 会被加一组 `omni:calibration:*` custom attribute:
  - 鱼眼相机:从 yaml 写入 xi/fx/fy/cx/cy + distortion + resolution
  - 针孔相机:通过 --resolution 命令行参数单独写入 resolution
  - 鱼眼 maskRadius:可手动 --mask_radius,或默认从 rayEnterDirection.exr 自动估计
  - 保存前默认修正 verticalAperture(与分辨率宽高比一致,避免 Isaac 静默改写)

之后 CameraRig 加载 USD 时不再依赖任何外部文件。

Usage:
    # 内参 + 从 EXR 自动估计 maskRadius + 修正 verticalAperture
    ./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \\
        --usd assets/cameras/oak_camera_4lut_2H30YA.usd \\
        --yaml docs/oak_camera/calibration/fisheye_cams.yaml \\
        --texture_dir assets/cameras/oak_camera_texture

    # EXR 路径也可从 USD 上 generalizedProjectionDirectionTexturePath 解析
    ./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \\
        --usd assets/cameras/oak_camera_4lut.usd \\
        --yaml docs/oak_camera/calibration/fisheye_cams.yaml

    # 手动覆盖某路 maskRadius
    ./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \\
        --usd assets/cameras/oak_camera_4lut_2H30YA.usd \\
        --yaml docs/oak_camera/calibration/fisheye_cams.yaml \\
        --texture_dir assets/cameras/oak_camera_texture \\
        --mask_radius CAM_A=952
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

import yaml
from pxr import Gf, Sdf, Usd, UsdGeom

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from sdg_utils.camera import (  # noqa: E402
    _ensure_square_pixel_aperture,
    _read_resolution_from_prim,
)
from sdg_utils.projection_lut import compute_mask_radius_from_enter_exr  # noqa: E402


CAM_KEY_TO_NAME = {
    "cam0": "CAM_A",
    "cam1": "CAM_B",
    "cam2": "CAM_C",
    "cam3": "CAM_D",
}


def load_yaml(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    cameras = {}
    for cam_key, cam in data.items():
        if cam.get("camera_model") != "omni":
            continue
        name = CAM_KEY_TO_NAME.get(cam_key, cam.get("rostopic", cam_key))
        intr = cam["intrinsics"]
        cameras[name] = {
            "xi": float(intr[0]),
            "fx": float(intr[1]),
            "fy": float(intr[2]),
            "cx": float(intr[3]),
            "cy": float(intr[4]),
            "dist_coeffs": [float(v) for v in cam["distortion_coeffs"]],
            "distortion_model": cam.get("distortion_model", "radtan"),
            "width": int(cam["resolution"][0]),
            "height": int(cam["resolution"][1]),
        }
    return cameras


def set_custom_attr(prim, name, type_name, value):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)
    return attr


def bake(prim, cal):
    """把一个相机的 omni 内参写入 prim 的 custom attributes。"""
    set_custom_attr(prim, "omni:calibration:cameraModel", Sdf.ValueTypeNames.Token, "omni")
    set_custom_attr(prim, "omni:calibration:xi", Sdf.ValueTypeNames.Float, cal["xi"])
    set_custom_attr(prim, "omni:calibration:fx", Sdf.ValueTypeNames.Float, cal["fx"])
    set_custom_attr(prim, "omni:calibration:fy", Sdf.ValueTypeNames.Float, cal["fy"])
    set_custom_attr(prim, "omni:calibration:cx", Sdf.ValueTypeNames.Float, cal["cx"])
    set_custom_attr(prim, "omni:calibration:cy", Sdf.ValueTypeNames.Float, cal["cy"])
    set_custom_attr(
        prim,
        "omni:calibration:distortionModel",
        Sdf.ValueTypeNames.Token,
        cal["distortion_model"],
    )
    set_custom_attr(
        prim,
        "omni:calibration:distortionCoeffs",
        Sdf.ValueTypeNames.Float4,
        Gf.Vec4f(*cal["dist_coeffs"]),
    )
    set_custom_attr(
        prim,
        "omni:calibration:resolution",
        Sdf.ValueTypeNames.Int2,
        Gf.Vec2i(cal["width"], cal["height"]),
    )


def parse_resolution_spec(spec):
    """解析 'NAME=WxH' 字符串。"""
    if "=" not in spec:
        raise argparse.ArgumentTypeError(
            f"--resolution 格式必须是 NAME=WxH,得到: {spec!r}"
        )
    name, dim = spec.split("=", 1)
    if "x" not in dim.lower():
        raise argparse.ArgumentTypeError(
            f"--resolution 格式必须是 NAME=WxH,得到: {spec!r}"
        )
    w_str, h_str = dim.lower().split("x", 1)
    return name.strip(), (int(w_str), int(h_str))


def parse_mask_radius_spec(spec):
    """解析 'NAME=R' 字符串(R 为像素半径,float)。"""
    if "=" not in spec:
        raise argparse.ArgumentTypeError(
            f"--mask_radius 格式必须是 NAME=R,得到: {spec!r}"
        )
    name, val = spec.split("=", 1)
    try:
        return name.strip(), float(val)
    except ValueError as e:
        raise argparse.ArgumentTypeError(
            f"--mask_radius 半径必须是数字,得到: {spec!r}"
        ) from e


def resolve_enter_exr_path(
    cam_name: str,
    prim: Usd.Prim,
    usd_path: str,
    texture_dir: str | None,
) -> str | None:
    """解析 rayEnterDirection.exr 路径: --texture_dir 优先,否则读 USD 属性。"""
    if texture_dir:
        p = os.path.join(texture_dir, f"{cam_name}_rayEnterDirection.exr")
        if os.path.isfile(p):
            return os.path.abspath(p)
    attr = prim.GetAttribute("generalizedProjectionDirectionTexturePath")
    if not attr or not attr.HasAuthoredValue():
        return None
    raw = str(attr.Get().resolvedPath or attr.Get().path)
    if not raw:
        return None
    if os.path.isabs(raw) and os.path.isfile(raw):
        return raw
    rel = os.path.normpath(os.path.join(os.path.dirname(usd_path), raw))
    if os.path.isfile(rel):
        return rel
    return None


def auto_mask_radius(
    cam_name: str,
    cal: dict,
    prim: Usd.Prim,
    usd_path: str,
    texture_dir: str | None,
    *,
    scale: float,
    mask_center: str,
) -> float | None:
    exr_path = resolve_enter_exr_path(cam_name, prim, usd_path, texture_dir)
    if exr_path is None:
        return None
    cx = cy = None
    if mask_center == "calibration":
        cx, cy = cal["cx"], cal["cy"]
    radius = compute_mask_radius_from_enter_exr(
        exr_path, scale=scale, center_x=cx, center_y=cy
    )
    print(
        f"  auto maskRadius {cam_name}: {radius:.1f}px "
        f"(from {exr_path}, center={mask_center})",
        flush=True,
    )
    return radius


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, help="camera rig USD path")
    parser.add_argument(
        "--yaml",
        default=None,
        help="Kalibr omni calibration yaml (可选)",
    )
    parser.add_argument(
        "--texture_dir",
        default=None,
        help="LUT 目录(含 CAM_*_rayEnterDirection.exr)。"
             "未手动指定 --mask_radius 时从此目录或 USD 纹理路径自动估计 maskRadius。",
    )
    parser.add_argument(
        "--resolution",
        action="append",
        default=[],
        type=parse_resolution_spec,
        metavar="NAME=WxH",
        help="为指定相机 bake 分辨率 (针孔相机常用),可多次提供",
    )
    parser.add_argument(
        "--mask_radius",
        action="append",
        default=[],
        type=parse_mask_radius_spec,
        metavar="NAME=R",
        help="手动指定有效成像圆半径(像素),覆盖自动估计",
    )
    parser.add_argument(
        "--auto_mask_radius",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="对 yaml 鱼眼相机从 EXR 自动估计 maskRadius(默认开启)",
    )
    parser.add_argument(
        "--mask_radius_scale",
        type=float,
        default=1.0,
        help="自动估计 maskRadius 时的缩放系数,默认 1.0",
    )
    parser.add_argument(
        "--mask_center",
        choices=("calibration", "imageCenter"),
        default="calibration",
        help="自动估计 mask 圆心:calibration=Kalibr cx/cy(默认);imageCenter=图像中心",
    )
    parser.add_argument(
        "--fix_vertical_aperture",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="保存前将 verticalAperture 设为与分辨率宽高比一致(默认开启)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    usd_path = os.path.abspath(args.usd)
    texture_dir = (
        os.path.abspath(args.texture_dir) if args.texture_dir else None
    )

    cameras = {}
    if args.yaml is not None:
        yaml_path = os.path.abspath(args.yaml)
        print(f"Loading calibration: {yaml_path}", flush=True)
        cameras = load_yaml(yaml_path)
        print(f"  found {len(cameras)} cameras in yaml: {list(cameras.keys())}", flush=True)
    else:
        print("No yaml provided, only --resolution will be applied.", flush=True)

    resolution_overrides = dict(args.resolution)
    if resolution_overrides:
        print(f"Resolution overrides: {resolution_overrides}", flush=True)

    mask_radius_overrides = dict(args.mask_radius)
    if mask_radius_overrides:
        print(f"Mask radius overrides: {mask_radius_overrides}", flush=True)
    print(f"Mask center mode: {args.mask_center}", flush=True)
    if args.auto_mask_radius:
        src = texture_dir or "(USD generalizedProjectionDirectionTexturePath)"
        print(f"Auto maskRadius: enabled, EXR source: {src}", flush=True)
    if args.fix_vertical_aperture:
        print("Fix verticalAperture: enabled", flush=True)

    print(f"Opening USD: {usd_path}", flush=True)
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"failed to open {usd_path}")

    actions = []
    skipped = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Camera):
            continue
        name = prim.GetName()
        cal = cameras.get(name)
        res_override = resolution_overrides.pop(name, None)
        mask_r_override = mask_radius_overrides.pop(name, None)

        if cal is not None:
            if res_override is not None:
                cal = {**cal, "width": res_override[0], "height": res_override[1]}
            bake(prim, cal)

            if mask_r_override is None and args.auto_mask_radius:
                mask_r_override = auto_mask_radius(
                    name,
                    cal,
                    prim,
                    usd_path,
                    texture_dir,
                    scale=args.mask_radius_scale,
                    mask_center=args.mask_center,
                )

            extra = ""
            if mask_r_override is not None:
                set_custom_attr(
                    prim,
                    "omni:calibration:maskRadius",
                    Sdf.ValueTypeNames.Float,
                    float(mask_r_override),
                )
                extra = f", maskRadius={mask_r_override:.0f}px"
            set_custom_attr(
                prim,
                "omni:calibration:maskCenterMode",
                Sdf.ValueTypeNames.Token,
                args.mask_center,
            )
            extra += f", maskCenter={args.mask_center}"
            actions.append(
                f"  baked {name}: model=omni, xi={cal['xi']:.4f}, "
                f"fx={cal['fx']:.2f}, fy={cal['fy']:.2f}, "
                f"cx={cal['cx']:.2f}, cy={cal['cy']:.2f}, "
                f"dist={cal['dist_coeffs']}, res={cal['width']}x{cal['height']}"
                f"{extra}"
            )
        elif res_override is not None or mask_r_override is not None:
            parts = []
            if res_override is not None:
                set_custom_attr(
                    prim,
                    "omni:calibration:resolution",
                    Sdf.ValueTypeNames.Int2,
                    Gf.Vec2i(res_override[0], res_override[1]),
                )
                parts.append(f"res={res_override[0]}x{res_override[1]}")
            if mask_r_override is not None:
                set_custom_attr(
                    prim,
                    "omni:calibration:maskRadius",
                    Sdf.ValueTypeNames.Float,
                    float(mask_r_override),
                )
                parts.append(f"maskRadius={mask_r_override}px")
            actions.append(f"  baked {name}: {', '.join(parts)}")
        else:
            skipped.append(name)

    if args.fix_vertical_aperture:
        n_ap = 0
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Camera):
                continue
            res = _read_resolution_from_prim(prim)
            if res is None:
                res = (1920, 1200)
            _ensure_square_pixel_aperture(prim, res[0], res[1])
            n_ap += 1
        print(f"  fixed verticalAperture on {n_ap} camera(s)", flush=True)

    for line in actions:
        print(line, flush=True)

    if skipped:
        print(
            f"  skipped (no yaml entry & no --resolution/--mask_radius): {skipped}",
            flush=True,
        )

    if resolution_overrides:
        print(
            f"WARNING: --resolution 指定但 USD 中未找到的相机: "
            f"{list(resolution_overrides.keys())}",
            flush=True,
        )
    if mask_radius_overrides:
        print(
            f"WARNING: --mask_radius 指定但 USD 中未找到的相机: "
            f"{list(mask_radius_overrides.keys())}",
            flush=True,
        )

    if not actions:
        print("ERROR: nothing baked.", flush=True)
        return 1

    print(f"Saving USD back: {usd_path}", flush=True)
    stage.GetRootLayer().Save()
    print(f"Done. Baked {len(actions)} cameras.", flush=True)
    return 0


if __name__ == "__main__":
    try:
        ret = main()
    finally:
        simulation_app.close()
    sys.exit(ret or 0)
