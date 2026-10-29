#!/usr/bin/env python3
"""由鱼眼四目标定生成 Isaac Sim 广义投影相机 USD，并 bake 内外参。

标定是 OpenCV / Kalibr equidistant（Kannala-Brandt，k1..k4），不是 OAK 的 Mei omni。
射线仍烤进 generalizedProjection LUT，方便沿用现有 CameraRig。

坐标系:
  - 标定相机系: OpenCV，X 右、Y 下、Z 前。gauge 为 cam3。
  - USD base_link: 原点在 cam3 光心；+X 为 cam3 朝向，+Y 向左，+Z 向上。
  - 相机 prim 本地轴是 RTX: X 右、Y 上、Z 后。光轴为本地 -Z。
  - corrected_lidar 保持标定给出的雷达轴，只把位姿变到 base_link。

底相机光圈与 OAK LUT 工装一致: focalLength=2.208mm, aperture=5.76×3.6mm
(1920×1200)。真实 fx/fy/cx/cy 在 omni:calibration:*，并已写入 LUT。

Usage:
    ./app/python.sh tools/cameras/yuguang_build_camera_usd.py
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
import sys

import numpy as np
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from sdg_utils.projection_lut import (  # noqa: E402
    compute_mask_radius_from_enter_data,
    equidistant_principal_domain,
    equidistant_project,
    equidistant_unproject,
)

# 与 assets/cameras/oak_camera_4lut_2H110SA_20260806.usd 的底相机一致。
FOCAL_LENGTH_MM = 2.208
HORIZONTAL_APERTURE_MM = 5.76

# cam3 OpenCV -> base_link。列是光轴系的基在机体中的坐标。
R_OPT_TO_BODY = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)
R_CV_TO_ISAAC = np.diag([1.0, -1.0, -1.0, 1.0])

CAM_KEYS = ("cam0", "cam1", "cam2", "cam3")


def _load_oak_lut():
    path = os.path.join(ROOT, "tools", "cameras", "oak_generate_lut_textures.py")
    spec = importlib.util.spec_from_file_location("oak_generate_lut_textures", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _as4(rows) -> np.ndarray:
    return np.array(rows, dtype=np.float64)


def _pose(rotation: np.ndarray, translation: np.ndarray | None = None) -> np.ndarray:
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = rotation
    if translation is not None:
        out[:3, 3] = translation
    return out


R_OPT_TO_BODY_4 = _pose(R_OPT_TO_BODY)


def camera_pose_in_body(t_cam_from_cam3: np.ndarray) -> np.ndarray:
    """cam 相对 base_link 的 RTX 位姿。点变换: p_body = T @ p_cam_rtx。"""
    t_cam3_from_cam = np.linalg.inv(t_cam_from_cam3)
    t_body_from_cam_cv = R_OPT_TO_BODY_4 @ t_cam3_from_cam
    return t_body_from_cam_cv @ R_CV_TO_ISAAC


def lidar_pose_in_body(t_cam3_from_lidar: np.ndarray) -> np.ndarray:
    """雷达帧相对 base_link。不把雷达轴改成相机光学轴。"""
    return R_OPT_TO_BODY_4 @ t_cam3_from_lidar


def recover_t_cam_from_cam3(t_body_from_cam_rtx: np.ndarray) -> np.ndarray:
    t_body_from_cam_cv = t_body_from_cam_rtx @ R_CV_TO_ISAAC
    t_cam3_from_cam = np.linalg.inv(R_OPT_TO_BODY_4) @ t_body_from_cam_cv
    return np.linalg.inv(t_cam3_from_cam)


def rot_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    r = a @ b.T
    cosine = float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def euler_xyz_deg(rotation: np.ndarray) -> np.ndarray:
    """内旋 XYZ，与 scipy Rotation.as_euler('XYZ') 一致。"""
    sy = float(np.clip(rotation[0, 2], -1.0, 1.0))
    y = math.asin(sy)
    if abs(rotation[0, 2]) < 0.999999:
        x = math.atan2(-rotation[1, 2], rotation[2, 2])
        z = math.atan2(-rotation[0, 1], rotation[0, 0])
    else:
        x = math.atan2(rotation[2, 1], rotation[1, 1])
        z = 0.0
    return np.degrees(np.array([x, y, z], dtype=np.float64))


def np_to_gf(matrix: np.ndarray) -> Gf.Matrix4d:
    """numpy 列向量约定 -> USD 行向量 Matrix4d。"""
    transposed = np.asarray(matrix, dtype=np.float64).T
    return Gf.Matrix4d(*[float(transposed[i, j]) for i in range(4) for j in range(4)])


def gf_to_np(matrix: Gf.Matrix4d) -> np.ndarray:
    raw = np.array([[matrix[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
    return raw.T


def set_attr(prim, name: str, type_name, value, custom: bool = True):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=custom)
    attr.Set(value)
    return attr


def load_rig(yaml_path: str) -> dict:
    with open(yaml_path, "r") as handle:
        data = yaml.safe_load(handle)
    cameras = {}
    for key in CAM_KEYS:
        cam = data["cameras"][key]
        if cam.get("distortion_model") != "equidistant":
            raise RuntimeError(f"{key} distortion_model={cam.get('distortion_model')}, 需要 equidistant")
        intr = [float(v) for v in cam["intrinsics"]]
        cameras[key] = {
            "name": key.replace("cam", "CAM_"),
            "fx": intr[0],
            "fy": intr[1],
            "cx": intr[2],
            "cy": intr[3],
            "dist": [float(v) for v in cam["distortion_coeffs"]],
            "width": int(cam["resolution"][0]),
            "height": int(cam["resolution"][1]),
            "T_cam_from_cam3": _as4(cam["T_camera_from_cam3"]),
        }
    return {
        "version": str(data.get("version", "")),
        "cameras": cameras,
        "T_cam3_from_lidar": _as4(data["T_cam3_from_corrected_lidar"]),
    }


def verify_roundtrip(cam: dict, samples: int = 800) -> float:
    theta_limit, theta_d_max = equidistant_principal_domain(cam["dist"])
    rng = np.random.default_rng(0)
    theta = rng.uniform(0.0, theta_limit * 0.98, samples)
    phi = rng.uniform(-np.pi, np.pi, samples)
    direction = np.stack(
        [np.sin(theta) * np.cos(phi), np.sin(theta) * np.sin(phi), np.cos(theta)],
        axis=0,
    )
    u, v, valid = equidistant_project(
        direction[0], direction[1], direction[2],
        cam["fx"], cam["fy"], cam["cx"], cam["cy"], cam["dist"],
        theta_limit=theta_limit,
    )
    inside = valid & (u >= 0) & (u < cam["width"]) & (v >= 0) & (v < cam["height"])
    dx, dy, dz, ray_ok = equidistant_unproject(
        u[inside], v[inside],
        cam["fx"], cam["fy"], cam["cx"], cam["cy"], cam["dist"],
        theta_limit=theta_limit, theta_d_max=theta_d_max,
    )
    src = direction[:, inside]
    got = np.stack([dx, dy, dz], axis=0)
    err = np.degrees(np.arccos(np.clip(np.sum(src * got, axis=0), -1.0, 1.0)))
    err = err[ray_ok]
    if err.size == 0:
        raise RuntimeError(f"{cam['name']} roundtrip 没有有效样本")
    mean_err = float(np.mean(err))
    max_err = float(np.max(err))
    print(
        f"  roundtrip {cam['name']}: mean {mean_err:.6f} deg, max {max_err:.6f} deg, "
        f"n={err.size}, theta_limit={math.degrees(theta_limit):.2f} deg, "
        f"theta_d_max={theta_d_max:.4f}",
        flush=True,
    )
    if mean_err > 1e-3 or max_err > 0.05:
        raise RuntimeError(f"{cam['name']} 等距模型往返误差过大")
    return mean_err


def generate_textures(cam: dict, texture_dir: str, oak_lut, tex_scale: int) -> dict:
    name = cam["name"]
    width, height = cam["width"], cam["height"]
    theta_limit, theta_d_max = equidistant_principal_domain(cam["dist"])
    enter_w, enter_h = width * tex_scale, height * tex_scale
    exit_size = max(width, height) * tex_scale

    u_ndc = np.linspace(0.0, 1.0, enter_w)
    v_ndc = np.linspace(0.0, 1.0, enter_h)
    grid_u, grid_v = np.meshgrid(u_ndc, v_ndc)
    dx, dy, dz, valid = equidistant_unproject(
        grid_u * width, grid_v * height,
        cam["fx"], cam["fy"], cam["cx"], cam["cy"], cam["dist"],
        theta_limit=theta_limit, theta_d_max=theta_d_max,
    )
    enter = np.stack([dx, -dy, -dz], axis=-1).astype(np.float32)
    enter_path = os.path.join(texture_dir, f"{name}_rayEnterDirection.exr")
    oak_lut.save_exr(enter_path, enter)

    dir_x, dir_y, dir_z = oak_lut.get_octahedral_directions(exit_size, exit_size)
    u_px, v_px, proj_ok = equidistant_project(
        dir_x, -dir_y, -dir_z,
        cam["fx"], cam["fy"], cam["cx"], cam["cy"], cam["dist"],
        theta_limit=theta_limit,
    )
    u_ndc_out = u_px / width
    v_ndc_out = v_px / height
    in_image = proj_ok & (u_ndc_out >= 0.0) & (u_ndc_out <= 1.0) & (v_ndc_out >= 0.0) & (v_ndc_out <= 1.0)
    exit_img = np.stack(
        [
            np.where(in_image, u_ndc_out, -1.0),
            np.where(in_image, v_ndc_out, -1.0),
        ],
        axis=-1,
    ).astype(np.float32)
    exit_path = os.path.join(texture_dir, f"{name}_rayExitPosition.exr")
    oak_lut.save_exr(exit_path, exit_img)

    cx = cam["cx"] * enter_w / width
    cy = cam["cy"] * enter_h / height
    radius_tex = compute_mask_radius_from_enter_data(enter, center_x=cx, center_y=cy)
    radius_px = float(radius_tex) * width / enter_w
    valid_ratio = float(np.mean(valid))
    print(
        f"  {name}: valid {valid_ratio * 100:.1f}%, maskRadius={radius_px:.1f}px "
        f"@ ({cam['cx']:.2f}, {cam['cy']:.2f})",
        flush=True,
    )
    return {
        "enter_path": enter_path,
        "exit_path": exit_path,
        "mask_radius": radius_px,
        "theta_limit_deg": math.degrees(theta_limit),
        "valid_ratio": valid_ratio,
    }


def _add_identity_xform(prim):
    xform = UsdGeom.Xformable(prim)
    xform.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))
    xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    xform.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(1.0, 1.0, 1.0))


def _set_matrix(prim, matrix: np.ndarray):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    op = xform.AddTransformOp(UsdGeom.XformOp.PrecisionDouble)
    op.Set(np_to_gf(matrix))
    written = gf_to_np(op.Get())
    if not np.allclose(written, matrix, atol=1e-9):
        raise RuntimeError(f"{prim.GetPath()} 变换写回后与输入不一致")


def _rel_asset(usd_path: str, file_path: str) -> str:
    rel = os.path.relpath(file_path, os.path.dirname(usd_path))
    return rel.replace(os.sep, "/")


def write_usd(usd_path: str, rig: dict, textures: dict) -> None:
    if os.path.exists(usd_path):
        os.remove(usd_path)
    stage = Usd.Stage.CreateNew(usd_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.GetRootLayer().customLayerData = {
        "creator": "tools/cameras/yuguang_build_camera_usd.py",
        "calibrationVersion": rig["version"],
        "baseFrame": "origin at cam3, +X cam3 forward, +Y left, +Z up",
    }

    root = UsdGeom.Xform.Define(stage, "/camera")
    stage.SetDefaultPrim(root.GetPrim())
    _add_identity_xform(root.GetPrim())

    base = UsdGeom.Xform.Define(stage, "/camera/base_link")
    _add_identity_xform(base.GetPrim())
    set_attr(base.GetPrim(), "omni:rig:frame", Sdf.ValueTypeNames.Token, "cam3_body")
    set_attr(
        base.GetPrim(),
        "omni:rig:description",
        Sdf.ValueTypeNames.String,
        "origin at cam3 optical center; +X forward, +Y left, +Z up",
    )

    mount = UsdGeom.Xform.Define(stage, "/camera/base_link/camera")
    _add_identity_xform(mount.GetPrim())

    sample = next(iter(rig["cameras"].values()))
    vap = HORIZONTAL_APERTURE_MM * sample["height"] / sample["width"]

    for key in CAM_KEYS:
        cam = rig["cameras"][key]
        name = cam["name"]
        prim = UsdGeom.Camera.Define(stage, f"/camera/base_link/camera/{name}").GetPrim()
        pose = camera_pose_in_body(cam["T_cam_from_cam3"])
        _set_matrix(prim, pose)
        recovered = recover_t_cam_from_cam3(pose)
        rot_err = rot_angle_deg(recovered[:3, :3], cam["T_cam_from_cam3"][:3, :3])
        trans_err = float(np.linalg.norm(recovered[:3, 3] - cam["T_cam_from_cam3"][:3, 3]))
        if rot_err > 1e-4 or trans_err > 1e-6:
            raise RuntimeError(f"{name} 外参闭环失败: rot {rot_err} deg, trans {trans_err} m")

        camera = UsdGeom.Camera(prim)
        camera.CreateFocalLengthAttr(FOCAL_LENGTH_MM)
        camera.CreateHorizontalApertureAttr(HORIZONTAL_APERTURE_MM)
        camera.CreateVerticalApertureAttr(vap)
        camera.CreateHorizontalApertureOffsetAttr(0.0)
        camera.CreateVerticalApertureOffsetAttr(0.0)
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 100000.0))
        camera.CreateFocusDistanceAttr(28.0)
        camera.CreateProjectionAttr(UsdGeom.Tokens.perspective)
        UsdGeom.Imageable(prim).CreateVisibilityAttr().Set(UsdGeom.Tokens.invisible)

        tex = textures[name]
        set_attr(prim, "cameraProjectionType", Sdf.ValueTypeNames.Token, "generalizedProjection", custom=False)
        set_attr(
            prim,
            "generalizedProjectionDirectionTexturePath",
            Sdf.ValueTypeNames.Asset,
            Sdf.AssetPath(_rel_asset(usd_path, tex["enter_path"])),
            custom=False,
        )
        set_attr(
            prim,
            "generalizedProjectionNDCTexturePath",
            Sdf.ValueTypeNames.Asset,
            Sdf.AssetPath(_rel_asset(usd_path, tex["exit_path"])),
            custom=False,
        )
        set_attr(prim, "omni:calibration:cameraModel", Sdf.ValueTypeNames.Token, "equidistant")
        set_attr(prim, "omni:calibration:distortionModel", Sdf.ValueTypeNames.Token, "equidistant")
        set_attr(prim, "omni:calibration:xi", Sdf.ValueTypeNames.Float, 0.0)
        set_attr(prim, "omni:calibration:fx", Sdf.ValueTypeNames.Float, cam["fx"])
        set_attr(prim, "omni:calibration:fy", Sdf.ValueTypeNames.Float, cam["fy"])
        set_attr(prim, "omni:calibration:cx", Sdf.ValueTypeNames.Float, cam["cx"])
        set_attr(prim, "omni:calibration:cy", Sdf.ValueTypeNames.Float, cam["cy"])
        set_attr(
            prim,
            "omni:calibration:distortionCoeffs",
            Sdf.ValueTypeNames.Float4,
            Gf.Vec4f(*cam["dist"]),
        )
        set_attr(
            prim,
            "omni:calibration:resolution",
            Sdf.ValueTypeNames.Int2,
            Gf.Vec2i(cam["width"], cam["height"]),
        )
        set_attr(prim, "omni:calibration:maskRadius", Sdf.ValueTypeNames.Float, float(tex["mask_radius"]))
        set_attr(prim, "omni:calibration:maskCenterMode", Sdf.ValueTypeNames.Token, "calibration")
        set_attr(prim, "omni:calibration:source", Sdf.ValueTypeNames.Token, key)

    lidar = UsdGeom.Xform.Define(stage, "/camera/base_link/corrected_lidar")
    lidar_pose = lidar_pose_in_body(rig["T_cam3_from_lidar"])
    _set_matrix(lidar.GetPrim(), lidar_pose)
    recovered_lidar = np.linalg.inv(R_OPT_TO_BODY_4) @ lidar_pose
    if not np.allclose(recovered_lidar, rig["T_cam3_from_lidar"], atol=1e-8):
        raise RuntimeError("雷达外参闭环失败")
    set_attr(lidar.GetPrim(), "omni:rig:frame", Sdf.ValueTypeNames.Token, "corrected_lidar")

    stage.GetRootLayer().Save()
    print(f"Saved USD: {usd_path}", flush=True)


def print_bake_report(rig: dict, textures: dict) -> None:
    sample = next(iter(rig["cameras"].values()))
    vap = HORIZONTAL_APERTURE_MM * sample["height"] / sample["width"]
    half_fov = math.degrees(math.atan(HORIZONTAL_APERTURE_MM / (2.0 * FOCAL_LENGTH_MM)))
    print("\n===== bake 参数 =====", flush=True)
    print(f"calibration: {rig['version']}", flush=True)
    print("base_link: cam3 光心, +X 前 / +Y 左 / +Z 上", flush=True)
    print(
        f"底相机: focalLength={FOCAL_LENGTH_MM:.3f} mm, "
        f"horizontalAperture={HORIZONTAL_APERTURE_MM:.3f} mm, "
        f"verticalAperture={vap:.3f} mm, "
        f"水平半视场={half_fov:.2f} deg",
        flush=True,
    )
    print(
        f"{'cam':<8}{'fx':>10}{'fy':>10}{'cx':>10}{'cy':>10}"
        f"{'maskR':>8}{'theta':>8}{'X':>10}{'Y':>10}{'Z':>10}{'rX':>8}{'rY':>8}{'rZ':>8}",
        flush=True,
    )
    for key in CAM_KEYS:
        cam = rig["cameras"][key]
        pose = camera_pose_in_body(cam["T_cam_from_cam3"])
        euler = euler_xyz_deg(pose[:3, :3])
        tex = textures[cam["name"]]
        t = pose[:3, 3]
        print(
            f"{cam['name']:<8}{cam['fx']:10.3f}{cam['fy']:10.3f}{cam['cx']:10.3f}{cam['cy']:10.3f}"
            f"{tex['mask_radius']:8.1f}{tex['theta_limit_deg']:8.1f}"
            f"{t[0]:10.4f}{t[1]:10.4f}{t[2]:10.4f}"
            f"{euler[0]:8.2f}{euler[1]:8.2f}{euler[2]:8.2f}",
            flush=True,
        )
    lidar = lidar_pose_in_body(rig["T_cam3_from_lidar"])
    euler = euler_xyz_deg(lidar[:3, :3])
    t = lidar[:3, 3]
    print(
        f"{'lidar':<8}{'':10}{'':10}{'':10}{'':10}{'':8}{'':8}"
        f"{t[0]:10.4f}{t[1]:10.4f}{t[2]:10.4f}"
        f"{euler[0]:8.2f}{euler[1]:8.2f}{euler[2]:8.2f}",
        flush=True,
    )
    print("theta 列是等距模型主分支半视场(度)。平移单位米，欧拉角为 base_link 下 XYZ 内旋(度)。", flush=True)
    print("distortion k1..k4:", flush=True)
    for key in CAM_KEYS:
        cam = rig["cameras"][key]
        coeffs = ", ".join(f"{v:.8f}" for v in cam["dist"])
        print(f"  {cam['name']}: [{coeffs}]", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--yaml",
        default=os.path.join(ROOT, "assets/cameras/yuguang_4fisheye/rig_calibration.yaml"),
    )
    parser.add_argument(
        "--usd",
        default=os.path.join(ROOT, "assets/cameras/yuguang_4fisheye.usd"),
    )
    parser.add_argument(
        "--texture_dir",
        default=os.path.join(ROOT, "assets/cameras/yuguang_4fisheye_texture"),
    )
    parser.add_argument("--tex_scale", type=int, default=1)
    return parser.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.texture_dir, exist_ok=True)
    os.makedirs(os.path.dirname(os.path.abspath(args.usd)), exist_ok=True)
    rig = load_rig(os.path.abspath(args.yaml))
    oak_lut = _load_oak_lut()
    print(f"calibration {rig['version']}: {os.path.abspath(args.yaml)}", flush=True)

    textures = {}
    for key in CAM_KEYS:
        cam = rig["cameras"][key]
        print(f"\n{cam['name']} ({key})", flush=True)
        verify_roundtrip(cam)
        textures[cam["name"]] = generate_textures(cam, args.texture_dir, oak_lut, args.tex_scale)

    write_usd(os.path.abspath(args.usd), rig, textures)
    print_bake_report(rig, textures)
    return 0


if __name__ == "__main__":
    sys.exit(main())
