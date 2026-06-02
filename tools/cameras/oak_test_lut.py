"""
Test script: Render CAM_A with Mei omni LUT vs Pinhole.

Two modes:
  - "cubes": Simple scene with colored cubes (good for verifying LUT distortion)
  - "interior": Full interior scene

Pattern follows official camera_opencv_fisheye.py example.

Usage:
    ./app/python.sh test_cam_a_lut.py [scene_usdc_path|None]
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import os
import sys
import argparse
import math
import cv2
import numpy as np
import isaacsim.core.utils.numpy.rotations as rot_utils
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.sensors.camera import Camera
from pxr import Sdf
import omni.usd


def log(msg):
    print(msg, file=sys.stdout, flush=True)

WIDTH, HEIGHT = 1920, 1200
FX = 1867.917308257951
FY = 1874.460477131494
PIXEL_SIZE_UM = 3.0
LUT_FOV_DEG = 170.0


def parse_args():
    parser = argparse.ArgumentParser(description="Render CAM_A with LUT and pinhole cameras.")
    parser.add_argument("--scene_usdc_path", type=str, default=None, help="usdc scene path.")
    args = parser.parse_args()
    return args


def setup_scene(world, stage, scene_usdc_path):
    if scene_usdc_path is not None:
        scene_prim = stage.DefinePrim("/World/Scene", "Xform")
        scene_prim.GetReferences().AddReference(scene_usdc_path)
        log(f"Loaded scene: {scene_usdc_path}")
        camera_position = np.array([1, -4, 0.5])
        camera_orientation = rot_utils.euler_angles_to_quats(np.array([0, 0, 0]), degrees=True)
        return camera_position, camera_orientation

    world.scene.add_default_ground_plane()
    cubes = [
        (np.array([0.0, 3.0, 0.5]), 1.0, np.array([255, 0, 0])),
        (np.array([2.0, 3.0, 0.5]), 1.0, np.array([0, 255, 0])),
        (np.array([-2.0, 3.0, 0.5]), 1.0, np.array([0, 0, 255])),
        (np.array([0.0, 5.0, 1.0]), 2.0, np.array([255, 255, 0])),
        (np.array([4.0, 2.0, 0.5]), 1.0, np.array([255, 0, 255])),
        (np.array([-4.0, 2.0, 0.5]), 1.0, np.array([0, 255, 255])),
    ]
    for i, (pos, scale, color) in enumerate(cubes):
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/cube_{i}",
                name=f"cube_{i}",
                position=pos,
                scale=scale * np.ones((1, 3)),
                size=1.0,
                color=color,
            )
        )
    log(f"Added {len(cubes)} cubes.")
    camera_position = np.array([0.0, 0.0, 3.0])
    camera_orientation = rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True)
    return camera_position, camera_orientation


def build_camera_params():
    pixel_size_m = PIXEL_SIZE_UM * 1e-6
    focal_length_real = pixel_size_m * (FX + FY) / 2
    horizontal_aperture_real = pixel_size_m * WIDTH
    vertical_aperture_real = pixel_size_m * HEIGHT
    lut_aperture_h = 2 * focal_length_real * math.tan(math.radians(LUT_FOV_DEG / 2))
    lut_aperture_v = lut_aperture_h * HEIGHT / WIDTH
    return {
        "focal_length_real": focal_length_real,
        "horizontal_aperture_real": horizontal_aperture_real,
        "vertical_aperture_real": vertical_aperture_real,
        "lut_aperture_h": lut_aperture_h,
        "lut_aperture_v": lut_aperture_v,
    }


def create_cameras(camera_position, camera_orientation):
    camera_lut = Camera(
        prim_path="/World/CAM_A_LUT",
        position=camera_position,
        frequency=30,
        resolution=(WIDTH, HEIGHT),
        orientation=camera_orientation,
    )
    camera_pin = Camera(
        prim_path="/World/CAM_A_Pinhole",
        position=camera_position,
        frequency=30,
        resolution=(WIDTH, HEIGHT),
        orientation=camera_orientation,
    )
    return camera_lut, camera_pin


def configure_cameras(camera_lut, camera_pin, params, texture_dir):
    camera_lut.set_focal_length(params["focal_length_real"])
    camera_lut.set_focus_distance(3.0)
    camera_lut.set_lens_aperture(0.0)
    camera_lut.set_horizontal_aperture(params["lut_aperture_h"])
    camera_lut.set_vertical_aperture(params["lut_aperture_v"])
    camera_lut.set_clipping_range(0.05, 1.0e5)

    camera_pin.set_focal_length(params["focal_length_real"])
    camera_pin.set_focus_distance(3.0)
    camera_pin.set_lens_aperture(0.0)
    camera_pin.set_horizontal_aperture(params["horizontal_aperture_real"])
    camera_pin.set_vertical_aperture(params["vertical_aperture_real"])
    camera_pin.set_clipping_range(0.05, 1.0e5)

    enter_tex = os.path.abspath(os.path.join(texture_dir, "CAM_A_rayEnterDirection.exr"))
    exit_tex = os.path.abspath(os.path.join(texture_dir, "CAM_A_rayExitPosition.exr"))
    lut_prim = camera_lut.prim
    lut_prim.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token, False).Set("generalizedProjection")
    lut_prim.CreateAttribute("fthetaWidth", Sdf.ValueTypeNames.Float, False).Set(float(WIDTH))
    lut_prim.CreateAttribute("fthetaHeight", Sdf.ValueTypeNames.Float, False).Set(float(HEIGHT))
    lut_prim.CreateAttribute("fthetaCx", Sdf.ValueTypeNames.Float, False).Set(float(WIDTH) / 2.0)
    lut_prim.CreateAttribute("fthetaCy", Sdf.ValueTypeNames.Float, False).Set(float(HEIGHT) / 2.0)
    lut_prim.CreateAttribute("generalizedProjectionDirectionTexturePath", Sdf.ValueTypeNames.Asset, False).Set(
        Sdf.AssetPath(enter_tex)
    )
    lut_prim.CreateAttribute("generalizedProjectionNDCTexturePath", Sdf.ValueTypeNames.Asset, False).Set(
        Sdf.AssetPath(exit_tex)
    )
    log("Cameras configured (using deprecated cameraProjectionType API).")


def render_frames(world, frame_count=30):
    log(f"Rendering {frame_count} frames...")
    for _ in range(frame_count):
        world.step(render=True)


def save_outputs(output_dir, stage, camera_lut, camera_position, params, rgb_lut, rgb_pin):
    suffix = "scene" if stage.GetPrimAtPath("/World/Scene").IsValid() else "cubes"
    report = [f"=== Render Report ({suffix}) ==="]
    for name, rgb in [("LUT", rgb_lut), ("Pinhole", rgb_pin)]:
        if rgb is not None:
            stats = f"min={rgb.min()}, max={rgb.max()}, mean={rgb.mean():.1f}, unique={len(np.unique(rgb))}"
            report.append(f"{name}: {stats}")
            file_name = f"CAM_A_{name.lower()}_{suffix}.png"
            cv2.imwrite(
                os.path.join(output_dir, file_name),
                cv2.cvtColor(rgb.astype(np.uint8), cv2.COLOR_RGB2BGR),
            )
            log(f"Saved: {file_name}")
        else:
            report.append(f"{name}: None")

    for attr_name in [
        "focalLength",
        "horizontalAperture",
        "verticalAperture",
        "fStop",
        "cameraProjectionType",
        "fthetaWidth",
        "fthetaHeight",
        "fthetaCx",
        "fthetaCy",
    ]:
        attr = camera_lut.prim.GetAttribute(attr_name)
        if attr and attr.Get() is not None:
            report.append(f"  {attr_name} = {attr.Get()}")

    report.append(f"position={camera_position}")
    report.append(f"focal_length={params['focal_length_real']*1e3:.3f}mm")
    report.append(f"LUT h_aperture={params['lut_aperture_h']*1e3:.3f}mm (FOV={LUT_FOV_DEG}deg)")
    report.append(f"Pin h_aperture={params['horizontal_aperture_real']*1e3:.3f}mm")

    report_str = "\n".join(report)
    report_path = os.path.join(output_dir, f"render_report_{suffix}.txt")
    with open(report_path, "w") as f:
        f.write(report_str + "\n")
    log(report_str)

    stage.Export(os.path.join(output_dir, f"debug_scene_{suffix}.usd"))


def main():
    args = parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    texture_dir = os.path.join(script_dir, "../../assets/cameras/oak_camera_texture")
    output_dir = os.path.join(script_dir, "../../workdir/render_output")
    os.makedirs(output_dir, exist_ok=True)

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    camera_position, camera_orientation = setup_scene(world, stage, args.scene_usdc_path)
    params = build_camera_params()
    camera_lut, camera_pin = create_cameras(camera_position, camera_orientation)

    world.reset()
    camera_lut.initialize()
    camera_pin.initialize()
    configure_cameras(camera_lut, camera_pin, params, texture_dir)

    render_frames(world, frame_count=30)
    rgb_lut = camera_lut.get_rgb()
    rgb_pin = camera_pin.get_rgb()

    save_outputs(output_dir, stage, camera_lut, camera_position, params, rgb_lut, rgb_pin)
    log("Done!")


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
