"""测试 CameraRig: 加载相机组 USD 并打印所有相机的内外参。

Usage:
    ./app/python.sh tools/cameras/print_cameraRig.py --usd assets/cameras/oak_camera_4lut.usd
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import argparse
import os
import sys

from loguru import logger

# 配置日志
# 移除默认的日志处理器
logger.remove()  
# 配置日志格式
log_format = (
    "<green>{time:YYYY-MM-DD HH:mm:ss.SSS}</green> | "
    "<level>{level: <8}</level> | "
    "<cyan>{name}</cyan>:<cyan>{function}</cyan>:<cyan>{line}</cyan> | "
    "<level>{message}</level>"
)
# 添加控制台输出
logger.add(
    sys.stdout,
    format="{message}",
    level="INFO",
    colorize=True
)

import omni.usd
from isaacsim.core.api import World

# 项目根目录 (tools/cameras/ -> 上三级)
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from sdg_utils.camera import CameraRig


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--usd",
        default="assets/cameras/oak_camera_4lut.usd",
        help="相机组 USD 路径 (相对项目根目录或绝对路径)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    camera_usd_path = args.usd
    if not os.path.isabs(camera_usd_path):
        camera_usd_path = os.path.join(ROOT_DIR, camera_usd_path)
    camera_usd_path = os.path.normpath(camera_usd_path)

    if not os.path.isfile(camera_usd_path):
        print(f"ERROR: camera USD not found: {camera_usd_path}", file=sys.stderr, flush=True)
        sys.exit(1)

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    rig = CameraRig(
        camera_usd_path=camera_usd_path,
        world=world,
        stage=stage,
        rig_prim_path="/World/camera_rig",
    )

    world.reset()
    rig.initialize()

    rig.print_all()


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
