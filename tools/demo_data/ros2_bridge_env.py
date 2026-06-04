"""在启动 SimulationApp / enable_extension(ros2.bridge) 之前设置环境变量。

Isaac Sim ROS2 Bridge 在 Linux 上要求启动前已 export（见扩展日志 FastDDS/CycloneDDS 说明）。
"""

from __future__ import annotations

import os
import sys


def setup_isaac_ros2_bridge_env(
    isaac_root: str | None = None,
    ros_distro: str | None = None,
    rmw_implementation: str | None = None,
) -> str:
    """设置 ROS_DISTRO / RMW_IMPLEMENTATION / LD_LIBRARY_PATH，返回 bridge 扩展路径。"""
    if isaac_root is None:
        # tools/demo_data -> 项目根 -> app
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        isaac_root = os.path.realpath(os.path.join(root, "app"))

    distro = ros_distro or os.environ.get("ROS_DISTRO") or "humble"
    rmw = rmw_implementation or os.environ.get("RMW_IMPLEMENTATION") or "rmw_fastrtps_cpp"

    bridge_ext = os.path.join(isaac_root, "exts", "isaacsim.ros2.bridge")
    lib_dir = os.path.join(bridge_ext, distro, "lib")

    if not os.path.isdir(lib_dir):
        raise RuntimeError(
            f"ROS2 bridge 库目录不存在: {lib_dir}\n"
            f"请确认 app 指向 Isaac Sim 5.1，且已安装 isaacsim.ros2.bridge 扩展。"
        )

    os.environ["ROS_DISTRO"] = distro
    os.environ["RMW_IMPLEMENTATION"] = rmw

    parts = [p for p in os.environ.get("LD_LIBRARY_PATH", "").split(":") if p]
    if lib_dir not in parts:
        parts.insert(0, lib_dir)
    os.environ["LD_LIBRARY_PATH"] = ":".join(parts)

    return bridge_ext


def print_setup_hint(isaac_root: str | None = None) -> None:
    if isaac_root is None:
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        isaac_root = os.path.realpath(os.path.join(root, "app"))
    ext = os.path.join(isaac_root, "exts", "isaacsim.ros2.bridge")
    print(
        "ROS2 Bridge 启动失败。请在运行 ./app/python.sh **之前** 执行：\n\n"
        "  export ROS_DISTRO=humble\n"
        "  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n"
        f"  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{ext}/humble/lib\n\n"
        "或直接使用: ./tools/demo_data/run_record_camera_rig_trajectory.sh ...",
        file=sys.stderr,
    )
