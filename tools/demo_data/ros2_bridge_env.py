"""在启动 SimulationApp / enable_extension(ros2.bridge) 之前设置环境变量。

Isaac Sim ROS2 Bridge 在 Linux 上要求启动前已 export（见扩展日志 FastDDS/CycloneDDS 说明）。
"""

from __future__ import annotations

import os
import sys


def _path_parts(env_var: str) -> list[str]:
    return [p for p in os.environ.get(env_var, "").split(":") if p]


def _without_system_ros(paths: list[str]) -> list[str]:
    """去掉 source /opt/ros/.../setup.bash 注入的路径，避免与 Isaac Python 3.11 冲突。"""
    return [p for p in paths if "/opt/ros/" not in p]


def setup_isaac_ros2_bridge_env(
    isaac_root: str | None = None,
    ros_distro: str | None = None,
    rmw_implementation: str | None = None,
) -> str:
    """设置 ROS_DISTRO / RMW_IMPLEMENTATION / LD_LIBRARY_PATH / PYTHONPATH，返回 bridge 扩展路径。"""
    if isaac_root is None:
        # tools/demo_data -> 项目根 -> app
        root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        isaac_root = os.path.realpath(os.path.join(root, "app"))

    distro = ros_distro or os.environ.get("ROS_DISTRO") or "humble"
    rmw = rmw_implementation or os.environ.get("RMW_IMPLEMENTATION") or "rmw_fastrtps_cpp"

    bridge_ext = os.path.join(isaac_root, "exts", "isaacsim.ros2.bridge")
    lib_dir = os.path.join(bridge_ext, distro, "lib")
    rclpy_dir = os.path.join(bridge_ext, distro, "rclpy")

    if not os.path.isdir(lib_dir):
        raise RuntimeError(
            f"ROS2 bridge 库目录不存在: {lib_dir}\n"
            f"请确认 app 指向 Isaac Sim 5.1，且已安装 isaacsim.ros2.bridge 扩展。"
        )
    if not os.path.isdir(rclpy_dir):
        raise RuntimeError(
            f"ROS2 bridge 内置 rclpy 目录不存在: {rclpy_dir}\n"
            f"请确认 isaacsim.ros2.bridge 扩展完整（含 {distro}/rclpy）。"
        )

    os.environ["ROS_DISTRO"] = distro
    os.environ["RMW_IMPLEMENTATION"] = rmw

    ld_parts = _without_system_ros(_path_parts("LD_LIBRARY_PATH"))
    if lib_dir not in ld_parts:
        ld_parts.insert(0, lib_dir)
    os.environ["LD_LIBRARY_PATH"] = ":".join(ld_parts)

    py_parts = _without_system_ros(_path_parts("PYTHONPATH"))
    if rclpy_dir not in py_parts:
        py_parts.insert(0, rclpy_dir)
    os.environ["PYTHONPATH"] = ":".join(py_parts)

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
        f"  export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{ext}/humble/lib\n"
        f"  export PYTHONPATH={ext}/humble/rclpy\n\n"
        "终端 A 勿 source /opt/ros/humble/setup.bash（会与 Isaac Python 3.11 冲突）。\n"
        "推荐: ./tools/demo_data/run_record_camera_rig_trajectory.sh ...",
        file=sys.stderr,
    )
