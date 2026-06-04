#!/usr/bin/env python3
"""键盘遥操 CameraRig：离散步长移动 + 录制。

在**系统 Python + ROS2** 环境运行:
    source /opt/ros/humble/setup.bash
    python tools/demo_data/keyboard_camera_rig_teleop.py

按键（终端需聚焦）:
    a / d        : CameraRig 向左 / 向右
    w / s        : CameraRig 向前 / 向后
    z / c        : yaw 左 / 右旋转（每按一次 10°）
    q / e        : 减小 / 增大步长
    j / k        : 开始 / 停止录制
    x            : 退出

Z 轴固定为仿真/键盘设置的初始高度，不可键盘调整。

发布:
    /camera_rig/nudge      std_msgs/Float64MultiArray  [lateral, longitudinal, dyaw] 米、度
    /camera_rig/init_pose  std_msgs/Float64MultiArray  6 元组位姿
    /camera_rig/record     std_msgs/String             start / stop

订阅:
    /camera_rig/state      std_msgs/Float64MultiArray  当前 rig 位姿
"""

from __future__ import annotations

import argparse
import shutil
import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64MultiArray, String

YAW_STEP_DEG = 10.0

HELP_LINES = [
    "========== CameraRig 键盘遥操（步长模式）==========",
    "  a / d     向左 / 向右",
    "  w / s     向前 / 向后",
    "  z / c     yaw  左 / 右   （每按一次 10°）",
    "  q / e     步长  减小 / 增大",
    "  j / k     开始录制 / 停止并保存",
    "  x         退出",
    "  Z 固定为初始高度，不随按键改变",
    "================================================",
]


def _read_key(timeout: float = 0.05) -> str:
    if select.select([sys.stdin], [], [], timeout)[0]:
        ch = sys.stdin.read(1)
        if ch == "\x1b":
            rest = sys.stdin.read(2) if select.select([sys.stdin], [], [], 0)[0] else ""
            return ch + rest
        return ch
    return ""


class TeleopDisplay:
    def __init__(self) -> None:
        self._use_alt = sys.stdout.isatty()

    def enter(self) -> None:
        if self._use_alt:
            sys.stdout.write("\033[?1049h\033[H\033[2J")
            for line in HELP_LINES:
                print(line)
            print("----------------------------------------")
            sys.stdout.flush()

    def leave(self) -> None:
        if self._use_alt:
            sys.stdout.write("\033[?1049l")
            sys.stdout.flush()

    def set_status(self, text: str) -> None:
        if self._use_alt:
            sys.stdout.write(f"\033[{len(HELP_LINES) + 2};1H\033[2K{text}\033[0m")
        else:
            width = shutil.get_terminal_size((80, 20)).columns
            sys.stdout.write("\r" + text[: max(width - 1, 20)].ljust(width - 1))
        sys.stdout.flush()


class KeyboardTeleop(Node):
    def __init__(
        self,
        step_size: float,
        step_step: float,
        min_step: float,
        max_step: float,
        init_pose: list[float] | None,
    ):
        super().__init__("keyboard_camera_rig_teleop")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

        self.pub_nudge = self.create_publisher(Float64MultiArray, "/camera_rig/nudge", 10)
        self.pub_record = self.create_publisher(String, "/camera_rig/record", 10)
        self.pub_init_pose = self.create_publisher(Float64MultiArray, "/camera_rig/init_pose", 10)
        self.create_subscription(
            Float64MultiArray, "/camera_rig/state", self._on_state, qos_profile_sensor_data
        )

        self.step_size = float(step_size)
        self.step_step = float(step_step)
        self.min_step = float(min_step)
        self.max_step = float(max_step)
        self.rig_pose: list[float] | None = None
        self.flash_msg = ""
        self.flash_until = 0.0

        if init_pose is not None:
            self._publish_init_pose(init_pose)

    def _on_state(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.rig_pose = [float(msg.data[i]) for i in range(6)]

    def _publish_init_pose(self, pose: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in pose]
        self.pub_init_pose.publish(msg)
        self._flash(f"初始位姿 z={pose[2]:.2f}")

    def _publish_nudge(
        self, lateral: float, longitudinal: float, dyaw: float = 0.0
    ) -> None:
        msg = Float64MultiArray()
        msg.data = [float(lateral), float(longitudinal), float(dyaw)]
        self.pub_nudge.publish(msg)

    def _flash(self, text: str, sec: float = 2.0) -> None:
        self.flash_msg = text
        self.flash_until = time.monotonic() + sec

    def _pulse_record(self, cmd: str) -> None:
        msg = String()
        msg.data = cmd
        self.pub_record.publish(msg)
        self._flash(f"录制 {cmd}")

    def _nudge_move(self, lateral_sign: int = 0, longitudinal_sign: int = 0) -> None:
        s = self.step_size
        lateral = float(lateral_sign) * s
        longitudinal = float(longitudinal_sign) * s
        self._publish_nudge(lateral, longitudinal)
        if longitudinal_sign > 0:
            label = "前"
        elif longitudinal_sign < 0:
            label = "后"
        elif lateral_sign < 0:
            label = "左"
        else:
            label = "右"
        self._flash(f"{label} {s:.2f} m")

    def _nudge_yaw(self, yaw_sign: int) -> None:
        dyaw = float(yaw_sign) * YAW_STEP_DEG
        self._publish_nudge(0.0, 0.0, dyaw)
        self._flash(f"yaw {dyaw:+.0f}°")

    def _format_status(self) -> str:
        pose_s = ""
        if self.rig_pose is not None:
            p = self.rig_pose
            pose_s = f" | rig=({p[0]:.2f},{p[1]:.2f},{p[2]:.2f}, yaw={p[5]:.0f}°)"
        flash = ""
        if time.monotonic() < self.flash_until:
            flash = f" | {self.flash_msg}"
        return f"step={self.step_size:.2f} m{pose_s}{flash}"

    def _handle_key(self, key: str) -> bool:
        """返回 False 表示退出。"""
        if key in ("x", "X"):
            return False
        if key in ("a", "A"):
            self._nudge_move(lateral_sign=-1)
        elif key in ("d", "D"):
            self._nudge_move(lateral_sign=1)
        elif key in ("w", "W"):
            self._nudge_move(longitudinal_sign=1)
        elif key in ("s", "S"):
            self._nudge_move(longitudinal_sign=-1)
        elif key in ("z", "Z"):
            self._nudge_yaw(1)
        elif key in ("c", "C"):
            self._nudge_yaw(-1)
        elif key in ("q", "Q"):
            self.step_size = max(self.min_step, self.step_size - self.step_step)
            self._flash(f"步长 -> {self.step_size:.2f} m")
        elif key in ("e", "E"):
            self.step_size = min(self.max_step, self.step_size + self.step_step)
            self._flash(f"步长 -> {self.step_size:.2f} m")
        elif key in ("j", "J"):
            self._pulse_record("start")
        elif key in ("k", "K"):
            self._pulse_record("stop")
        return True

    def run(self, poll_hz: float) -> None:
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        period = 1.0 / poll_hz
        ui = TeleopDisplay()
        ui.enter()
        try:
            tty.setcbreak(fd)
            while rclpy.ok():
                t0 = time.monotonic()
                key = _read_key(timeout=period)
                if key and not self._handle_key(key):
                    break
                rclpy.spin_once(self, timeout_sec=0)
                ui.set_status(self._format_status())
                elapsed = time.monotonic() - t0
                sleep_t = period - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            ui.leave()


def _resolve_init_pose(args: argparse.Namespace) -> list[float] | None:
    fields = (
        ("init_x", 0),
        ("init_y", 1),
        ("init_z", 2),
        ("init_roll", 3),
        ("init_pitch", 4),
        ("init_yaw", 5),
    )
    if not any(getattr(args, name) is not None for name, _ in fields):
        return None
    pose = list(args.init_pose)
    for name, idx in fields:
        val = getattr(args, name)
        if val is not None:
            pose[idx] = float(val)
    return pose


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--poll-hz", type=float, default=30.0, help="键盘轮询频率 Hz")
    p.add_argument("--step-size", type=float, default=0.1, help="初始步长 (米)")
    p.add_argument("--step-step", type=float, default=0.05, help="q/e 每次调整步长 (米)")
    p.add_argument("--min-step", type=float, default=0.02, help="最小步长 (米)")
    p.add_argument("--max-step", type=float, default=2.0, help="最大步长 (米)")
    p.add_argument(
        "--init-pose",
        type=float,
        nargs=6,
        default=[0.0, 0.0, 1.5, 0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="默认初始位姿；可被 --init-x 等覆盖",
    )
    p.add_argument("--init-x", type=float, default=None)
    p.add_argument("--init-y", type=float, default=None)
    p.add_argument("--init-z", type=float, default=None)
    p.add_argument("--init-roll", type=float, default=None)
    p.add_argument("--init-pitch", type=float, default=None)
    p.add_argument("--init-yaw", type=float, default=None)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = KeyboardTeleop(
        step_size=args.step_size,
        step_step=args.step_step,
        min_step=args.min_step,
        max_step=args.max_step,
        init_pose=_resolve_init_pose(args),
    )
    try:
        node.run(poll_hz=args.poll_hz)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\n已退出。")


if __name__ == "__main__":
    main()
