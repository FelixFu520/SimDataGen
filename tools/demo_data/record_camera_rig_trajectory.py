#!/usr/bin/env python3
"""在 Isaac Sim 中加载场景 + CameraRig，经 ROS2 接收键盘遥操并录制 rig 轨迹。

Usage:
    ./app/python.sh tools/demo_data/record_camera_rig_trajectory.py \\
        --scene_usd /home/fufa/projects2026/SimDataGen/asset_extern/home_000/interior_template.usdc \\
        --camera_usd /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H30YA.usd \\
        --output_dir workdir/demo_trajectory/home_000_manual

另开终端（系统 ROS2）:
    source /opt/ros/humble/setup.bash
    python tools/demo_data/keyboard_camera_rig_teleop.py

推荐用封装脚本（自动设置 ROS2 bridge 环境）:
    ./tools/demo_data/run_record_camera_rig_trajectory.sh ...
"""

import os
import sys

_ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT_DIR not in sys.path:
    sys.path.insert(0, _ROOT_DIR)

from tools.demo_data.ros2_bridge_env import print_setup_hint, setup_isaac_ros2_bridge_env

setup_isaac_ros2_bridge_env()

from isaacsim import SimulationApp

launch_config = {
    "headless": False,
    "renderer": "RaytracedLighting",
}
simulation_app = SimulationApp(launch_config=launch_config)

import argparse
import json
import time
from typing import List, Optional

import numpy as np
from loguru import logger

import carb
from isaacsim.core.utils.extensions import enable_extension

settings = carb.settings.get_settings()
settings.set("/rtx/verifyDriverVersion/enabled", False)

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.gen.omap")
simulation_app.update()

try:
    import rclpy
except ModuleNotFoundError:
    print_setup_hint()
    raise SystemExit(
        "rclpy 不可用：isaacsim.ros2.bridge 未成功加载。"
        "请先 export ROS_DISTRO / RMW_IMPLEMENTATION / LD_LIBRARY_PATH，"
        "或使用 run_record_camera_rig_trajectory.sh 启动。"
    ) from None
from std_msgs.msg import Float64MultiArray, String
from rclpy.node import Node

from isaacsim.core.api import World

ROOT_DIR = _ROOT_DIR

from pxr import UsdGeom

from isaacsim.asset.gen.omap.bindings import _omap  # noqa: F401 — 须与 enable_extension 配合

from sdg_utils.camera import CameraRig, _gf_matrix_to_np
from sdg_utils.occupancy import (
    OccSliceCache,
    get_mesh_paths,
    get_semantic_occupancy,
    save_semantic_occupancy_ply,
)
from sdg_utils.usd import load_usd_file
from sdg_utils.trajectory import save_path_ply
from tools.demo_data.occupancy_map_panel import OccupancyMapPanel

# Isaac 编辑器默认透视相机，勿用 CameraRig 上的 CAM_* 作为视口相机
VIEWPORT_OBSERVER_CAMERA = "/OmniverseKit_Persp"
PRIMARY_VIEWPORT_WINDOW = "Viewport"
# 6 路相机在主窗口内 2×3 停靠布局（与 demo 截图一致）
VIEWPORT_GRID_ROWS = (
    ("CAM_A", "CAM_Front", "CAM_B"),
    ("CAM_D", "CAM_Back", "CAM_C"),
)

logger.remove()
logger.add(sys.stdout, format="{message}", level="INFO", colorize=True)


class CameraRigTrajectoryRecorder(Node):
    def __init__(
        self,
        world: World,
        camera_rig: CameraRig,
        output_dir: str,
        init_pose: List[float],
        follow_viewport: bool = True,
        viewport_camera: str = "CAM_Front",
        viewport_cameras_extra: Optional[List[str]] = None,
        occ_map_panel: Optional[OccupancyMapPanel] = None,
    ):
        super().__init__("camera_rig_trajectory_recorder")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)

        self.world = world
        self.camera_rig = camera_rig
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)

        self.pose = [float(v) for v in init_pose]  # x,y,z, roll,pitch,yaw (deg)
        self.fixed_z = float(init_pose[2])
        self.recording = False
        self.recorded: List[List[float]] = []
        self.path_index = 0
        self.follow_viewport = bool(follow_viewport)
        self.viewport_camera = (viewport_camera or "CAM_A").strip()
        self.viewport_cameras_extra = list(viewport_cameras_extra or [])
        self.occ_map_panel = occ_map_panel

        self.create_subscription(Float64MultiArray, "/camera_rig/nudge", self._on_nudge, 10)
        self.create_subscription(String, "/camera_rig/record", self._on_record, 10)
        self.create_subscription(Float64MultiArray, "/camera_rig/init_pose", self._on_init_pose, 10)
        self.pub_state = self.create_publisher(Float64MultiArray, "/camera_rig/state", 10)

        self._apply_pose()
        self._publish_state()
        self._update_occ_map_panel()

    def _update_occ_map_panel(self) -> None:
        if self.occ_map_panel is None:
            return
        recorded = self.recorded if self.recording else []
        self.occ_map_panel.update_pose(self.pose, recorded=recorded)

    def _is_rig_sensor_camera(self, camera_path: str) -> bool:
        rig = self.camera_rig.rig_prim_path.rstrip("/")
        cam = str(camera_path or "")
        return cam == rig or cam.startswith(rig + "/")

    def _viewport_uses_rig_sensor(self) -> bool:
        return self.viewport_camera.lower() not in ("perspective", "persp", "omniversekit_persp")

    def _resolve_rig_camera_path(self, name: str) -> Optional[str]:
        for cam_name, prim_path in zip(
            self.camera_rig.cameras_name,
            self.camera_rig.cameras_prim_path,
        ):
            if cam_name == name:
                return str(prim_path)
        return None

    def _set_viewport_camera(self) -> str:
        """启动时将视口切到 rig 上的传感器（默认 CAM_A），显示与采图一致的鱼眼画面。"""
        if self._viewport_uses_rig_sensor():
            cam_path = self._resolve_rig_camera_path(self.viewport_camera)
            if cam_path is None:
                avail = ", ".join(self.camera_rig.cameras_name)
                logger.warning(
                    f"未找到相机 '{self.viewport_camera}'，可用: {avail}；回退 {VIEWPORT_OBSERVER_CAMERA}"
                )
                cam_path = VIEWPORT_OBSERVER_CAMERA
                self.viewport_camera = "perspective"
        else:
            cam_path = VIEWPORT_OBSERVER_CAMERA

        try:
            from omni.kit.viewport.utility import get_active_viewport

            self._set_viewport_camera_on_api(get_active_viewport(), cam_path)
        except Exception:
            pass
        return cam_path

    def _set_viewport_camera_on_api(self, vp, cam_path: str) -> None:
        if vp is None:
            return
        vp.camera_path = cam_path
        try:
            import omni.kit.commands

            omni.kit.commands.execute(
                "SetViewportCamera",
                camera_path=cam_path,
                viewport_api=vp,
            )
        except Exception:
            pass

    def _viewport_window_name(self, camera_name: str) -> str:
        if camera_name == "CAM_A":
            return PRIMARY_VIEWPORT_WINDOW
        return f"Viewport {camera_name}"

    def _create_secondary_viewport(
        self, camera_name: str, index: int = 0, *, docked: bool = False
    ) -> Optional[str]:
        """再开一个 Viewport 窗口并绑定 rig 上的相机。"""
        cam_path = self._resolve_rig_camera_path(camera_name)
        if cam_path is None:
            avail = ", ".join(self.camera_rig.cameras_name)
            logger.warning(
                f"额外视口：未找到相机 '{camera_name}'，可用: {avail}"
            )
            return None
        vp_w, vp_h = 520, 390
        cols = 3
        row, col = divmod(index, cols)
        try:
            from omni.kit.viewport.utility import create_viewport_window

            kwargs = {
                "name": self._viewport_window_name(camera_name),
                "width": vp_w,
                "height": vp_h,
                "camera_path": cam_path,
            }
            if not docked:
                kwargs["position_x"] = 40 + col * (vp_w + 16)
                kwargs["position_y"] = 40 + row * (vp_h + 40)
            window = create_viewport_window(**kwargs)
            if window is not None:
                self._set_viewport_camera_on_api(window.viewport_api, cam_path)
                return cam_path
        except Exception as exc:
            logger.warning(f"创建视口失败 ({camera_name}): {exc}")
        return None

    @staticmethod
    async def _dock_viewport_window(
        child_name: str, parent_name: str, position, ratio: float, wait_frames: int = 4
    ) -> None:
        import omni.kit.app
        from omni import ui

        app = omni.kit.app.get_app()
        for _ in range(30):
            child = ui.Workspace.get_window(child_name)
            parent = ui.Workspace.get_window(parent_name)
            if child is not None and parent is not None:
                child.deferred_dock_in(parent_name)
                for _ in range(wait_frames):
                    await app.next_update_async()
                child.dock_in(parent, position, ratio)
                await app.next_update_async()
                return
            await app.next_update_async()

    def _schedule_viewport_grid_docking(self) -> None:
        """将 5 个额外观口停靠成 2×3 网格（主视口 CAM_A 占左上）。"""
        try:
            import omni.kit.async_engine
            from omni import ui
        except ImportError:
            return

        async def _dock_all() -> None:
            # 上行：CAM_Front | CAM_B 依次向右拆分
            await self._dock_viewport_window(
                self._viewport_window_name("CAM_Front"),
                PRIMARY_VIEWPORT_WINDOW,
                ui.DockPosition.RIGHT,
                2.0 / 3.0,
            )
            await self._dock_viewport_window(
                self._viewport_window_name("CAM_B"),
                self._viewport_window_name("CAM_Front"),
                ui.DockPosition.RIGHT,
                0.5,
            )
            # 下行：CAM_D | CAM_Back | CAM_C 在对应列下方拆分
            await self._dock_viewport_window(
                self._viewport_window_name("CAM_D"),
                PRIMARY_VIEWPORT_WINDOW,
                ui.DockPosition.BOTTOM,
                0.5,
            )
            await self._dock_viewport_window(
                self._viewport_window_name("CAM_Back"),
                self._viewport_window_name("CAM_Front"),
                ui.DockPosition.BOTTOM,
                0.5,
            )
            await self._dock_viewport_window(
                self._viewport_window_name("CAM_C"),
                self._viewport_window_name("CAM_B"),
                ui.DockPosition.BOTTOM,
                0.5,
            )

        omni.kit.async_engine.run_coroutine(_dock_all())

    def _should_use_viewport_grid(self) -> bool:
        if self.viewport_camera != "CAM_A" or len(self.viewport_cameras_extra) < 5:
            return False
        grid_names = {name for row in VIEWPORT_GRID_ROWS for name in row}
        rig_names = set(self.camera_rig.cameras_name)
        return grid_names.issubset(rig_names)

    def _setup_viewports_grid(self) -> tuple[str, List[str]]:
        primary = self._set_viewport_camera()
        extra_paths: List[str] = []
        for row in VIEWPORT_GRID_ROWS:
            for camera_name in row:
                if camera_name == self.viewport_camera:
                    continue
                path = self._create_secondary_viewport(camera_name, docked=True)
                if path:
                    extra_paths.append(path)
                    simulation_app.update()
        self._schedule_viewport_grid_docking()
        for _ in range(12):
            simulation_app.update()
        return primary, extra_paths

    def _setup_viewports_legacy(self) -> tuple[str, List[str]]:
        primary = self._set_viewport_camera()
        extra_paths: List[str] = []
        seen = {self.viewport_camera.lower()}
        for i, name in enumerate(self.viewport_cameras_extra):
            key = name.strip()
            if not key or key.lower() in seen:
                continue
            seen.add(key.lower())
            path = self._create_secondary_viewport(key, index=i)
            if path:
                extra_paths.append(path)
                simulation_app.update()
        return primary, extra_paths

    def _setup_viewports(self) -> tuple[str, List[str]]:
        if self._should_use_viewport_grid():
            return self._setup_viewports_grid()
        return self._setup_viewports_legacy()

    @staticmethod
    def _normalize_xy(vec: np.ndarray) -> np.ndarray:
        xy = np.asarray(vec[:2], dtype=np.float64)
        n = float(np.linalg.norm(xy))
        if n < 1e-9:
            return np.array([1.0, 0.0], dtype=np.float64)
        return xy / n

    def _rig_body_axes_xy(self) -> tuple[np.ndarray, np.ndarray]:
        """CameraRig 当前世界位姿下的局部 X、Y 轴（水平面单位向量）。"""
        xform_cache = UsdGeom.XformCache()
        T = _gf_matrix_to_np(
            xform_cache.GetLocalToWorldTransform(self.camera_rig.rig_prim)
        )
        R = T[:3, :3]
        return self._normalize_xy(R[:, 0]), self._normalize_xy(R[:, 1])

    def _rig_teleop_directions_xy(self) -> tuple[np.ndarray, np.ndarray]:
        """rig 本体水平方向单位向量: (right, forward)。左=-right，后=-forward。"""
        axis_x, axis_y = self._rig_body_axes_xy()
        # CameraRig 工装：前向 ≈ 局部 +X，右向 ≈ 局部 -Y（与 USD 挂载一致）
        return -axis_y, axis_x

    def _teleop_delta_xy_to_world(self, lateral: float, longitudinal: float) -> tuple[float, float]:
        """lateral: 负=左、正=右；longitudinal: 正=前、负=后（米）。"""
        right_xy, forward_xy = self._rig_teleop_directions_xy()
        delta = lateral * right_xy + longitudinal * forward_xy
        return float(delta[0]), float(delta[1])

    def _on_nudge(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            return
        lateral = float(msg.data[0])
        longitudinal = float(msg.data[1])
        dyaw = float(msg.data[2]) if len(msg.data) >= 3 else 0.0
        dz = float(msg.data[3]) if len(msg.data) >= 4 else 0.0
        # 与键盘语义对齐：a=左 d=右 w=前 s=后 u=上升 i=下降
        dx, dy = self._teleop_delta_xy_to_world(lateral, longitudinal)
        self.pose[0] += dx
        self.pose[1] += dy
        self.pose[5] += dyaw
        self.fixed_z += dz
        self.pose[2] = self.fixed_z
        self._apply_pose()
        self._refresh_sim_view()
        self._append_pose_if_recording()
        self._print_move_log(dx, dy, dyaw, dz)
        self._publish_state()
        self._update_occ_map_panel()

    def _on_init_pose(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            logger.warning("init_pose 需要 6 个元素: x,y,z,roll,pitch,yaw")
            return
        self.pose = [float(msg.data[i]) for i in range(6)]
        self.fixed_z = float(self.pose[2])
        self._apply_pose()
        self._print_move_log(0.0, 0.0, 0.0, prefix="init_pose")
        self._publish_state()
        self._update_occ_map_panel()

    def _on_record(self, msg: String) -> None:
        cmd = (msg.data or "").strip().lower()
        if cmd == "start":
            self.recording = True
            self.recorded = [self.pose.copy()]
            if self.occ_map_panel is not None:
                self.occ_map_panel.set_recording(True)
            print(
                f"\n>>> 开始录制 轨迹 #{self.path_index:04d} "
                f"(j=录制中, a/d/w/s/u/i/z/c=步进/升降/转向, k=保存; 未按 j 前的移动不入轨)",
                flush=True,
            )
            self._print_pose_line(len(self.recorded) - 1, tag="REC")
        elif cmd == "stop":
            if self.recording:
                self._save_trajectory()
            else:
                print(">>> 未在录制中，无轨迹可保存（请先按 j）", flush=True)
            self.recording = False
            if self.occ_map_panel is not None:
                self.occ_map_panel.set_recording(False)

    def _apply_pose(self) -> None:
        self.pose[2] = self.fixed_z
        x, y, z, roll, pitch, yaw = self.pose
        self.camera_rig.set_pose(x, y, z, roll, pitch, yaw)
        if self.follow_viewport:
            self._follow_viewport_now()

    def _publish_state(self) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in self.pose]
        self.pub_state.publish(msg)

    def _follow_viewport_now(self) -> None:
        """perspective 模式：侧上方俯视 rig。CAM_A 等传感器模式：相机随 rig 运动，勿改外参。"""
        if not self.follow_viewport or self._viewport_uses_rig_sensor():
            return
        try:
            from isaacsim.core.utils.viewports import set_camera_view

            x, y, z = self.pose[0], self.pose[1], self.pose[2]
            eye = np.array([x - 2.5, y - 2.5, z + 1.8], dtype=np.float64)
            target = np.array([x, y, z], dtype=np.float64)
            set_camera_view(eye, target, camera_prim_path=VIEWPORT_OBSERVER_CAMERA)
        except Exception:
            pass

    def _refresh_sim_view(self, n_frames: int = 3) -> None:
        """每次键盘步进后强制刷新 Isaac 画面。"""
        for _ in range(n_frames):
            self.world.step(render=True)
            simulation_app.update()

    def _print_pose_line(self, index: int, tag: str = "") -> None:
        p = self.pose
        label = f"[{index:03d}]"
        extra = f" {tag}" if tag else ""
        print(
            f"  {label}{extra}  x={p[0]:.3f}  y={p[1]:.3f}  z={p[2]:.3f}  "
            f"roll={p[3]:.1f}  pitch={p[4]:.1f}  yaw={p[5]:.1f}",
            flush=True,
        )

    def _print_move_log(
        self,
        dx: float,
        dy: float,
        dyaw: float = 0.0,
        dz: float = 0.0,
        prefix: str = "",
    ) -> None:
        if self.recording:
            idx = len(self.recorded) - 1
            self._print_pose_line(idx, tag="REC")
            delta = f"Δ=({dx:+.3f}, {dy:+.3f}, yaw {dyaw:+.1f}°"
            if abs(dz) > 1e-6:
                delta += f", z {dz:+.3f}"
            delta += f")  共 {len(self.recorded)} 点"
            print(f"       {delta}", flush=True)
        else:
            p = self.pose
            head = f"{prefix} " if prefix else ""
            delta = f"Δ=({dx:+.3f}, {dy:+.3f})"
            if abs(dyaw) > 1e-6:
                delta = f"Δ=({dx:+.3f}, {dy:+.3f}, yaw {dyaw:+.1f}°)"
            if abs(dz) > 1e-6:
                delta = f"Δ=({dx:+.3f}, {dy:+.3f}, z {dz:+.3f})"
                if abs(dyaw) > 1e-6:
                    delta = f"Δ=({dx:+.3f}, {dy:+.3f}, yaw {dyaw:+.1f}°, z {dz:+.3f})"
            print(
                f"  {head}移动(未录制)  x={p[0]:.3f}  y={p[1]:.3f}  z={p[2]:.3f}  "
                f"yaw={p[5]:.1f}°  {delta}  [按 j 开始记入轨迹]",
                flush=True,
            )

    def _print_trajectory_table(self, title: str = "当前轨迹") -> None:
        if not self.recorded:
            print(f"--- {title}: (空) ---", flush=True)
            return
        print(f"--- {title} ({len(self.recorded)} 点) ---", flush=True)
        for i, p in enumerate(self.recorded):
            print(
                f"  [{i:03d}]  x={p[0]:.3f}  y={p[1]:.3f}  z={p[2]:.3f}  "
                f"roll={p[3]:.1f}  pitch={p[4]:.1f}  yaw={p[5]:.1f}",
                flush=True,
            )

    def _append_pose_if_recording(self) -> None:
        if not self.recording:
            return
        if self.recorded and np.allclose(self.recorded[-1], self.pose, atol=1e-5):
            return
        self.recorded.append(self.pose.copy())
        self._update_occ_map_panel()

    def _save_trajectory(self) -> None:
        if len(self.recorded) < 2:
            logger.warning("轨迹点不足 2 个，跳过保存")
            return

        poses = np.asarray(self.recorded, dtype=np.float64)
        xyz = poses[:, :3]
        paths = xyz[np.newaxis, :, :]  # (1, N, 3) 与 gen_data paths.npy 一致

        tag = f"{self.path_index:04d}"
        npy_path = os.path.join(self.output_dir, f"paths_{tag}.npy")
        rig_path = os.path.join(self.output_dir, f"rig_poses_{tag}.npy")
        np.save(npy_path, paths)
        np.save(rig_path, poses)

        ply_path = os.path.join(self.output_dir, f"trajectory_{tag}.ply")
        save_path_ply(xyz, free_xyz=None, ply_path=ply_path)

        meta = {
            "path_index": self.path_index,
            "num_points": int(len(poses)),
            "columns": ["x", "y", "z", "roll", "pitch", "yaw"],
            "fixed_z": self.fixed_z,
            "paths_npy": os.path.basename(npy_path),
            "rig_poses_npy": os.path.basename(rig_path),
            "ply": os.path.basename(ply_path),
        }
        meta_path = os.path.join(self.output_dir, f"trajectory_{tag}.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        print(f"\n>>> 已保存 {len(poses)} 点 -> {npy_path}", flush=True)
        self._print_trajectory_table("已保存轨迹")
        self.path_index += 1

    def run(self) -> None:
        self.world.reset()
        for _ in range(3):
            simulation_app.update()
        self.camera_rig.initialize(attach_depth=False, attach_semantic=False)
        for _ in range(3):
            self.world.step(render=True)
            simulation_app.update()
        vp_cam, vp_extra = self._setup_viewports()
        self._follow_viewport_now()
        self._refresh_sim_view(n_frames=8)
        if vp_extra:
            vp_info = f"{vp_cam} + {len(vp_extra)} 额外视口"
        else:
            vp_info = vp_cam
        print(
            f"场景已加载 | rig={self.camera_rig.rig_prim_path} | "
            f"初始=({self.pose[0]:.2f},{self.pose[1]:.2f},{self.pose[2]:.2f}) | "
            f"z={self.fixed_z:.2f} | 视口相机={vp_info}",
            flush=True,
        )
        print("等待键盘: j=开始录 k=保存 | a/d=左右 w/s=前后 | 轨迹打印见下方\n", flush=True)
        self._print_move_log(0.0, 0.0, prefix="初始")

        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            simulation_app.update()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scene_usd", required=True, help="场景 USD/USDC 路径")
    p.add_argument("--camera_usd", required=True, help="CameraRig 相机组 USD 路径")
    p.add_argument("--output_dir", required=True, help="轨迹输出目录")
    p.add_argument(
        "--init_pose",
        type=float,
        nargs=6,
        default=[0.0, 0.0, 1.5, 0.0, 0.0, 0.0],
        metavar=("X", "Y", "Z", "ROLL", "PITCH", "YAW"),
        help="rig 初始位姿 (米, 度)；可被下方单项参数覆盖",
    )
    p.add_argument("--init-x", type=float, default=None, help="rig 初始 X (米)，覆盖 --init_pose[0]")
    p.add_argument("--init-y", type=float, default=None, help="rig 初始 Y (米)，覆盖 --init_pose[1]")
    p.add_argument("--init-z", type=float, default=None, help="rig 初始 Z (米)，覆盖 --init_pose[2]")
    p.add_argument("--init-roll", type=float, default=None, help="rig 初始 roll (度)，覆盖 --init_pose[3]")
    p.add_argument("--init-pitch", type=float, default=None, help="rig 初始 pitch (度)，覆盖 --init_pose[4]")
    p.add_argument("--init-yaw", type=float, default=None, help="rig 初始 yaw (度)，覆盖 --init_pose[5]")
    p.add_argument("--rig_prim_path", default="/World/camera_rig", help="rig 挂载 prim 路径")
    p.add_argument(
        "--follow-viewport",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="perspective 模式下侧上方俯视 rig；CAM_A 模式下画面随 rig 自动更新",
    )
    p.add_argument(
        "--viewport-camera",
        default="CAM_A",
        help="主视口绑定的相机：rig 上的名称如 CAM_A（默认）、CAM_Front，或 perspective",
    )
    p.add_argument(
        "--viewport-cameras",
        nargs="*",
        default=None,
        metavar="NAME",
        help="额外 Viewport 窗口（可多个），如 CAM_B CAM_C CAM_D CAM_Front CAM_Back",
    )
    p.add_argument(
        "--viewport-camera-2",
        default=None,
        metavar="NAME",
        help="(兼容) 等同 --viewport-cameras 的单个相机",
    )
    p.add_argument(
        "--occupancy-resolution",
        type=float,
        default=0.1,
        help="occupancy 体素分辨率 (米)，用于生成 z 横截面地图",
    )
    p.add_argument(
        "--no-occupancy-map",
        action="store_true",
        help="不生成 occupancy 地图面板",
    )
    return p.parse_args()


def resolve_viewport_cameras_extra(args: argparse.Namespace) -> List[str]:
    names: List[str] = []
    if args.viewport_cameras:
        names.extend(str(n).strip() for n in args.viewport_cameras if str(n).strip())
    if args.viewport_camera_2:
        names.append(str(args.viewport_camera_2).strip())
    deduped: List[str] = []
    seen: set[str] = set()
    for name in names:
        key = name.lower()
        if key not in seen:
            seen.add(key)
            deduped.append(name)
    return deduped


def load_or_compute_occ_slice_cache(
    stage,
    output_dir: str,
    resolution: float,
) -> OccSliceCache:
    """加载或计算 3D occupancy，并按需生成/缓存各 z 横截面。"""
    occ_dir = os.path.join(output_dir, "occupancy")
    os.makedirs(occ_dir, exist_ok=True)
    occupied_npy = os.path.join(occ_dir, "occupied_positions.npy")
    free_npy = os.path.join(occ_dir, "free_positions.npy")

    if os.path.isfile(occupied_npy) and os.path.isfile(free_npy):
        logger.info(f"命中 occupancy 缓存: {occ_dir}")
        occupied_data = np.load(occupied_npy)
        free_data = np.load(free_npy)
    else:
        logger.info("计算 occupancy（首次较慢，结果会缓存到 output_dir/occupancy/）")
        mesh_paths = get_mesh_paths(stage)
        logger.info(f"物理碰撞 mesh 数量: {len(mesh_paths)}")
        semantic_occupancy = get_semantic_occupancy(
            stage,
            resolution=resolution,
            mesh_paths=mesh_paths,
        )
        occupied_data = semantic_occupancy[semantic_occupancy[:, 3] != 0]
        free_data = semantic_occupancy[semantic_occupancy[:, 3] == 0]
        np.save(occupied_npy, occupied_data)
        np.save(free_npy, free_data)
        save_semantic_occupancy_ply(
            occupied_data,
            os.path.join(occ_dir, "occupied_positions.ply"),
        )
        logger.info(
            f"occupancy 已缓存: occupied={occupied_data.shape[0]}, free={free_data.shape[0]}"
        )

    return OccSliceCache(occ_dir, occupied_data, free_data, resolution)


def resolve_init_pose(args: argparse.Namespace) -> list[float]:
    pose = [float(v) for v in args.init_pose]
    for name, idx in (
        ("init_x", 0),
        ("init_y", 1),
        ("init_z", 2),
        ("init_roll", 3),
        ("init_pitch", 4),
        ("init_yaw", 5),
    ):
        val = getattr(args, name)
        if val is not None:
            pose[idx] = float(val)
    return pose


def main() -> None:
    args = parse_args()
    scene_usd = os.path.abspath(args.scene_usd)
    camera_usd = os.path.abspath(args.camera_usd)
    output_dir = os.path.abspath(args.output_dir)

    if not os.path.isfile(scene_usd):
        logger.error(f"场景不存在: {scene_usd}")
        sys.exit(1)
    if not os.path.isfile(camera_usd):
        logger.error(f"相机 USD 不存在: {camera_usd}")
        sys.exit(1)

    world, stage = load_usd_file(scene_usd)
    init_pose = resolve_init_pose(args)

    occ_map_panel: Optional[OccupancyMapPanel] = None
    if not args.no_occupancy_map:
        world.reset()
        for _ in range(3):
            simulation_app.update()
        occ_slice_cache = load_or_compute_occ_slice_cache(
            stage,
            output_dir,
            resolution=args.occupancy_resolution,
        )
        occ_map_panel = OccupancyMapPanel(occ_slice_cache, init_pose)

    camera_rig = CameraRig(
        camera_usd_path=camera_usd,
        world=world,
        stage=stage,
        rig_prim_path=args.rig_prim_path,
    )

    rclpy.init()
    recorder = CameraRigTrajectoryRecorder(
        world=world,
        camera_rig=camera_rig,
        output_dir=output_dir,
        init_pose=init_pose,
        follow_viewport=args.follow_viewport,
        viewport_camera=args.viewport_camera,
        viewport_cameras_extra=resolve_viewport_cameras_extra(args),
        occ_map_panel=occ_map_panel,
    )
    try:
        recorder.run()
    finally:
        if occ_map_panel is not None:
            occ_map_panel.destroy()
        recorder.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
