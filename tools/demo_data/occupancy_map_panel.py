"""Isaac Sim 中显示 CameraRig 在 occupancy 横截面地图上的实时位置。"""

from __future__ import annotations

from typing import List, Optional

import numpy as np

from sdg_utils.occupancy import OccSlice2DMeta, OccSliceCache, render_occ_map_with_rig


class OccupancyMapPanel:
    """在 Isaac Sim 界面停靠 occupancy 地图，并随 rig 位姿更新标记。"""

    WINDOW_TITLE = "CameraRig - Occupancy Map"

    def __init__(self, slice_cache: OccSliceCache, init_pose: List[float]):
        import omni.ui as ui

        self._cache = slice_cache
        self._meta: OccSlice2DMeta | None = None
        self._pose = [float(v) for v in init_pose]
        self._recorded: List[List[float]] = []
        self._recording = False
        self._provider = ui.ByteImageProvider()
        self._frame: Optional[ui.Frame] = None

        self._window = ui.Window(
            self.WINDOW_TITLE,
            width=360,
            height=360,
            visible=True,
            dockPreference=ui.DockPreference.RIGHT_BOTTOM,
        )
        with self._window.frame:
            with ui.VStack(spacing=4):
                self._meta_label = ui.Label("", height=20)
                self._frame = ui.Frame()
                self._frame.set_build_fn(self._build_image_frame)
                self._status_label = ui.Label(self._status_text(), height=20)

        self.set_slice_z(init_pose[2])

    def _build_image_frame(self) -> None:
        import omni.ui as ui

        with ui.VStack():
            ui.ImageWithProvider(
                self._provider,
                fill_policy=ui.IwpFillPolicy.IWP_PRESERVE_ASPECT_FIT,
            )

    def _status_text(self) -> str:
        p = self._pose
        rec = f" | REC {len(self._recorded)} 点" if self._recording else ""
        return (
            f"rig=({p[0]:.2f}, {p[1]:.2f}) yaw={p[5]:.0f}°"
            f"{rec}  |  黑=占据 白=可通行  红=rig  绿=轨迹"
        )

    def set_recording(self, recording: bool) -> None:
        self._recording = recording
        if not recording:
            self._recorded = []
        self._refresh()

    def set_slice_z(self, slice_z: float) -> None:
        z_key = round(float(slice_z), 2)
        if self._meta is not None and abs(self._meta["slice_z"] - z_key) < 1e-4:
            return
        self._meta = self._cache.get_slice(z_key)
        self._meta_label.text = (
            f"z={self._meta['slice_z']:.2f} m  |  res={self._meta['resolution']:.2f} m/px"
        )

    def update_pose(
        self,
        pose: List[float],
        recorded: Optional[List[List[float]]] = None,
    ) -> None:
        self.set_slice_z(pose[2])
        self._pose = [float(v) for v in pose]
        if recorded is not None:
            self._recorded = [list(p) for p in recorded]
        self._refresh()

    def _refresh(self) -> None:
        if self._meta is None:
            return
        rgba = render_occ_map_with_rig(self._meta, self._pose, self._recorded or None)
        self._provider.set_bytes_data(list(rgba.tobytes()), [rgba.shape[1], rgba.shape[0]])
        if self._frame is not None:
            self._frame.rebuild()
        if hasattr(self, "_status_label") and self._status_label is not None:
            self._status_label.text = self._status_text()

    def destroy(self) -> None:
        if self._window is not None:
            self._window.destroy()
            self._window = None
