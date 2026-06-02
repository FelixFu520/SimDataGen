from typing import List, Any, Optional, Dict
import os
import re
import numpy as np
from PIL import Image
from loguru import logger

from pxr import Gf, Sdf, UsdGeom, Usd
from isaacsim.core.api import World
from isaacsim.sensors.camera import Camera as IsaacCamera

from .misc import generate_high_contrast_colors
from .projection_lut import (
    read_lut_enter_exr as _read_lut_enter_exr,
    mask_from_lut_enter_exr as _mask_from_lut_enter_exr,
    depth_to_pointcloud_lut,
    depth_to_pointcloud_mei,
)


def _ensure_square_pixel_aperture(prim: Usd.Prim, width: int, height: int) -> None:
    """将 verticalAperture 设为与分辨率宽高比一致。

    IsaacCamera 初始化时若宽高比不一致会静默改写 verticalAperture(日志里
    'Setting verticalAperture to ... to ensure square pixels'),导致
    generalizedProjection LUT 鱼眼的 distance_to_image_plane 与标定几何错位。
    """
    if width <= 0 or height <= 0:
        return
    cam_geom = UsdGeom.Camera(prim)
    h_ap = float(cam_geom.GetHorizontalApertureAttr().Get() or 0.0)
    if h_ap <= 0:
        return
    target_v = h_ap * float(height) / float(width)
    v_attr = cam_geom.GetVerticalApertureAttr()
    old_v = float(v_attr.Get() or 0.0)
    if abs(old_v - target_v) > 1e-5:
        v_attr.Set(target_v)
        logger.info(
            f"相机 '{prim.GetName()}' verticalAperture: {old_v:.6f} -> {target_v:.6f} "
            f"(匹配分辨率 {width}x{height})"
        )


# OpenCV坐标系到Isaac Sim(RTX)相机坐标系的转换矩阵
# OpenCV: X右, Y下, Z前 (相机朝向+Z)
# Isaac Sim RTX 相机本地坐标: X右, Y上, Z后 (相机朝向-Z)
# 转换关系: 翻转Y和Z轴
R_cv_to_isaac = np.eye(4)
R_cv_to_isaac[1, 1] = -1
R_cv_to_isaac[2, 2] = -1


def _read_attr(prim: Usd.Prim, name: str, default=None):
    a = prim.GetAttribute(name)
    return a.Get() if a and a.HasAuthoredValue() else default


def _read_resolution_from_prim(prim: Usd.Prim) -> Optional[tuple]:
    """从相机 prim 读取 `omni:calibration:resolution` (int2)。"""
    res = _read_attr(prim, "omni:calibration:resolution")
    if res is None:
        return None
    return (int(res[0]), int(res[1]))


def _read_calibration_from_prim(prim: Usd.Prim) -> Optional[Dict[str, Any]]:
    """从相机 prim 上读取 bake 进 USD 的 omni 标定 custom attribute。

    需要 prim 上存在 `omni:calibration:cameraModel` 等属性
    (由 tools/bake_camera_intrinsics.py 写入)。仅对 omni / fisheye 等带 LUT
    的鱼眼相机有效;针孔相机通常没有这个标定块。
    """
    model_attr = prim.GetAttribute("omni:calibration:cameraModel")
    if not model_attr or not model_attr.HasAuthoredValue():
        return None

    dist = _read_attr(prim, "omni:calibration:distortionCoeffs")
    res = _read_resolution_from_prim(prim)
    # mask_radius: LUT 鱼眼像点圆半径(像素),由 tools/bake_camera_intrinsics.py
    # 通过 --mask_radius NAME=R 写入;缺省时为 None,运行时回退到内切圆估计。
    mask_radius = _read_attr(prim, "omni:calibration:maskRadius")
    mask_center_mode = _read_attr(prim, "omni:calibration:maskCenterMode")
    return {
        "model": str(model_attr.Get()),
        "xi": float(_read_attr(prim, "omni:calibration:xi", 0.0)),
        "fx": float(_read_attr(prim, "omni:calibration:fx", 0.0)),
        "fy": float(_read_attr(prim, "omni:calibration:fy", 0.0)),
        "cx": float(_read_attr(prim, "omni:calibration:cx", 0.0)),
        "cy": float(_read_attr(prim, "omni:calibration:cy", 0.0)),
        "distortion_model": str(_read_attr(prim, "omni:calibration:distortionModel", "radtan")),
        "distortion_coeffs": [float(v) for v in dist] if dist is not None else [],
        "width": res[0] if res is not None else None,
        "height": res[1] if res is not None else None,
        "mask_radius": float(mask_radius) if mask_radius is not None else None,
        # "calibration":圆心用 Kalibr cx/cy(原版/内参扰动版工装,与 RTX 有效区对齐)
        # "imageCenter":圆心用 (w/2,h/2),仅当实测 FOV 居中时使用
        "mask_center_mode": str(mask_center_mode)
        if mask_center_mode is not None
        else "calibration",
    }


def _read_ftheta_from_prim(prim: Usd.Prim) -> Optional[Dict[str, Any]]:
    """读取 NVIDIA F-Theta 鱼眼标定(USD 内置 schema, Isaac Sim/Drive 使用)。

    若 prim 上存在 `fthetaWidth` 与 `fthetaHeight`,认为是 F-Theta 标定相机。
    多项式系数 polyA..F:前 3 个 (A,B,C) 是像素半径->角度的正向映射,
    后 3 个 (D,E,F) 是角度->像素半径的反向映射。
    """
    width = _read_attr(prim, "fthetaWidth")
    height = _read_attr(prim, "fthetaHeight")
    if width is None or height is None:
        return None

    def _f(name):
        v = _read_attr(prim, name)
        return float(v) if v is not None else None

    return {
        "model": "ftheta",
        "width": int(width),
        "height": int(height),
        "cx": _f("fthetaCx"),
        "cy": _f("fthetaCy"),
        "max_fov_deg": _f("fthetaMaxFov"),
        "poly": {
            "A": _f("fthetaPolyA"),
            "B": _f("fthetaPolyB"),
            "C": _f("fthetaPolyC"),
            "D": _f("fthetaPolyD"),
            "E": _f("fthetaPolyE"),
            "F": _f("fthetaPolyF"),
        },
        "front_face_resolution_scale": _f("fisheyeFrontFaceResolutionScale"),
        "resolution_budget": _f("fisheyeResolutionBudget"),
    }


def _gf_matrix_to_np(mat: Gf.Matrix4d) -> np.ndarray:
    """USD 行优先 Matrix4d -> numpy 4x4。USD 中点乘约定为 row-vector * M,
    转成数学上常用的 col-major (M_math = M_usd.T),方便提取 translation/rotation。"""
    arr = np.array([[mat[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)
    return arr.T  # 转成标准 col-major


class CameraRig:
    """相机组类:加载相机 USD,管理 USD 相机 prim 与对应的 IsaacCamera 实例。"""

    def __init__(
        self,
        camera_usd_path: str,
        world: World,
        stage: Usd.Stage,
        rig_prim_path: str = "/World/camera_rig",
    ):
        """
        Args:
            camera_usd_path: 相机组 usd 路径, 如 "assets/cameras/oak_camera_4lut.usd"
            world: Isaac Sim World 对象
            stage: USD Stage 对象
            rig_prim_path: 相机组挂载到舞台上的根 prim 路径

        相机的真实 omni 内参必须事先用 tools/bake_camera_intrinsics.py
        bake 到 USD 文件中(作为 `omni:calibration:*` custom attribute)。
        """
        self.camera_usd_path = os.path.abspath(camera_usd_path)
        self.world: World = world
        self.stage: Usd.Stage = stage
        self.rig_prim_path = rig_prim_path

        # rig 根 prim
        self.rig_prim: Usd.Prim = stage.DefinePrim(self.rig_prim_path, "Xform")

        self.cameras_name: List[str] = []
        self.cameras_prim_path: List[str] = []
        self.cameras_prim: List[Usd.Prim] = []
        self.isaac_cameras: List[IsaacCamera] = []
        # Mei omni 内参(来自 omni:calibration:* custom attribute),按相机名索引
        self.calibration: Dict[str, Dict[str, Any]] = {}
        # NVIDIA F-Theta 鱼眼标定(来自 ftheta* 属性),按相机名索引
        self.ftheta_calibration: Dict[str, Dict[str, Any]] = {}
        # 每个相机的分辨率 (width, height),按相机名索引
        self.resolutions: Dict[str, tuple] = {}
        # LUT 鱼眼:由 rayEnterDirection.exr 生成的 bitmap mask(优先于圆形 mask)
        self._lut_mask_bitmaps: Dict[str, np.ndarray] = {}
        # LUT enter 方向场 (H,W,3) float32,与 RTX 渲染射线一致
        self._lut_enter_directions: Dict[str, np.ndarray] = {}

        # ----- 语义分割相关状态(可选) -----
        # _mesh_path_to_semantic_id: 由 bind_semantic_id_map() 注入的 mesh_path -> semantic_id 映射
        # _semantic_color_lut: 可视化用颜色 LUT(index = semantic_id, value = RGB)
        # _instance_annotator_name: 实际成功 attach 上的 instance_id annotator 名字
        # 注意: 每帧重新构建 remap LUT, 不做跨帧缓存(避免 annot_id 漂移导致同 mesh 被分到不同 sem_id)
        self._mesh_path_to_semantic_id: Dict[str, int] = {}
        self._semantic_color_lut: Optional[np.ndarray] = None
        self._instance_annotator_name: Optional[str] = None

        self._load_usd()
        self._collect_cameras()
        self._build_isaac_cameras()
        logger.info(
            f"CameraRig 初始化完成, rig={self.rig_prim_path}, "
            f"相机数={len(self.cameras_name)}, 相机={self.cameras_name}"
        )

    # ------------------------------------------------------------------
    # 加载与构造
    # ------------------------------------------------------------------
    def _load_usd(self) -> None:
        """将相机组 USD 作为 reference 加载到 rig prim 下。"""
        if not os.path.exists(self.camera_usd_path):
            raise FileNotFoundError(f"camera usd not found: {self.camera_usd_path}")

        refs = self.rig_prim.GetReferences()
        refs.ClearReferences()
        refs.AddReference(self.camera_usd_path)
        logger.info(f"已 reference 相机组 USD: {self.camera_usd_path} -> {self.rig_prim_path}")

    def _collect_cameras(self) -> None:
        """递归遍历 rig 下所有 UsdGeom.Camera prim,并读取每个相机的 omni 内参。"""
        self.base_link_prim: Optional[Usd.Prim] = None
        for prim in Usd.PrimRange(self.rig_prim):
            if prim.GetName() == "base_link" and self.base_link_prim is None:
                self.base_link_prim = prim
            if prim.IsA(UsdGeom.Camera):
                self.cameras_prim.append(prim)
                self.cameras_prim_path.append(str(prim.GetPath()))
                self.cameras_name.append(prim.GetName())

        # 按相机名排序
        order = np.argsort(self.cameras_name)
        self.cameras_prim = [self.cameras_prim[i] for i in order]
        self.cameras_prim_path = [self.cameras_prim_path[i] for i in order]
        self.cameras_name = [self.cameras_name[i] for i in order]

        # 读取每个相机的标定 (omni / ftheta,可选) 以及分辨率
        # 分辨率优先级:
        #   1. omni:calibration:resolution (来自 bake)
        #   2. omni 标定块里的 width/height
        #   3. ftheta 标定块里的 width/height
        for name, prim in zip(self.cameras_name, self.cameras_prim):
            omni_cal = _read_calibration_from_prim(prim)
            if omni_cal is not None:
                self.calibration[name] = omni_cal

            ftheta_cal = _read_ftheta_from_prim(prim)
            if ftheta_cal is not None:
                self.ftheta_calibration[name] = ftheta_cal

            res = _read_resolution_from_prim(prim)
            if res is None and omni_cal is not None and omni_cal["width"] and omni_cal["height"]:
                res = (omni_cal["width"], omni_cal["height"])
            if res is None and ftheta_cal is not None:
                res = (ftheta_cal["width"], ftheta_cal["height"])
            if res is not None:
                self.resolutions[name] = res

            if omni_cal is not None:
                self._try_load_lut_mask_bitmap(name, prim)

    def _resolve_lut_texture_path(self, raw_path: str) -> str:
        if os.path.isabs(raw_path):
            return raw_path
        return os.path.normpath(
            os.path.join(os.path.dirname(self.camera_usd_path), raw_path)
        )

    def _try_load_lut_mask_bitmap(self, name: str, prim: Usd.Prim) -> None:
        attr = prim.GetAttribute("generalizedProjectionDirectionTexturePath")
        if not attr or not attr.HasAuthoredValue():
            return
        raw = str(attr.Get().resolvedPath or attr.Get().path)
        if not raw:
            return
        exr_path = self._resolve_lut_texture_path(raw)
        if not os.path.isfile(exr_path):
            logger.warning(f"相机 '{name}' LUT enter 纹理不存在: {exr_path},回退圆形 mask")
            return
        try:
            data = _read_lut_enter_exr(exr_path)
            self._lut_enter_directions[name] = data
            self._lut_mask_bitmaps[name] = _mask_from_lut_enter_exr(data)
            logger.info(
                f"相机 '{name}' 已从 LUT 生成 bitmap mask: {exr_path} "
                f"(valid={int(self._lut_mask_bitmaps[name].sum() / 255)} px)"
            )
        except Exception as e:
            logger.warning(f"相机 '{name}' 读取 LUT mask 失败 ({exr_path}): {e},回退圆形 mask")

    def _build_isaac_cameras(self) -> None:
        """为每个 USD 相机 prim 构造一个 IsaacCamera(包装器,便于后续渲染/取图)。

        分辨率获取顺序:
          1. USD prim 上 `omni:calibration:resolution` 属性
          2. 默认 1920x1200
        如果想给某个相机自定义分辨率,请在 USD 上 bake `omni:calibration:resolution`。
        """
        self.isaac_cameras = []
        for name, prim_path in zip(self.cameras_name, self.cameras_prim_path):
            resolution = self.resolutions.get(name)
            if resolution is None:
                resolution = (1920, 1200)
                logger.warning(
                    f"相机 '{name}' 未在 USD 上设置 omni:calibration:resolution,"
                    f"使用默认分辨率 {resolution}"
                )
                self.resolutions[name] = resolution

            prim = self.stage.GetPrimAtPath(prim_path)
            if prim.IsValid():
                _ensure_square_pixel_aperture(prim, resolution[0], resolution[1])

            cam = IsaacCamera(
                prim_path=prim_path,
                name=f"isaac_{name}",
                resolution=resolution,
                frequency=30,
            )
            self.isaac_cameras.append(cam)

    def initialize(self, attach_depth: bool = True, attach_semantic: bool = False) -> None:
        """在 world.reset() 之后调用,初始化所有 IsaacCamera。

        Args:
            attach_depth: 是否同时 attach `distance_to_image_plane` annotator
                          (需要采深度时打开;只取 RGB 可以关掉省内存)。
            attach_semantic: 是否同时 attach `semantic_segmentation` 与
                             `instance_id_segmentation` annotator
                             (需要采语义图时打开)。
        """
        for cam in self.isaac_cameras:
            cam.initialize()
            if attach_depth:
                cam.attach_annotator("distance_to_image_plane")
            if attach_semantic:
                # semantic_segmentation: 直接返回我们写入的 id_{N} label
                cam.attach_annotator("semantic_segmentation", colorize=False)
                # instance_id_segmentation: 输出 {annot_id -> prim_path}, prim_path 帧间稳定。
                # 不同 Isaac Sim / Replicator 版本名字不同, 按优先级依次尝试。
                for ann_name in (
                    "instance_id_segmentation_fast",
                    "instance_id_segmentation",
                    "instance_segmentation_fast",
                    "instance_segmentation",
                ):
                    try:
                        cam.attach_annotator(ann_name)
                        self._instance_annotator_name = ann_name
                        break
                    except Exception as e:
                        logger.debug(f"attach_annotator({ann_name}) 失败: {e}")

    # ------------------------------------------------------------------
    # 内外参提取
    # ------------------------------------------------------------------
    def get_intrinsics(self, name: str) -> Dict[str, Any]:
        """读取一个相机的内参(USD 上的 + 标定 yaml 中的 omni 真实内参)。"""
        idx = self.cameras_name.index(name)
        prim = self.cameras_prim[idx]
        cam_geom = UsdGeom.Camera(prim)

        focal_length = float(cam_geom.GetFocalLengthAttr().Get())  # mm(USD 缺省单位)
        h_ap = float(cam_geom.GetHorizontalApertureAttr().Get())
        v_ap = float(cam_geom.GetVerticalApertureAttr().Get())
        proj_type_attr = prim.GetAttribute("cameraProjectionType")
        proj_type = str(proj_type_attr.Get()) if proj_type_attr.HasAuthoredValue() else "pinhole"
        clip = cam_geom.GetClippingRangeAttr().Get()

        info: Dict[str, Any] = {
            "name": name,
            "prim_path": self.cameras_prim_path[idx],
            "projection_type": proj_type,
            "focal_length_mm": focal_length,
            "horizontal_aperture_mm": h_ap,
            "vertical_aperture_mm": v_ap,
            "clipping_range": (float(clip[0]), float(clip[1])) if clip is not None else None,
        }

        # LUT 纹理路径
        for attr_name in (
            "generalizedProjectionDirectionTexturePath",
            "generalizedProjectionNDCTexturePath",
        ):
            attr = prim.GetAttribute(attr_name)
            if attr and attr.HasAuthoredValue():
                info[attr_name] = str(attr.Get().resolvedPath or attr.Get().path)

        # 分辨率
        resolution = self.resolutions.get(name)
        if resolution is not None:
            width, height = resolution
            info["resolution"] = (width, height)
            # USD pinhole 等效像素内参
            #   - 对于真实 pinhole 相机,这就是真实内参
            #   - 对于使用 LUT 的 fisheye/omni 相机,这只是 USD 几何参数推出的等效值,
            #     与 LUT 烤进去的真实标定不同(真实标定见 info["calibration"])
            info["fx_px_from_usd"] = focal_length / h_ap * width if h_ap > 0 else None
            info["fy_px_from_usd"] = focal_length / v_ap * height if v_ap > 0 else None
            info["cx_px_from_usd"] = width / 2.0
            info["cy_px_from_usd"] = height / 2.0

        # 真实 omni 内参(仅鱼眼相机有 bake)
        cal = self.calibration.get(name)
        if cal is not None and cal["width"] and cal["height"]:
            info["calibration"] = {
                "model": cal["model"],
                "xi": cal["xi"],
                "fx_px": cal["fx"],
                "fy_px": cal["fy"],
                "cx_px": cal["cx"],
                "cy_px": cal["cy"],
                "distortion_model": cal["distortion_model"],
                "distortion_coeffs": cal["distortion_coeffs"],
                "resolution": (cal["width"], cal["height"]),
            }

        # F-Theta 鱼眼标定(仅 NVIDIA ftheta 相机有)
        ftheta = self.ftheta_calibration.get(name)
        if ftheta is not None:
            info["ftheta"] = ftheta

        return info

    def get_extrinsics(self, name: str, time: Usd.TimeCode = Usd.TimeCode.Default()) -> Dict[str, np.ndarray]:
        """读取一个相机的外参,所有 4x4 矩阵的相机坐标系都是 RTX 约定 (X右, Y上, Z后)。

        - T_rig_cam     : 相对于挂载点 (`rig_prim_path`) 的变换
        - T_baselink_cam: 相对于工装 IMU/base_link 的变换 (若 USD 中存在 base_link)
        - T_world_cam   : 相对于世界坐标系的变换
        - T_world_cam_cv: world->camera 但相机坐标系是 OpenCV 约定 (X右, Y下, Z前)
        """
        idx = self.cameras_name.index(name)
        prim = self.cameras_prim[idx]

        xform_cache = UsdGeom.XformCache(time)
        T_world_cam = _gf_matrix_to_np(xform_cache.GetLocalToWorldTransform(prim))
        T_world_rig = _gf_matrix_to_np(xform_cache.GetLocalToWorldTransform(self.rig_prim))
        T_rig_cam = np.linalg.inv(T_world_rig) @ T_world_cam

        result: Dict[str, np.ndarray] = {
            "T_rig_cam": T_rig_cam,
            "T_world_cam": T_world_cam,
            "T_world_cam_cv": T_world_cam @ R_cv_to_isaac,
        }

        if self.base_link_prim is not None:
            T_world_baselink = _gf_matrix_to_np(
                xform_cache.GetLocalToWorldTransform(self.base_link_prim)
            )
            result["T_baselink_cam"] = np.linalg.inv(T_world_baselink) @ T_world_cam

        return result

    # ------------------------------------------------------------------
    # 运行时位姿控制
    # ------------------------------------------------------------------
    def set_pose(self, x: float, y: float, z: float,
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0) -> None:
        """设置整个相机组挂载点的世界位姿(影响所有相机)。
        roll/pitch/yaw 单位为度,按 XYZ 顺序应用。
        """
        rig_xform = UsdGeom.Xformable(self.rig_prim)
        rig_xform.ClearXformOpOrder()
        translate_op = rig_xform.AddTranslateOp()
        translate_op.Set(Gf.Vec3f(float(x), float(y), float(z)))
        rotate_op = rig_xform.AddRotateXYZOp()
        rotate_op.Set(Gf.Vec3f(float(roll), float(pitch), float(yaw)))

    # ------------------------------------------------------------------
    # 运行时数据获取
    # ------------------------------------------------------------------
    def get_cameras_name(self) -> List[str]:
        return list(self.cameras_name)

    def get_cameras_rgb(self) -> List[Optional[np.ndarray]]:
        """对每个相机调用 IsaacCamera.get_rgb(),返回 (H,W,3) uint8 数组列表。"""
        return [cam.get_rgb() for cam in self.isaac_cameras]

    def get_cameras_distance_to_image_plane(self) -> List[Optional[np.ndarray]]:
        """对每个相机调用 IsaacCamera.get_depth(),返回 (H,W) float32 列表。"""
        return [cam.get_depth() for cam in self.isaac_cameras]

    def depth_to_pointcloud_opencv(
        self,
        name: str,
        rgb: np.ndarray,
        depth: np.ndarray,
        mask: Optional[np.ndarray] = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """将 RGB+depth 反投影为 OpenCV 相机坐标系下的 3D 点。

        - 有 LUT enter 纹理:用与 RTX 一致的射线方向 + distance_to_image_plane
        - 仅有 omni 标定:回退 Mei 解析模型
        - 针孔:用 K 反投影
        """
        width, height = self.resolutions[name]
        if mask is None:
            mask = self._build_mask(name, width, height)

        lut = self._lut_enter_directions.get(name)
        if lut is not None:
            return depth_to_pointcloud_lut(rgb, depth, mask, lut)

        omni_cal = self.calibration.get(name)
        if omni_cal is not None:
            return depth_to_pointcloud_mei(
                rgb, depth, mask,
                omni_cal["fx"], omni_cal["fy"], omni_cal["cx"], omni_cal["cy"],
                omni_cal["xi"], omni_cal["distortion_coeffs"],
            )

        K = self.get_intrinsics_matrix(name)
        valid = (mask > 0) & np.isfinite(depth) & (depth > 0.01) & (depth < 100.0)
        v_idx, u_idx = np.where(valid)
        if u_idx.size == 0:
            return np.empty((0, 3), dtype=np.float64), np.empty((0, 3), dtype=np.uint8)
        u = u_idx.astype(np.float64)
        v = v_idx.astype(np.float64)
        d = depth[v_idx, u_idx].astype(np.float64)
        pix = np.stack([u, v, np.ones_like(u)], axis=-1)
        rays = (np.linalg.inv(K) @ pix.T).T
        pts = rays * d[:, None]
        return pts, rgb[v_idx, u_idx]

    def get_intrinsics_matrix(self, name: str) -> np.ndarray:
        """返回相机的 3x3 像素内参矩阵 K(与 Isaac Sim 实际渲染像素几何一致)。

        - LUT 鱼眼相机(有 omni:calibration:*):fx/fy/cx/cy 全部来自 USD 上 bake 的真实标定
          (此时 RTX 用 LUT 纹理渲染,主点 = cx/cy 与渲染一致)
        - 其他 pinhole 相机(含 F-Theta 但渲染端 cameraProjectionType=pinhole 的):
          fx = focal_length / horizontalAperture * width
          fy = focal_length / verticalAperture  * height
          cx = width / 2 + horizontalApertureOffset / horizontalAperture * width
          cy = height / 2 + verticalApertureOffset  / verticalAperture  * height
          注意:fthetaCx/Cy **不应**作为这里的主点 — 它们是 ftheta 模型自己的主点,
          而 RTX 在 cameraProjectionType="pinhole" 下渲染时主点完全由 aperture/offset 控制。
        """
        idx = self.cameras_name.index(name)
        prim = self.cameras_prim[idx]
        cam_geom = UsdGeom.Camera(prim)
        focal_length = float(cam_geom.GetFocalLengthAttr().Get())
        h_ap = float(cam_geom.GetHorizontalApertureAttr().Get())
        v_ap = float(cam_geom.GetVerticalApertureAttr().Get())
        h_off = float(cam_geom.GetHorizontalApertureOffsetAttr().Get() or 0.0)
        v_off = float(cam_geom.GetVerticalApertureOffsetAttr().Get() or 0.0)
        width, height = self.resolutions[name]

        omni_cal = self.calibration.get(name)
        if omni_cal is not None:
            fx, fy = omni_cal["fx"], omni_cal["fy"]
            cx, cy = omni_cal["cx"], omni_cal["cy"]
        else:
            fx = focal_length / h_ap * width if h_ap > 0 else 0.0
            fy = focal_length / v_ap * height if v_ap > 0 else 0.0
            cx = width / 2.0 + (h_off / h_ap * width if h_ap > 0 else 0.0)
            cy = height / 2.0 + (v_off / v_ap * height if v_ap > 0 else 0.0)

        return np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def get_camera_to_world_opencv(self, name: str,
                                   time: Usd.TimeCode = Usd.TimeCode.Default()) -> np.ndarray:
        """返回 OpenCV 相机坐标系 -> 世界坐标系的 4x4 变换矩阵。

        用法: P_world_homo = T @ P_opencv_homo
        """
        idx = self.cameras_name.index(name)
        xform_cache = UsdGeom.XformCache(time)
        T_world_cam_isaac = _gf_matrix_to_np(
            xform_cache.GetLocalToWorldTransform(self.cameras_prim[idx])
        )
        # P_world = T_world_cam_isaac @ R_cv_to_isaac @ P_opencv
        return T_world_cam_isaac @ R_cv_to_isaac

    def get_transform_between_cameras_opencv(
        self, name_from: str, name_to: str,
        time: Usd.TimeCode = Usd.TimeCode.Default(),
    ) -> np.ndarray:
        """返回 OpenCV 坐标系下相机 `name_from` -> 相机 `name_to` 的 4x4 变换矩阵。

        用法: P_to_homo = T @ P_from_homo
        即把 `name_from` 相机坐标系中的点变换到 `name_to` 相机坐标系。

        推导:
            T_world_from_cv = T_world_from_isaac @ R_cv_to_isaac
            T_world_to_cv   = T_world_to_isaac   @ R_cv_to_isaac
            T_to_from_cv    = inv(T_world_to_cv) @ T_world_from_cv
        """
        T_world_from_cv = self.get_camera_to_world_opencv(name_from, time)
        T_world_to_cv = self.get_camera_to_world_opencv(name_to, time)
        return np.linalg.inv(T_world_to_cv) @ T_world_from_cv

    # ------------------------------------------------------------------
    # 落盘
    # ------------------------------------------------------------------
    def _resolve_frame_id(
        self,
        frame_id: Optional[str] = None,
        path_idx: Optional[int] = None,
        point_idx: Optional[int] = None,
    ) -> str:
        """统一处理两种命名约定:
            1. 直接传 `frame_id`(优先级最高)
            2. 传 `path_idx`+`point_idx` -> "{path_idx:04d}_{point_idx:04d}"
        都没传则默认 "0000"。
        """
        if frame_id is not None:
            return frame_id
        if path_idx is not None and point_idx is not None:
            return f"{int(path_idx):04d}_{int(point_idx):04d}"
        return "0000"

    def save_cameras_rgb(
        self,
        output_dir: str,
        frame_id: Optional[str] = None,
        path_idx: Optional[int] = None,
        point_idx: Optional[int] = None,
    ) -> None:
        """每个相机一张 jpg: <output_dir>/<camera_name>/<frame_id>.jpg

        - 也接受 `path_idx`+`point_idx`, 自动拼成 "PPPP_PPPP" 的 frame_id (向后兼容旧命名)。
        """
        fid = self._resolve_frame_id(frame_id, path_idx, point_idx)
        for name, rgb in zip(self.cameras_name, self.get_cameras_rgb()):
            if rgb is None:
                logger.warning(f"save_cameras_rgb: {name} rgb is None, skip")
                continue
            out_path = os.path.join(output_dir, name, f"{fid}.jpg")
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            Image.fromarray(rgb).save(out_path, quality=95)

    def save_cameras_depth(
        self,
        output_dir: str,
        frame_id: Optional[str] = None,
        path_idx: Optional[int] = None,
        point_idx: Optional[int] = None,
    ) -> None:
        """每个相机 npy + 归一化 png:
          <output_dir>/<camera_name>/<frame_id>.npy
          <output_dir>/<camera_name>/<frame_id>.png
        """
        fid = self._resolve_frame_id(frame_id, path_idx, point_idx)
        for name, depth in zip(self.cameras_name, self.get_cameras_distance_to_image_plane()):
            if depth is None:
                logger.warning(f"save_cameras_depth: {name} depth is None, skip")
                continue
            cam_dir = os.path.join(output_dir, name)
            os.makedirs(cam_dir, exist_ok=True)
            npy_path = os.path.join(cam_dir, f"{fid}.npy")
            np.save(npy_path, depth)
            # 可视化用 png
            finite = np.isfinite(depth)
            if finite.any():
                dmin, dmax = depth[finite].min(), depth[finite].max()
                norm = np.where(finite, (depth - dmin) / max(dmax - dmin, 1e-6), 1.0)
                vis = (norm * 255).clip(0, 255).astype(np.uint8)
            else:
                vis = np.full(depth.shape, 255, dtype=np.uint8)
            Image.fromarray(vis).save(os.path.join(cam_dir, f"{fid}.png"))

    # ------------------------------------------------------------------
    # 语义分割
    # ------------------------------------------------------------------
    def bind_semantic_id_map(self, mesh_path_to_id: Dict[str, int]) -> None:
        """绑定 `mesh_path -> semantic_id` 映射, 并预生成可视化用的颜色 LUT。

        必须在 `get_cameras_semantic` / `save_cameras_semantic` 之前调用,
        否则会拿到全 0 的图。`apply_semantics_to_meshes` 写入 USD 的 `id_{N}`
        label 中的 N 与此映射的 semantic_id 必须一致(同一份字典)。
        """
        self._mesh_path_to_semantic_id = {str(k): int(v) for k, v in mesh_path_to_id.items()}
        max_id = max(self._mesh_path_to_semantic_id.values()) if self._mesh_path_to_semantic_id else 0
        # 0 留给背景(天空 / 未命中); 1..max_id 用高对比色
        colors = generate_high_contrast_colors(max(max_id, 1))
        lut = np.zeros((max_id + 1, 3), dtype=np.uint8)
        for sid in range(1, max_id + 1):
            c = colors[(sid - 1) % len(colors)]
            lut[sid] = (c[0], c[1], c[2])
        self._semantic_color_lut = lut
        logger.info(f"bind_semantic_id_map: 共 {len(self._mesh_path_to_semantic_id)} 个 mesh, max_id={max_id}")

    def _extract_semantic_id_from_label(self, label_data: Any) -> int:
        """从 Replicator 的 idToLabels 值中提取我们写入的 `id_{semantic_id}`。

        不同版本/schema 的输出形态:
            {"class": "id_3"} / {"class": ["id_3"]} / {"class": {"labels": ["id_3"]}}
            ["id_3"] / "id_3"
        """
        if label_data is None:
            return 0
        if isinstance(label_data, dict):
            if "class" in label_data:
                sid = self._extract_semantic_id_from_label(label_data["class"])
                if sid != 0:
                    return sid
            for value in label_data.values():
                sid = self._extract_semantic_id_from_label(value)
                if sid != 0:
                    return sid
            return 0
        if isinstance(label_data, (list, tuple, set)):
            for value in label_data:
                sid = self._extract_semantic_id_from_label(value)
                if sid != 0:
                    return sid
            return 0
        match = re.search(r"\bid_(\d+)\b", str(label_data))
        if match is None:
            return 0
        return int(match.group(1))

    def _extract_prim_path_from_label(self, label_data: Any) -> str:
        """从 instance_id_segmentation 的 idToLabels value 中提取 prim path 字符串。

        Replicator 不同版本形态:
            "/World/.../SM_xxx" / {"class": "/World/.../SM_xxx"} /
            {"instance": "/World/..."} / {"path": "/..."} / ["/World/..."]
        """
        if label_data is None:
            return ""
        if isinstance(label_data, str):
            s = label_data.strip()
            return s if s.startswith("/") else ""
        if isinstance(label_data, dict):
            for key in ("instance", "path", "primPath", "prim_path", "name", "class"):
                if key in label_data:
                    p = self._extract_prim_path_from_label(label_data[key])
                    if p:
                        return p
            for value in label_data.values():
                p = self._extract_prim_path_from_label(value)
                if p:
                    return p
            return ""
        if isinstance(label_data, (list, tuple, set)):
            for value in label_data:
                p = self._extract_prim_path_from_label(value)
                if p:
                    return p
            return ""
        s = str(label_data).strip()
        return s if s.startswith("/") else ""

    def _lookup_semantic_id_for_path(self, path: str) -> int:
        """根据 prim_path 反查 semantic_id, 找不到时沿 USD 父级上溯做兜底匹配。

        `mesh_path_to_id` 来自 get_mesh_paths 返回的 prim path (通常是带物理碰撞的
        Mesh, 在 Section 这一层); 而 instance_id annotator 在不同版本可能给出 mesh
        的父级 prim path (例如 SM_xxx)。逐级 parent 查询能稳定 hit。
        """
        if not path:
            return 0
        sid = self._mesh_path_to_semantic_id.get(path)
        if sid is not None:
            return int(sid)
        cur = path
        while "/" in cur and len(cur) > 1:
            cur = cur.rsplit("/", 1)[0]
            if not cur:
                break
            sid = self._mesh_path_to_semantic_id.get(cur)
            if sid is not None:
                return int(sid)
        return 0

    def _build_lut_from_instance_id_to_labels(self, id_to_labels: Dict[Any, Any]) -> np.ndarray:
        """用当帧 instance_id annotator 的 idToLabels 反查我们的 semantic_id。

        prim_path 在帧间稳定, 所以完全可以用当帧 idToLabels 重建 LUT, 不需要任何
        跨帧缓存。
        """
        if not id_to_labels:
            return np.zeros(1, dtype=np.uint16)
        items = []
        for k, v in id_to_labels.items():
            try:
                ik = int(k)
            except (TypeError, ValueError):
                continue
            items.append((ik, v))
        if not items:
            return np.zeros(1, dtype=np.uint16)

        max_annot_id = max(ik for ik, _ in items)
        lut = np.zeros(max_annot_id + 1, dtype=np.uint16)
        unmatched_examples: List[str] = []
        unmatched_count = 0
        for ik, v in items:
            path = self._extract_prim_path_from_label(v)
            if not path:
                continue
            sid = self._lookup_semantic_id_for_path(path)
            lut[ik] = sid
            if sid == 0:
                unmatched_count += 1
                if len(unmatched_examples) < 5:
                    unmatched_examples.append(f"annot_id={ik} path={path}")
        if unmatched_count > 0:
            logger.debug(
                "_build_lut_from_instance_id_to_labels: {} 个 prim_path 未在 mesh_path_to_semantic_id "
                "找到匹配, 这些像素会被打成 0(背景), 示例: {}", unmatched_count, unmatched_examples,
            )
        return lut

    def _build_lut_from_semantic_id_to_labels(self, id_to_labels: Dict[Any, Any]) -> np.ndarray:
        """[fallback] 用 semantic_segmentation 的 `id_{N}` label 反查 semantic_id。

        仅在 instance_id annotator 不可用时使用。
        """
        if not id_to_labels:
            return np.zeros(1, dtype=np.uint16)
        items = []
        for k, v in id_to_labels.items():
            try:
                ik = int(k)
            except (TypeError, ValueError):
                continue
            items.append((ik, v))
        if not items:
            return np.zeros(1, dtype=np.uint16)
        max_annot_id = max(ik for ik, _ in items)
        lut = np.zeros(max_annot_id + 1, dtype=np.uint16)
        for ik, v in items:
            sid = self._extract_semantic_id_from_label(v)
            lut[ik] = sid
        return lut

    def get_cameras_semantic(self) -> List[np.ndarray]:
        """获取每个相机的语义分割图(uint16, 像素值 = semantic_id, 0 = 背景/天空)。

        优先使用 instance_id_segmentation annotator(prim_path 帧间稳定),
        失败时回退到 semantic_segmentation。每帧重新构建 LUT, 不做跨帧缓存。
        """
        results: List[np.ndarray] = []
        for cam, cam_name in zip(self.isaac_cameras, self.cameras_name):
            frame = cam.get_current_frame()

            data = None
            id_to_labels: Dict[Any, Any] = {}
            used_channel = ""

            if self._instance_annotator_name:
                inst = frame.get(self._instance_annotator_name)
                if inst is not None:
                    if isinstance(inst, dict):
                        data = np.asarray(inst.get("data"))
                        info = inst.get("info", {}) or {}
                        id_to_labels = info.get("idToLabels", {}) or {}
                    else:
                        data = np.asarray(inst)
                    used_channel = "instance_id"

            if data is None or id_to_labels == {} or data.size == 0:
                sem = frame.get("semantic_segmentation")
                if sem is not None:
                    if isinstance(sem, dict):
                        data2 = np.asarray(sem.get("data"))
                        info2 = sem.get("info", {}) or {}
                        id_to_labels2 = info2.get("idToLabels", {}) or {}
                    else:
                        data2 = np.asarray(sem)
                        id_to_labels2 = {}
                    if data is None or data.size == 0:
                        data = data2
                        id_to_labels = id_to_labels2
                        used_channel = "semantic_seg"
                    elif id_to_labels == {} and id_to_labels2:
                        # instance 拿到 data 但 idToLabels 为空, semantic 通道有 idToLabels:
                        # 用 instance 的 data + semantic 的 labels 是不安全的, 整体 fallback
                        data = data2
                        id_to_labels = id_to_labels2
                        used_channel = "semantic_seg"

            width, height = self.resolutions[cam_name]
            if data is None:
                logger.warning("get_cameras_semantic[{}]: 两个 annotator 都未返回数据", cam_name)
                results.append(np.zeros((height, width), dtype=np.uint16))
                continue

            if data.ndim == 3:
                logger.warning("get_cameras_semantic[{}]: 收到彩色 annotator 输出, 取第 0 通道", cam_name)
                data = data[..., 0]
            data = data.astype(np.int32, copy=False)

            if used_channel == "instance_id":
                lut = self._build_lut_from_instance_id_to_labels(id_to_labels)
            else:
                lut = self._build_lut_from_semantic_id_to_labels(id_to_labels)

            if not id_to_labels:
                logger.warning(
                    "get_cameras_semantic[{}]: 当帧 idToLabels 为空(channel={}), 输出全 0",
                    cam_name, used_channel or "<none>",
                )
                results.append(np.zeros_like(data, dtype=np.uint16))
                continue

            if lut is None or len(lut) == 0:
                results.append(np.zeros_like(data, dtype=np.uint16))
                continue
            np.clip(data, 0, len(lut) - 1, out=data)
            remapped = lut[data]
            results.append(remapped.astype(np.uint16))

        return results

    def save_cameras_semantic(
        self,
        output_dir: str,
        frame_id: Optional[str] = None,
        path_idx: Optional[int] = None,
        point_idx: Optional[int] = None,
        save_visualization: bool = True,
    ) -> None:
        """保存每个相机的语义分割图。

        - <output_dir>/<camera>/<frame_id>.png : uint16 PNG, 值 = semantic_id
        - <output_dir>/../semantic_vis/<camera>/<frame_id>.png : 可视化彩图(可选)
        """
        fid = self._resolve_frame_id(frame_id, path_idx, point_idx)
        sems = self.get_cameras_semantic()
        for camera_name, sem in zip(self.cameras_name, sems):
            sem_path = os.path.join(output_dir, camera_name, f"{fid}.png")
            os.makedirs(os.path.dirname(sem_path), exist_ok=True)
            Image.fromarray(sem.astype(np.uint16), mode="I;16").save(sem_path)

            if save_visualization:
                vis_dir = os.path.join(os.path.dirname(output_dir), "semantic_vis", camera_name)
                os.makedirs(vis_dir, exist_ok=True)
                vis_path = os.path.join(vis_dir, f"{fid}.png")

                if self._semantic_color_lut is not None:
                    lut = self._semantic_color_lut
                    sem_clipped = np.clip(sem, 0, len(lut) - 1)
                    rgb = lut[sem_clipped]
                else:
                    sem_max = max(int(sem.max()), 1)
                    gray = (sem.astype(np.float32) / sem_max * 255).astype(np.uint8)
                    rgb = np.stack([gray, gray, gray], axis=-1)
                Image.fromarray(rgb.astype(np.uint8)).save(vis_path)

    def save_semantic_id_map(self, output_dir: str) -> None:
        """把 mesh_path <-> semantic_id 与 semantic_id -> RGB 落地为 JSON。"""
        import json
        os.makedirs(output_dir, exist_ok=True)

        id_map_path = os.path.join(output_dir, "semantic_id_map.json")
        id_to_path = {str(v): k for k, v in self._mesh_path_to_semantic_id.items()}
        with open(id_map_path, "w", encoding="utf-8") as f:
            json.dump({
                "mesh_path_to_id": self._mesh_path_to_semantic_id,
                "id_to_mesh_path": id_to_path,
            }, f, ensure_ascii=False, indent=2)

        if self._semantic_color_lut is not None:
            color_path = os.path.join(output_dir, "semantic_id_color.json")
            color_dict = {str(i): [int(c[0]), int(c[1]), int(c[2])]
                          for i, c in enumerate(self._semantic_color_lut)}
            with open(color_path, "w", encoding="utf-8") as f:
                json.dump(color_dict, f, ensure_ascii=False, indent=2)

        logger.info(f"save_semantic_id_map: 已写入 {id_map_path}")

    def save_cameras_mask(self, output_dir: str) -> None:
        """为每个相机生成有效像素 mask(255=有效, 0=无效)并保存为 PNG。

        - 鱼眼 LUT 相机:优先用 rayEnterDirection.exr 生成 bitmap mask
        - 其它带 omni 标定但无 LUT 的鱼眼:圆形 mask
        - 针孔等:矩形全 255 mask
        """
        for name in self.cameras_name:
            width, height = self.resolutions[name]
            mask = self._build_mask(name, width, height)
            mask_path = os.path.join(output_dir, f"{name}_mask.png")
            os.makedirs(os.path.dirname(mask_path), exist_ok=True)
            Image.fromarray(mask).save(mask_path)

    def _build_mask(self, name: str, width: int, height: int) -> np.ndarray:
        """构造相机的有效像素 mask(255=有效, 0=无效)。

        - LUT 鱼眼:优先用 rayEnterDirection.exr 的 bitmap(与 RTX 渲染 FOV 一致)
        - 否则圆形 mask(圆心/半径见 omni:calibration:maskCenterMode / maskRadius)
        - 其它相机:矩形全 255
        """
        lut_mask = self._lut_mask_bitmaps.get(name)
        if lut_mask is not None:
            h, w = lut_mask.shape[:2]
            if (w, h) != (width, height):
                from PIL import Image as PILImage
                lut_mask = np.array(
                    PILImage.fromarray(lut_mask).resize((width, height), PILImage.NEAREST)
                )
            return lut_mask

        omni_cal = self.calibration.get(name)
        if omni_cal is None:
            return np.full((height, width), 255, dtype=np.uint8)
        if omni_cal.get("mask_center_mode") == "imageCenter":
            cx = width / 2.0
            cy = height / 2.0
        else:
            cx = omni_cal["cx"]
            cy = omni_cal["cy"]
        radius = omni_cal.get("mask_radius")
        if radius is None:
            radius = min(cx, cy, width - cx, height - cy)
            logger.warning(
                f"相机 '{name}' 未 bake omni:calibration:maskRadius,"
                f"回退到内切圆 radius={radius:.1f}px (可能远小于真实鱼眼像点圆)。"
                f" 建议跑 tools/bake_camera_intrinsics.py --mask_radius {name}=<R>"
            )
        y, x = np.ogrid[:height, :width]
        dist2 = (x - cx) ** 2 + (y - cy) ** 2
        mask = (dist2 <= radius ** 2).astype(np.uint8) * 255
        return mask

    # ------------------------------------------------------------------
    # 打印
    # ------------------------------------------------------------------
    def print_all(self) -> None:
        """打印所有相机的内外参。"""
        np.set_printoptions(precision=6, suppress=True)
        logger.info("=" * 80)
        logger.info(f"CameraRig: {self.rig_prim_path}  ({len(self.cameras_name)} cameras)")
        logger.info(f"USD: {self.camera_usd_path}")
        logger.info("=" * 80)

        for name in self.cameras_name:
            intr = self.get_intrinsics(name)
            extr = self.get_extrinsics(name)

            logger.info("")
            logger.info(f"===== {name}  ({intr['prim_path']}) =====")
            logger.info(f"  projection_type        : {intr['projection_type']}")
            logger.info(
                f"  USD focal_length       : {intr['focal_length_mm']:.6f} mm  "
                f"(horizontal_aperture={intr['horizontal_aperture_mm']:.6f} mm, "
                f"vertical_aperture={intr['vertical_aperture_mm']:.6f} mm)"
            )
            if intr.get("clipping_range") is not None:
                logger.info(f"  clipping_range         : {intr['clipping_range']}")
            for k in (
                "generalizedProjectionDirectionTexturePath",
                "generalizedProjectionNDCTexturePath",
            ):
                if k in intr:
                    logger.info(f"  {k}: {intr[k]}")

            if "resolution" in intr:
                w, h = intr["resolution"]
                logger.info(
                    f"  USD-equivalent pinhole : fx={intr['fx_px_from_usd']:.4f}px  "
                    f"fy={intr['fy_px_from_usd']:.4f}px  "
                    f"cx={intr['cx_px_from_usd']:.4f}px  cy={intr['cy_px_from_usd']:.4f}px  "
                    f"resolution=({w}x{h})"
                )

            if "calibration" in intr:
                c = intr["calibration"]
                logger.info(
                    f"  {c['model']} intrinsics       : xi={c['xi']:.6f}  "
                    f"fx={c['fx_px']:.4f}px  fy={c['fy_px']:.4f}px  "
                    f"cx={c['cx_px']:.4f}px  cy={c['cy_px']:.4f}px"
                )
                logger.info(
                    f"  distortion ({c['distortion_model']}): {c['distortion_coeffs']}"
                )

            if "ftheta" in intr:
                f = intr["ftheta"]
                logger.info(
                    f"  ftheta intrinsics      : "
                    f"cx={f['cx']:.4f}px  cy={f['cy']:.4f}px  "
                    f"resolution=({f['width']}x{f['height']})  "
                    f"max_fov={f['max_fov_deg']:.2f}deg"
                )
                poly = f["poly"]
                logger.info(
                    f"  ftheta poly (forward A,B,C / inverse D,E,F): "
                    f"A={poly['A']}  B={poly['B']}  C={poly['C']}  "
                    f"D={poly['D']}  E={poly['E']}  F={poly['F']}"
                )

            # 外参打印
            T_rig_cam = extr["T_rig_cam"]
            T_world_cam = extr["T_world_cam"]

            t_rig = T_rig_cam[:3, 3]
            t_world = T_world_cam[:3, 3]
            logger.info(f"  Translate (rig frame)   : X={t_rig[0]:.6f}  Y={t_rig[1]:.6f}  Z={t_rig[2]:.6f}")
            logger.info(f"  Translate (world frame) : X={t_world[0]:.6f}  Y={t_world[1]:.6f}  Z={t_world[2]:.6f}")
            if "T_baselink_cam" in extr:
                t_base = extr["T_baselink_cam"][:3, 3]
                logger.info(
                    f"  Translate (baselink/IMU): X={t_base[0]:.6f}  Y={t_base[1]:.6f}  Z={t_base[2]:.6f}"
                )
                logger.info(f"  T_baselink_cam (RTX cam frame, X右Y上Z后):\n{extr['T_baselink_cam']}")
            logger.info(f"  T_rig_cam      (RTX cam frame):\n{T_rig_cam}")
            logger.info(f"  T_world_cam    (RTX cam frame):\n{T_world_cam}")
            logger.info(f"  T_world_cam_cv (OpenCV cam frame, X右Y下Z前):\n{extr['T_world_cam_cv']}")

        logger.info("=" * 80)
