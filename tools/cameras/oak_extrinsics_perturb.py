"""外参小幅度随机扰动（可复现）及 T_cam_imu 辅助换算。

外参扰动**仅**修改平移 xyz，不修改任何旋转分量。
"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

# 默认：平移各轴 ±1 mm
DEFAULT_TRANS_RANGE_M = 0.001

CAM_NAMES = ("CAM_A", "CAM_B", "CAM_C", "CAM_D")

CAM_SEED_OFFSET = {
    "CAM_A": 101,
    "CAM_B": 202,
    "CAM_C": 303,
    "CAM_D": 404,
    "CAM_Front": 505,
    "CAM_Back": 606,
}


def _as_matrix4(T) -> np.ndarray:
    return np.asarray(T, dtype=np.float64).reshape(4, 4)


def t_cam_imu_to_T_ci(T_cam_imu) -> np.ndarray:
    """Kalibr T_cam_imu (cam→imu) → T_ci (imu→cam)。"""
    return np.linalg.inv(_as_matrix4(T_cam_imu))


def t_ci_to_isaac_sim_pose(T_ci) -> tuple[np.ndarray, np.ndarray]:
    """T_ci (imu→cam) → Isaac Sim Translate + XYZ 内旋欧拉角（度）。"""
    T_ic = np.linalg.inv(_as_matrix4(T_ci))
    translate = T_ic[:3, 3].copy()
    euler_xyz_deg = Rotation.from_matrix(T_ic[:3, :3]).as_euler("XYZ", degrees=True)
    return translate, euler_xyz_deg


def t_cam_imu_to_isaac_sim_pose(T_cam_imu) -> tuple[np.ndarray, np.ndarray]:
    return t_ci_to_isaac_sim_pose(t_cam_imu_to_T_ci(T_cam_imu))


def matrix_to_yaml_nested(T: np.ndarray) -> list[list[float]]:
    return _as_matrix4(T).tolist()


def perturb_translate(rng: np.random.Generator, trans_range_m: float) -> np.ndarray:
    return rng.uniform(-trans_range_m, trans_range_m, size=3)


def perturb_T_cam_imu_translate(
    T_cam_imu,
    rng: np.random.Generator,
    *,
    trans_range_m: float = DEFAULT_TRANS_RANGE_M,
) -> np.ndarray:
    """仅扰动 T_cam_imu 平移列，旋转矩阵不变。"""
    T = _as_matrix4(T_cam_imu).copy()
    T[:3, 3] += perturb_translate(rng, trans_range_m)
    return T


def rng_for_camera(seed: int, cam_name: str) -> np.random.Generator:
    offset = CAM_SEED_OFFSET.get(cam_name, abs(hash(cam_name)) % 10000)
    return np.random.default_rng(int(seed) + offset)
