"""外参小幅度随机扰动（可复现）及 T_cam_imu ↔ Isaac Sim 位姿换算。"""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

# 默认小幅波动：平移 ±3 mm，旋转 ±1.5°（各轴独立均匀分布）
DEFAULT_TRANS_RANGE_M = 0.003
DEFAULT_ROT_RANGE_DEG = 1.5

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


def perturb_T_cam_imu(
    T_cam_imu,
    rng: np.random.Generator,
    *,
    trans_range_m: float = DEFAULT_TRANS_RANGE_M,
    rot_range_deg: float = DEFAULT_ROT_RANGE_DEG,
) -> np.ndarray:
    """在相机坐标系施加小随机扰动：T' = T @ T_delta。"""
    T = _as_matrix4(T_cam_imu)
    dt = rng.uniform(-trans_range_m, trans_range_m, size=3)
    dr_deg = rng.uniform(-rot_range_deg, rot_range_deg, size=3)
    R_delta = Rotation.from_euler("xyz", dr_deg, degrees=True).as_matrix()
    T_delta = np.eye(4)
    T_delta[:3, :3] = R_delta
    T_delta[:3, 3] = dt
    return T @ T_delta


def rng_for_camera(seed: int, cam_name: str) -> np.random.Generator:
    offset = CAM_SEED_OFFSET.get(cam_name, abs(hash(cam_name)) % 10000)
    return np.random.default_rng(int(seed) + offset)


def perturb_local_transform(
    T_local: np.ndarray,
    rng: np.random.Generator,
    *,
    trans_range_m: float = DEFAULT_TRANS_RANGE_M,
    rot_range_deg: float = DEFAULT_ROT_RANGE_DEG,
) -> np.ndarray:
    """对任意 4×4 局部变换施加与 T_cam_imu 相同量级的右乘扰动。"""
    return perturb_T_cam_imu(
        T_local, rng, trans_range_m=trans_range_m, rot_range_deg=rot_range_deg
    )
