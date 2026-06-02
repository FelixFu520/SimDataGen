import numpy as np
from scipy.spatial.transform import Rotation


def parse_T_ci(filepath):
    """从 kalibr 的 IMU-相机标定结果文件中解析 T_ci (imu to cam) 4x4 矩阵。"""
    with open(filepath, "r") as f:
        lines = f.readlines()

    matrix_lines = []
    capture = False
    for line in lines:
        if "T_ci:" in line and "imu" in line and "cam" in line:
            capture = True
            continue
        if capture:
            stripped = line.strip()
            if stripped.startswith("[") or stripped.startswith(" ["):
                row_str = stripped.strip("[]").split()
                matrix_lines.append([float(x.strip("[]")) for x in row_str])
            if len(matrix_lines) == 4:
                break

    return np.array(matrix_lines)


def parse_stereo_baseline(filepath):
    """从 kalibr 双目标定结果文件中解析 baseline T_1_0 为 4x4 矩阵。"""
    with open(filepath, "r") as f:
        lines = f.readlines()

    q = None
    t = None
    for line in lines:
        if line.strip().startswith("q:"):
            vals = line.split("[")[1].split("]")[0].split()
            q = [float(v) for v in vals]
        elif line.strip().startswith("t:"):
            vals = line.split("[")[1].split("]")[0].split()
            t = [float(v) for v in vals]

    rot = Rotation.from_quat(q)  # kalibr/scipy 均为 [x, y, z, w]
    T = np.eye(4)
    T[:3, :3] = rot.as_matrix()
    T[:3, 3] = t
    return T


def t_ci_to_isaac_sim_pose(T_ci):
    """
    给定 T_ci (imu→cam 的 4x4 变换矩阵)，
    计算以 IMU 为世界原点时相机的位姿，
    输出 Isaac Sim 格式的 Translate (xyz) 和 Orientation (xyz 欧拉角, 度数)。

    Isaac Sim 默认使用 XYZ 内旋(intrinsic)欧拉角。
    """
    T_ic = np.linalg.inv(T_ci)

    translate = T_ic[:3, 3]
    R = T_ic[:3, :3]

    rot = Rotation.from_matrix(R)
    euler_xyz_deg = rot.as_euler("XYZ", degrees=True)

    return translate, euler_xyz_deg


def print_pose(name, T_ci):
    translate, orientation = t_ci_to_isaac_sim_pose(T_ci)
    print(f"===== {name} (Isaac Sim Pose, IMU as world origin) =====")
    print(f"  T_ci (imu→cam):")
    print(f"  {T_ci}")
    print(f"  Translate:   X={translate[0]:.6f}  Y={translate[1]:.6f}  Z={translate[2]:.6f}")
    print(f"  Orientation: X={orientation[0]:.1f}  Y={orientation[1]:.1f}  Z={orientation[2]:.1f}")
    print()


def main():
    np.set_printoptions(precision=8, suppress=True)

    # CAM_A: 直接从 IMU-CAM 标定获得 T_ci_A (imu→camA)
    T_ci_A = parse_T_ci("docs/oak_camera/calibration/CAM_A-results-imucam.txt")
    print_pose("CAM_A (直接标定)", T_ci_A)

    # 双目标定: T_AB 是 cam0(CAM_A) → cam1(CAM_B) 的变换
    T_AB = parse_stereo_baseline("docs/oak_camera/calibration/log1-results-cam.txt")

    # 推算 CAM_B 的 T_ci: T_ci_B = T_AB @ T_ci_A
    # 链路: imu --T_ci_A--> camA --T_AB--> camB
    T_ci_B = T_AB @ T_ci_A
    print_pose("CAM_B (由 CAM_A + 双目baseline 推算)", T_ci_B)

    # 双目标定: T_BC 是 cam0(CAM_B) → cam1(CAM_C) 的变换
    T_BC = parse_stereo_baseline("docs/oak_camera/calibration/log1-results-cam (1).txt")

    # 推算 CAM_C 的 T_ci: T_ci_C = T_BC @ T_ci_B
    # 链路: imu --T_ci_A--> camA --T_AB--> camB --T_BC--> camC
    T_ci_C = T_BC @ T_ci_B
    print_pose("CAM_C (由 A→B→C 推算)", T_ci_C)

    # 双目标定: T_CD 是 cam0(CAM_C) → cam1(CAM_D) 的变换
    T_CD = parse_stereo_baseline("docs/oak_camera/calibration/log1-results-cam (2).txt")

    # 推算 CAM_D 的 T_ci: T_ci_D = T_CD @ T_ci_C
    # 链路: imu → camA → camB → camC → camD
    T_ci_D = T_CD @ T_ci_C
    print_pose("CAM_D (由 A→B→C→D 推算)", T_ci_D)

    # 验证闭环: T_DA 是 cam0(CAM_D) → cam1(CAM_A) 的变换
    T_DA = parse_stereo_baseline("docs/oak_camera/calibration/log1-results-cam (3).txt")
    T_ci_A_loop = T_DA @ T_ci_D
    print_pose("CAM_A (闭环验证: A→B→C→D→A)", T_ci_A_loop)

    # 共面性检查
    print("=" * 60)
    print("共面性检查 (各相机在 IMU 坐标系下的位置):")
    print("=" * 60)
    for name, T_ci in [("CAM_A", T_ci_A), ("CAM_B", T_ci_B), ("CAM_C", T_ci_C), ("CAM_D", T_ci_D)]:
        pos = np.linalg.inv(T_ci)[:3, 3]
        print(f"  {name}: X={pos[0]:.4f}  Y={pos[1]:.4f}  Z={pos[2]:.4f}")


if __name__ == "__main__":
    main()
