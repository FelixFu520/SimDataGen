# OAK+H110SA
在[OAK相机](camera_OAK.md) 基础上增加两个H110SA的针孔相机
## 组装相机
在`oak_camera_4lut.usd`基础上, 添加两个针孔相机, 使用Isaacsim UI界面添加, 结果保存成`oak_camera_4lut_2H110SA.usd`

### bake参数到usd 
```
./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA.usd \
    --yaml docs/oak_camera/calibration/fisheye_cams.yaml \
    --texture_dir assets/cameras/oak_camera_texture \
    --resolution CAM_Front=1920x1200 \
    --resolution CAM_Back=1920x1200
```
（`maskRadius` 与 `verticalAperture` 由 bake 脚本自动处理，见 [camera_OAK.md](camera_OAK.md)）
### 打印相机参数
```
./app/python.sh tools/cameras/print_cameraRig.py --usd assets/cameras/oak_camera_4lut_2H110SA.usd
```

## 验证相机
### 采集数据
```
./app/python.sh gen_data.py \
--seed 0 \
--scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao03/108_Bazaar/Demo.usd \
--camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA.usd \
--output_dir /home/fufa/projects2026/SimDataGen/workdir/108_Bazaar_oak_camera_4lut_2H110SA \
--occupancy_resolution 0.25 \
--num_points 60 \
--num_paths 1 \
--max_angle_deviation 4 \
--erode_iterations 2 \
--obstacle_dilate_iterations 1 \
--obstacle_envelope_iterations 10 \
--step_size_xy 0.25 \
--step_size_z 0.25 \
--max_dz_per_step 0.25 \
--min_path_extent 1 \
--min_path_compact_window 10 \
--max_path_generation_attempts 10000
```

### 投影验证
```
./app/python.sh project_cloud.py --data_dir workdir/108_Bazaar_oak_camera_4lut_2H110SA --show_num 60
```
### mask验证
```
./app/python.sh tools/check_data/overlay_mask_verify.py --base workdir/108_Bazaar_oak_camera_4lut_2H110SA
```

## 2026-08-06 重新标定

- 标定数据：`docs/oak_camera/calibration20260806/fisheye_cams.yaml`（只含 4 路 OAK 鱼眼 `cam0..cam3` → `CAM_A..CAM_D` 的 omni 内参与 `T_cam_imu`，无 imucam / 双目 baseline 文件，故外参只能取自 `T_cam_imu`）
- 纹理目录：`assets/cameras/oak_camera_texture_20260806/`
- 输出 USD：`assets/cameras/oak_camera_4lut_2H110SA_20260806.usd`（由原版 `oak_camera_4lut_2H110SA.usd` 复制后处理，**不修改原文件**）

> **坐标系提醒（关键）**：Kalibr/OpenCV 相机系为 X右 Y下 Z前（+Z 为光轴）；Isaac Sim RTX 相机系为 X右 Y上 Z后（-Z 为视线）。转换矩阵 `R_cv_to_isaac = diag(1, -1, -1)`。LUT 纹理生成、外参写入都必须处理该差异，否则相机会朝向错误。

### 1. 生成 LUT 纹理（含 OpenCV→RTX 翻转）
```
./app/python.sh tools/cameras/oak_generate_lut_textures.py \
    --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml \
    --output_dir assets/cameras/oak_camera_texture_20260806
```

### 2. 复制 USD 并 bake 内参
```
cp assets/cameras/oak_camera_4lut_2H110SA.usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd

./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
    --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml \
    --texture_dir assets/cameras/oak_camera_texture_20260806 \
    --resolution CAM_Front=1920x1200 \
    --resolution CAM_Back=1920x1200
```
（`maskRadius` 与 `verticalAperture` 由 bake 脚本自动处理；`CAM_Front/CAM_Back` 是 H110SA 针孔相机，只 bake 分辨率）

### 3. 写入 LUT 纹理路径
bake 脚本不改纹理路径，复制来的 USD 仍指向旧纹理目录，需重写为新目录（相对 USD 目录）：
```
./app/python.sh tools/cameras/oak_set_camera_lut_texture_paths.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
    --texture_dir assets/cameras/oak_camera_texture_20260806
```

### 4. 按新标定 `T_cam_imu` 写鱼眼外参
bake 只写内参，不动相机在 rig 中的摆位。按新标定重摆 4 路鱼眼外参（`T_baselink_cam = inv(T_cam_imu) @ R_cv_to_isaac`，只动 CAM_A/B/C/D，针孔不动）：
```
./app/python.sh tools/cameras/oak_set_extrinsics_from_tcamimu.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
    --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml
```
> 注：`T_cam_imu` 的平移（IMU→相机杆臂）噪声较大；若日后拿到新的双目 baseline / imucam 文件，用 `oak_camera_extrinsics.py` 那套链式推算的相机间平移更准。

### 5. 整体翻转 180° 修正朝向
直接用 `T_cam_imu` 写入的鱼眼会出现「图像上下颠倒 + 4 相机逆时针」，与真实工装（顺时针安装、正视角）相反。绕 `base_link`（工装中心）翻转 180° 可同时把 up 翻正、把环绕方向反向；这是**刚体旋转**，保持相机间相对外参不变（`common` 仍与 yaml 一致）。针孔 `CAM_Front/CAM_Back` 本就正立，**不翻**：
```
./app/python.sh tools/cameras/oak_flip_cameras_180.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
    --cameras CAM_A CAM_B CAM_C CAM_D --axis x
```
> `--axis x/y/z` 对应绕 base_link 哪条轴翻转；x 与 y 会相差一个 180° 偏航。翻转后核对渲染图（正视角 + 顺时针）即可；若整体方位差 180°，改用 `--axis y` 重跑。

### 6. 打印/核对参数
```
./app/python.sh tools/cameras/print_cameraRig.py --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd
```

## 验证相机（2026-08-06）

### 采集数据
```
./app/python.sh gen_data.py \
--seed 0 \
--scene_usd_url /home/fufa/projects2026/SimDataGen/extern_asserts/Intime/home_000/interior_template.usdc \
--camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
--output_dir /home/fufa/projects2026/SimDataGen/workdir/home_000_oak_camera_4lut_2H110SA_20260806 \
--occupancy_resolution 0.25 \
--num_points 60 \
--num_paths 1 \
--max_angle_deviation 4 \
--erode_iterations 2 \
--obstacle_dilate_iterations 1 \
--obstacle_envelope_iterations 10 \
--step_size_xy 0.25 \
--step_size_z 0.25 \
--max_dz_per_step 0.25 \
--min_path_extent 1 \
--min_path_compact_window 10 \
--max_path_generation_attempts 10000
```

### 内外参对齐验证（common vs 标定 yaml）
把 `common/*.npy` 里的内外参与标定 yaml 对比：内参应逐项一致；外参按「相机间相对位姿」（OpenCV 约定）对比应一致。
```
./app/python.sh tools/check_data/compare_common_vs_calibration.py \
    --data_dir workdir/home_000_oak_camera_4lut_2H110SA_20260806 \
    --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml
```
> `common` 的相机间外参与世界外参都是 OpenCV 约定（`get_camera_to_world_opencv` 已做 RTX→CV 转换）；yaml 用 `T_camj_cami = T_cam_imu[j] @ inv(T_cam_imu[i])` 换算相对位姿。相机间相对位姿对 rig 整体翻转不敏感，故第 5 步翻转后仍保持一致。

### 投影 / mask 验证（可选）
```
./app/python.sh project_cloud.py --data_dir workdir/home_000_oak_camera_4lut_2H110SA_20260806 --show_num 60
./app/python.sh tools/check_data/overlay_mask_verify.py --base workdir/home_000_oak_camera_4lut_2H110SA_20260806
```
