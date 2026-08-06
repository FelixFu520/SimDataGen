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

标定数据：`docs/oak_camera/calibration20260806/fisheye_cams.yaml`  
纹理目录：`assets/cameras/oak_camera_texture_20260806/`  
输出 USD：`assets/cameras/oak_camera_4lut_2H110SA_20260806.usd`（由原版复制后 bake，不修改原 `oak_camera_4lut_2H110SA.usd`）

```
cp assets/cameras/oak_camera_4lut_2H110SA.usd \
   assets/cameras/oak_camera_4lut_2H110SA_20260806.usd
```

### 1. 生成 LUT 纹理
```
./app/python.sh tools/cameras/oak_generate_lut_textures.py \
    --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml \
    --output_dir assets/cameras/oak_camera_texture_20260806
```

### 2. 写入纹理路径
```
./app/python.sh tools/cameras/oak_set_camera_lut_texture_paths.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
    --texture_dir assets/cameras/oak_camera_texture_20260806
```

### 3. bake 内参到 usd
```
./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
    --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml \
    --texture_dir assets/cameras/oak_camera_texture_20260806
```
（`maskRadius` 与 `verticalAperture` 由 bake 脚本自动处理，见 [camera_OAK.md](camera_OAK.md)）

### 4. bake 外参到 usd
从 yaml 的 `T_cam_imu`（本仓库约定等同 Kalibr `T_ci`，imu→cam，OpenCV 系）写入 4 路鱼眼 translate + orient。  
旋转需做 OpenCV→USD 转换 `R_usd = R_ic @ diag(-1,1,-1)`（绕相机 Y 转 180°：光轴朝外且画面不颠倒；`diag(1,-1,-1)` 会上下颠倒）：
```
./app/python.sh tools/cameras/oak_bake_camera_extrinsics_from_yaml.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
    --yaml docs/oak_camera/calibration20260806/fisheye_cams.yaml
```

本次 bake 后鱼眼位姿（IMU/base_link 下，光轴朝外、画面正向）：

| 相机 | Translate (m) | Orientation XYZ (°) |
|------|---------------|---------------------|
| CAM_A | 0.087717, -0.068214, 0.008430 | -88.5, -48.0, -179.8 |
| CAM_B | 0.072117, 0.073447, 0.003153 | 90.6, -42.0, -0.2 |
| CAM_C | -0.075247, 0.066531, 0.011970 | 89.7, 47.7, 1.2 |
| CAM_D | -0.070606, -0.080998, 0.004269 | -88.0, 42.2, 177.6 |

说明：后续不再使用 `CAM_Front` / `CAM_Back`，已从 `oak_camera_4lut_2H110SA_20260806.usd` 中删除，仅保留 4 路鱼眼（CAM_A~D）。mask 由 USD 内 LUT EXR / `maskRadius` 自动生成，无需额外适配。

### 5. 打印相机参数
```
./app/python.sh tools/cameras/print_cameraRig.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_20260806.usd
```

### 6. 验证相机
#### 采集数据
```
./app/python.sh gen_data.py \
--seed 0 \
--scene_usd_url /home/fufa/projects2026/SimDataGen/extern_asserts/TaoBao03/108_Bazaar/Demo.usd \
--camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA_20260806.usd \
--output_dir /home/fufa/projects2026/SimDataGen/workdir/108_Bazaar_oak_camera_4lut_2H110SA_20260806 \
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

#### 投影验证
```
./app/python.sh project_cloud.py \
    --data_dir workdir/108_Bazaar_oak_camera_4lut_2H110SA_20260806 \
    --show_num 60
```

#### mask 验证
```
./app/python.sh tools/check_data/overlay_mask_verify.py \
    --base workdir/108_Bazaar_oak_camera_4lut_2H110SA_20260806
```
