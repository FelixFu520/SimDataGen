# OAK+H110SA
在[OAK相机](camera_OAK.md) 基础上增加两个H110SA的针孔相机
## 组装相机
在`oak_camera_4lut.usd`基础上, 添加两个针孔相机, 使用Isaacsim UI界面添加, 结果保存成`oak_camera_4lut_2H110SA_regular.usd`

### bake参数到usd 
```
./app/python.sh tools/cameras/oak_bake_camera_intrinsics.py \
    --usd assets/cameras/oak_camera_4lut_2H110SA_regular.usd \
    --yaml docs/oak_camera/calibration/fisheye_cams.yaml \
    --texture_dir assets/cameras/oak_camera_texture
```
（`maskRadius` 与 `verticalAperture` 由 bake 脚本自动处理，见 [camera_OAK.md](camera_OAK.md)）

针孔相机保持原有分辨率
### 打印相机参数
```
./app/python.sh tools/cameras/print_cameraRig.py --usd assets/cameras/oak_camera_4lut_2H110SA_regular.usd
```

## 验证相机
### 采集数据
```
./app/python.sh gen_data.py \
--seed 0 \
--scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao03/108_Bazaar/Demo.usd \
--camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/oak_camera_4lut_2H110SA_regular.usd \
--output_dir /home/fufa/projects2026/SimDataGen/workdir/108_Bazaar_oak_camera_4lut_2H110SA_regular \
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
./app/python.sh project_cloud.py --data_dir workdir/108_Bazaar_oak_camera_4lut_2H110SA_regular --show_num 60
```
### mask验证
```
./app/python.sh tools/check_data/overlay_mask_verify.py --base workdir/108_Bazaar_oak_camera_4lut_2H110SA_regular
```
