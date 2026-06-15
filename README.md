# SimDataGen
使用isaacsim在场景中采集数据

## 示例视频
- [SimDataGen示例视频 1](https://www.bilibili.com/video/BV1v5Es6yEJe/)
- [SimDataGen示例视频 2](https://www.bilibili.com/video/BV1i5Es6yEcC/)

## 安装
[安装教程](docs/install.md)

## 思路
[思路/流程](docs/method.md)

## 支持相机/工装
- [ZEDX双目相机](docs/camera_zedx.md)
- [oak工装](docs/camera_OAK.md)
- [oak+2pinhole工装](docs/camera_OAK_H30YA.md)
- [oak+2pinhole工装-内外参扰动](docs/camera_OAK_H30YA_intrinsics_extrinsics_perturbed.md)
- [oak+2pinhole工装(针孔是H110SA)](docs/camera_OAK_H110SA.md)
- [oak+2pinhole工装(针孔是H110SA), 规则外参](docs/camera_OAK_H110SA_regular.md)


## 使用
### 采集数据
```
./app/python.sh gen_data.py \
--seed 0 \
--scene_usd_url /home/fufa/projects2026/SimDataGen/asset_extern/TaoBao03/108_Bazaar/Demo.usd \
--camera_usd_url /home/fufa/projects2026/SimDataGen/assets/cameras/ZED_X.usdc \
--output_dir /home/fufa/projects2026/SimDataGen/workdir/108_Bazaar_ZEDX \
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
./app/python.sh project_cloud.py --data_dir workdir/108_Bazaar_ZEDX/ --show_num 60
```