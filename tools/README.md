# tools

SimDataGen 辅助脚本集合，按功能分为相机配置、采数演示、数据检查、USD 批处理、导出与云端任务等子目录。需要 Isaac Sim 环境的脚本统一通过 `./app/python.sh` 调用。

## 目录结构

| 目录 / 文件 | 说明 |
|---|---|
| `cameras/` | OAK 鱼眼相机标定、LUT、内外参 bake |
| `demo_data/` | 手动轨迹录制与按轨迹采数 |
| `check_data/` | 采数结果检查与可视化 |
| `usd/` | 场景 USD 批量修改 |
| `export/` | Blender / 采数结果导出 |
| `volcengine/` | 火山引擎 ML 任务提交 |
| `capture_and_project.py` | 最小端到端采集 + 反投影测试 |

---

## demo_data — 手动轨迹录制与采数

用于 demo 流程：在 Isaac Sim 中键盘遥操 CameraRig 录制轨迹，再按轨迹批量采 RGB / 深度 / 语义。

### 典型流程

1. **终端 A**（Isaac Sim）：启动录制器  
   ```bash
   ./tools/demo_data/run_record_camera_rig_trajectory.sh \
       --scene_usd asset_extern/home_000/interior_template.usdc \
       --camera_usd assets/cameras/oak_camera_4lut_2H30YA.usd \
       --output_dir workdir/demo_trajectory/home_000_manual
   ```
2. **终端 B**（系统 ROS2）：键盘遥操  
   ```bash
   source /opt/ros/humble/setup.bash
   python tools/demo_data/keyboard_camera_rig_teleop.py
   ```
3. 按轨迹采数：  
   ```bash
   ./tools/demo_data/run_gen_data_from_trajectory.sh \
       --scene_usd_url ... --camera_usd_url ... \
       --trajectory_dir workdir/demo_trajectory/home_000_manual \
       --output_dir workdir/demo_data/home_000_manual
   ```

### 脚本说明

| 脚本 | 功能 |
|---|---|
| `run_record_camera_rig_trajectory.sh` | 封装启动脚本：自动配置 Isaac Sim ROS2 bridge 环境变量（`ROS_DISTRO`、`LD_LIBRARY_PATH`、`PYTHONPATH` 等），避免与系统 `/opt/ros` 的 Python 3.10 冲突 |
| `record_camera_rig_trajectory.py` | 在 Isaac Sim 中加载场景 + CameraRig，经 ROS2 接收键盘遥操指令，录制 rig 位姿序列，保存为 `rig_poses_XXXX.npy` |
| `keyboard_camera_rig_teleop.py` | 键盘遥操节点（系统 Python + ROS2）：`a/d/w/s` 平移、`u/i` 升降、`z/c` yaw 旋转、`j/k` 开始/停止录制；发布 `/camera_rig/nudge`、`/camera_rig/record` 等 topic |
| `ros2_bridge_env.py` | ROS2 bridge 环境变量设置库，供 `record_camera_rig_trajectory.py` 在启动 SimulationApp 前调用 |
| `run_gen_data_from_trajectory.sh` | 封装启动 `gen_data_from_trajectory.py` |
| `gen_data_from_trajectory.py` | 读取 `rig_poses_*.npy` 轨迹，在场景中按位姿采 RGB / 深度 / 语义，输出布局与 `gen_data.py` 一致；跳过 occupancy 与自动路径生成 |

---

## cameras — OAK 相机配置

围绕 Kalibr omni 标定 yaml，生成 LUT 纹理、bake 内外参到 USD，并支持扰动版相机组搭建。

### 推荐流水线

```
fisheye_cams.yaml
  → oak_generate_lut_textures.py      # 生成 EXR LUT
  → oak_set_camera_lut_texture_paths.py  # 写入 USD 纹理路径
  → oak_bake_camera_intrinsics.py     # bake 内参 + maskRadius
  → oak_bake_camera_extrinsics.py     # bake 外参平移扰动（可选）
```

一键搭建内参扰动变体：`oak_setup_intrinsics_change_variant.py`（复制 USD → 生成 yaml → LUT → bake → 写路径）。

### 脚本说明

| 脚本 | 功能 |
|---|---|
| `oak_generate_lut_textures.py` | 从 Kalibr omni 标定 yaml 生成 Mei 鱼眼模型的 `rayEnterDirection` / `rayExitPosition` EXR 纹理，供 Isaac Sim `OmniLensDistortionLutAPI` 使用 |
| `oak_set_camera_lut_texture_paths.py` | 将 LUT 纹理路径写入相机 USD（相对 USD 目录），无需在 Isaac Sim 中手改 |
| `oak_bake_camera_intrinsics.py` | 将 yaml 内参及自定义分辨率 bake 进 USD（`omni:calibration:*` 属性）；自动估计 `maskRadius`、修正 `verticalAperture` |
| `oak_bake_camera_extrinsics.py` | 将外参平移 xyz 扰动 bake 进 USD（仅改 translate，不碰旋转，避免朝向漂移） |
| `oak_generate_perturbed_yaml.py` | 从原始标定生成扰动版 `fisheye_cams.yaml`；支持 `small_change` / `pinhole_like` / `fisheye_like` / `extrinsics_change` 等 profile |
| `oak_setup_intrinsics_change_variant.py` | 一键搭建内参扰动相机组（`pinhole_like`、`fisheye_like` 等变体） |
| `oak_compute_mask_radius.py` | 单独估计鱼眼 LUT 相机的 `maskRadius`（像素），用于预览或调试；bake 时已内置自动估计 |
| `oak_test_lut.py` | 渲染 CAM_A 的 LUT 鱼眼 vs 针孔对比，验证 LUT 畸变效果（`cubes` 或 `interior` 场景） |
| `print_cameraRig.py` | 加载相机组 USD，打印所有相机的内外参 |
| `oak_camera_extrinsics.py` | Kalibr 标定文件解析与 `T_ci` → Isaac Sim 位姿换算；从 IMU-相机标定 + 双目标定 baseline 推算 CAM_A~D 外参 |
| `oak_extrinsics_perturb.py` | 外参小幅随机扰动库（仅平移 xyz，默认 ±1 mm，可复现） |

---

## check_data — 数据检查与可视化

对 `gen_data` / `gen_data_from_trajectory` 产出目录做完整性检查、预览与 Foxglove 可视化。

| 脚本 | 功能 |
|---|---|
| `check_datagen_integrity.py` | 检查采数目录完整性：解析 `gen_data.log` 成功标志，统计 `rgb/CAM_A` 帧数 |
| `make_rgb_depth_video.py` | 将多相机 RGB + Depth 按 2×2 布局拼接，导出 H.264 预览视频（默认最长边 2048） |
| `depth_npy_filter_normalize.py` | 深度 `.npy` 过滤非有限值与超阈值像素，min-max 归一化后存 PNG |
| `overlay_mask_verify.py` | 将 mask 叠加到 RGB / 深度图上，检查 mask 对齐是否正确 |
| `merge_ply_dedup.py` | 合并目录下 `<prefix>*.ply` 点云文件，按 xyz 去重 |
| `path_vis_to_mcap.py` | 将 `vis/` 帧点云、轨迹 `paths.npy`、占据栅格写入 MCAP，供 Foxglove 播放（含 `/tf`、`/sim/pointcloud`、`/sim/path` 等 topic） |

---

## usd — 场景 USD 批量修改

批量处理 Blender 导出的室内场景 USD，通过子进程隔离防止坏文件导致整体崩溃。

| 脚本 | 功能 |
|---|---|
| `modify_usd_root_scale.py` | 批量将项目级 USD 的 `/root` scale 改为指定值（默认 `0.01`，cm → m）；跳过 `Props/`、`Materials/` 子目录 |
| `modify_usd_light.py` | 批量修改 USD 中所有 Light 的 `intensity` |
| `modify_usd_colliders.py` | 批量给 Mesh 添加物理碰撞属性（`UsdPhysics.CollisionAPI`）；支持 `none` / `convexHull` 等 approximation |
| `print_usdc.py` | Dump 相机组 USD 结构（prim 层级、属性等），用于调试 |

> 以上 USD 修改脚本需在仓库根目录通过 `./app/python.sh` 运行。

---

## export — 资产与样本导出

| 脚本 | 功能 |
|---|---|
| `export_usd_from_blender.py` | 扫描 Blender 资产目录（`<index> - <name>` 或 `<index>_<name>` 格式），批量 headless 导出为 USD |
| `export_omni_samples_excel.py` | 从 workdir 按场景聚合任务目录，每场景随机选一个 seed，将 CAM_A/B/C/D 的 rgb、depth、semantic_vis 首帧嵌入 Excel |

---

## volcengine — 云端任务

| 脚本 | 功能 |
|---|---|
| `submit_volcengine.py` | 通过火山引擎 ML Platform SDK 提交训练/采数任务；支持配置资源队列、GPU 规格、TOS / vePFS 挂载、自定义镜像与执行命令 |

---

## 根目录脚本

### `capture_and_project.py`

最小端到端测试：在指定位姿 `(1,1,1)` 采集一组相机的 RGB + Depth，反投影为世界系点云并写 PLY。

流程：
1. 启动 SimulationApp（PathTracing，与 `gen_data` 一致）
2. 加载场景 + 相机组 USD
3. CameraRig 设置位姿并渲染
4. 保存 RGB、Depth、Mask、内外参
5. 纯 numpy 反投影（Mei omni 模型 + radtan 去畸变）→ 世界系 PLY

```bash
./app/python.sh tools/capture_and_project.py \
    --scene_usd /path/to/scene.usd \
    --camera_usd assets/cameras/oak_camera_4lut_2H30YA.usd \
    --output_dir workdir/capture_project_test
```
