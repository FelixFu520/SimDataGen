# OAK-4P-New-B036801 迁移至 Isaac Sim 思路整理

## 1. 问题定义

**核心矛盾**：OAK-4P-New-B036801 相机使用 Kalibr 标定后输出 `camera_model: omni`（Mei 统一全向模型），但 Isaac Sim 原生 UI 不支持 Mei 模型的 ξ (xi) 参数。强行代入其他模型（如 fisheyeKannalaBrandtK3）会导致严重的重投影误差。

**解决方案**：使用 Isaac Sim 的 **广义投影模型 (Generalized Projection / OmniLensDistortionLutAPI)**，通过预计算的 LUT 纹理（EXR 格式）将 Mei 模型的光线映射关系"烘焙"进两张纹理中。

## 2. 校准数据解析

`docs/oak_camera/calibration/fisheye_cams.yaml` 包含 4 个相机（cam0-cam3），每个都是 `camera_model: omni`。

**Mei 模型 intrinsics 格式**：`[xi, fx, fy, cx, cy]`

| 相机 | xi | fx | fy | cx | cy | 分辨率 |
|------|-----|-----|-----|-----|-----|--------|
| cam0 (CAM_A) | 2.120 | 1867.92 | 1874.46 | 960.90 | 547.17 | 1920x1200 |
| cam1 (CAM_B) | 1.337 | 1374.25 | 1380.05 | 944.79 | 548.29 | 1920x1200 |
| cam2 (CAM_C) | 2.242 | 1929.62 | 1936.15 | 964.12 | 578.15 | 1920x1200 |
| cam3 (CAM_D) | 2.071 | 1836.77 | 1842.88 | 943.87 | 675.43 | 1920x1200 |

**畸变模型**：`distortion_model: radtan`，系数 `[k1, k2, p1, p2]`

## 3. Mei 统一全向模型数学描述

### 3.1 正向投影 (3D → 2D pixel)

给定 3D 点 P = (X, Y, Z)：

1. **单位球面投影**: `Ps = P / ||P||`
2. **ξ 平移**: `Ps' = Ps + [0, 0, ξ]^T`
3. **透视投影**: `m = [Ps'_x / Ps'_z, Ps'_y / Ps'_z]^T`
4. **RadTan 畸变 + 内参**: `pixel = K * distort(m)`

### 3.2 逆向投影 (2D pixel → 3D ray)

给定像素 (u, v)：

1. **内参逆变换**: `m_d = K^(-1) * [u, v, 1]^T`
2. **迭代去畸变**: 牛顿法求解无畸变的 m
3. **逆 Mei 变换**: 从 m 和 ξ 恢复单位球面上的方向

   关键推导：设 `r² = mx² + my²`
   - `a = r² + 1`
   - `b = 2ξr²`
   - `c = ξ²r² - 1`
   - `zs = (-b + sqrt(b² - 4ac)) / (2a)`
   - `scale = zs + ξ`
   - 方向: `(mx * scale, my * scale, zs)`

## 4. LUT 纹理生成策略

### 4.1 rayEnterDirectionTexture（像素 → 光线方向）

- **格式**: RGB 32-bit float EXR
- **尺寸**: ≥ 传感器分辨率（使用 tex_scale=2 提升精度）
- **编码**: UV = NDC (0,0)→(1,1) 对应传感器平面，RGB = 归一化方向向量
- **算法**: 对每个纹理坐标执行 Mei 逆投影

### 4.2 rayExitPositionTexture（光线方向 → NDC）

- **格式**: RG 32-bit float EXR
- **尺寸**: 正方形（高分辨率以保证角度精度）
- **编码**: UV = 八面体编码的方向，RG = NDC 坐标
- **算法**: 对每个八面体方向执行 Mei 正向投影

### 4.3 八面体编码

NVIDIA 使用八面体编码将 3D 单位球面方向映射到 2D 纹理的 [-1,1]² 空间。这种编码比等距柱状投影（equirectangular）更均匀，避免极点畸变。

## 5. Y 轴约定

NVIDIA RTX 相机空间 Y 轴朝上，但纹理 V 坐标（行方向）朝下。`tools/cameras/oak_generate_lut_textures.py` 中对 dirY 取反来匹配这一约定。这是一个需要仔细验证的关键点。

## 6. opticalCenter 陷阱

**关键**：由于 EXR 纹理生成时已经将真实的 cx/cy 烘焙进去了，所以在 Isaac Sim 中设置 `opticalCenter` 时**必须设为几何中心** `(width/2, height/2)` 以避免双重偏移。

或者反过来：如果纹理假设 cx/cy 在几何中心，则 Isaac Sim 的 opticalCenter 应填真实的 cx/cy。

当前 `tools/cameras/oak_generate_lut_textures.py` 使用真实 cx/cy 生成纹理 → Isaac Sim 中 opticalCenter 应设为几何中心。
但 `tools/cameras/oak_test_lut.py` 中设置了 `optical_center=(float(CX), float(CY))`，这可能是**双重偏移**的 bug。

## 7. 执行步骤

### Step 1: 验证数学正确性
运行 `tools/cameras/oak_generate_lut_textures.py` 中的 roundtrip 验证：像素 → 3D 方向 → 像素，检查重投影误差。

### Step 2: 生成 LUT 纹理
运行 `tools/cameras/oak_generate_lut_textures.py` 生成 4 个相机的 EXR 纹理对。

### Step 3: 验证纹理内容
编写脚本检查 EXR 文件中的值是否合理（方向向量归一化、NDC 范围 [0,1] 等）。

### Step 4: 修复 test_cam_a_lut.py 的 opticalCenter 问题
将 `optical_center=(float(CX), float(CY))` 改为 `optical_center=(float(WIDTH)/2, float(HEIGHT)/2)`。

### Step 5: 在 Isaac Sim 中测试
运行 `tools/cameras/oak_test_lut.py` 渲染测试场景，检查 LUT 相机输出是否有合理的鱼眼畸变效果。

### Step 6: 结果分析
比较 LUT 相机和 Pinhole 相机的渲染输出，确认畸变效果正确。

## 8. 关键发现：坐标系转换

### 8.1 坐标系差异

| 坐标系 | X | Y | Z |
|--------|---|---|---|
| OpenCV / Mei 模型 | 右 (Right) | 下 (Down) | 前 (Forward, 光轴) |
| NVIDIA RTX 相机空间 | 右 (Right) | 上 (Up) | 后 (Backward) |

**转换公式**：OpenCV(X, Y, Z) → RTX(X, -Y, -Z)

### 8.2 Enter texture（像素→方向）

Mei unproject 产生 OpenCV 空间方向后，需同时翻转 Y 和 Z：
```python
dirY = -dirY  # Y-down → Y-up
dirZ = -dirZ  # Z-forward → Z-backward
```

### 8.3 Exit texture（方向→像素）

八面体编码的方向在 RTX 空间，需转回 OpenCV 空间再做 Mei forward projection：
```python
dirX_cv = dirX       # X 不变
dirY_cv = -dirY      # Y-up → Y-down
dirZ_cv = -dirZ      # Z-backward → Z-forward
```

### 8.4 之前只翻转 Y 的错误

之前只做 `dirY = -dirY` 而没翻转 Z，导致中心像素的方向指向 RTX 的 +Z（背向场景），渲染出的图像几乎全黑。

## 9. 关键发现：OmniLensDistortionLutAPI 不生效

### 9.1 问题

在 Isaac Sim 5.1 的 headless 模式下，新版 `OmniLensDistortionLutAPI`（通过 `camera.set_lut_properties()` 设置）**不生效**。使用该 API 后，LUT 相机渲染结果与普通 Pinhole 完全相同。

### 9.2 验证过程

1. 确认 `set_opencv_fisheye_properties()` 等其他新 API **可以**正常生效
2. 确认 LUT 纹理文件存在且内容正确
3. 确认 API 调用成功，属性值已设置
4. 对比 LUT vs 无 LUT 渲染输出完全一致

### 9.3 解决方案：使用旧式 deprecated API

RTX 渲染器仍然支持旧式的 `cameraProjectionType` 属性：

```python
from pxr import Sdf

prim = camera.prim
prim.CreateAttribute("cameraProjectionType", Sdf.ValueTypeNames.Token, False).Set("generalizedProjection")
prim.CreateAttribute("fthetaWidth", Sdf.ValueTypeNames.Float, False).Set(float(WIDTH))
prim.CreateAttribute("fthetaHeight", Sdf.ValueTypeNames.Float, False).Set(float(HEIGHT))
prim.CreateAttribute("fthetaCx", Sdf.ValueTypeNames.Float, False).Set(float(WIDTH) / 2.0)
prim.CreateAttribute("fthetaCy", Sdf.ValueTypeNames.Float, False).Set(float(HEIGHT) / 2.0)
prim.CreateAttribute("generalizedProjectionDirectionTexturePath", Sdf.ValueTypeNames.Asset, False).Set(
    Sdf.AssetPath("/absolute/path/to/rayEnterDirection.exr"))
prim.CreateAttribute("generalizedProjectionNDCTexturePath", Sdf.ValueTypeNames.Asset, False).Set(
    Sdf.AssetPath("/absolute/path/to/rayExitPosition.exr"))
```

### 9.4 旧式 API 参数说明

| 属性 | 说明 |
|------|------|
| `cameraProjectionType` | 设为 `"generalizedProjection"` 启用 LUT 模式 |
| `fthetaWidth/Height` | 传感器像素分辨率 |
| `fthetaCx/Cy` | 光心位置（设为几何中心，因为真实 cx/cy 已烘焙入 LUT） |
| `generalizedProjectionDirectionTexturePath` | rayEnterDirection EXR 纹理的**绝对路径** |
| `generalizedProjectionNDCTexturePath` | rayExitPosition EXR 纹理的**绝对路径** |

## 10. Base Camera FOV 设置

LUT 相机的 base UsdGeomCamera 需要足够宽的 FOV（170°），否则 RTX 光线追踪无法覆盖 Mei 模型的超广角范围：

```python
import math
lut_fov_deg = 170.0
focal_length = pixel_size_m * (FX + FY) / 2
lut_aperture_h = 2 * focal_length * math.tan(math.radians(lut_fov_deg / 2))
lut_aperture_v = lut_aperture_h * HEIGHT / WIDTH
camera.set_horizontal_aperture(lut_aperture_h)
camera.set_vertical_aperture(lut_aperture_v)
```

## 11. 最终测试结果

LUT 鱼眼效果在 cubes 和 interior 场景中均成功验证：
- 明显的桶形畸变效果 ✓
- 超广角 FOV ✓
- 圆形有效区域（典型鱼眼特征）✓
- 所有场景物体可见且颜色正确 ✓

## 12. 注意事项

- **tex_scale**: 当前用 tex_scale=1（与传感器分辨率相同）。可增大到 2 提升精度。
- **undistort_iters**: 迭代去畸变需要足够多的迭代次数（当前 50 次应该足够）。
- **discriminant < 0**: 对于 xi > 1 的情况，某些像素（边缘区域 r² > 1/(xi²-1)）落在 Mei 投影的有效域之外，需要标记为无效。
- **base camera FOV**: LUT 相机需要设置足够宽的 FOV（≥170°），以便覆盖 LUT 的完整重映射范围。
- **纹理路径**: 必须使用**绝对路径**，否则 RTX 渲染器可能找不到 EXR 文件。
- **opticalCenter / fthetaCx/Cy**: 由于 cx/cy 已烘焙入 EXR 纹理，这些参数应设为图像几何中心 (width/2, height/2)，避免双重偏移。
