# Bug集合
## Bug 修复记录：free positions 铺成一大片巨大平面、与 occupied 错位

### 现象

在 AsianVillage 场景跑 `gen_data.py` 生成 occupancy 时：

- `workdir/AsianVillage/occupancy/occupied_positions.ply`（occupied，彩色）贴着村庄真实建筑/地形。
- `workdir/AsianVillage/path/filtered_free_positions.ply`（free，白色）却是一张又长又大、明显超出村庄范围的斜面平面。
- 二者在 3D 里叠加显示明显错位，大量 free point 没有任何 occupancy point 覆盖。

数据对比（修复前）：

| | X 范围 | Y 范围 | Z 范围 | Z 层数 |
|---|---|---|---|---|
| occupied | -98.5 ~ 140.5 | -190.5 ~ -47.5 | -45.5 ~ 6.5 | 53 层 |
| free | -98.5 ~ 140.5 | -190.5 ~ -47.5 | **0.5 ~ 5.5** | **仅 6 层** |

free 的 Z 只覆盖 0.5~5.5 一层薄片，而 occupied 覆盖 -45.5~6.5 整个地形，二者完全对不上。

运行日志里还能看到联合包围盒被撑爆：

```
统一网格联合包围盒: (-5000, -5000, ...) ~ (5000, 5000, ...)
```

XY 跨度达到 10000×10000 米，远超村庄真实尺寸。

### 根因

#### 1. 罪魁祸首是运行时注入的 GroundPlane

`sdg_utils/usd.py` 的 `load_usd_file()` 在加载场景时，额外创建了一个 Isaac Sim 的不可见地面：

```python
objects.GroundPlane("/World/ground_plane", visible=False)
```

这个 `GroundPlane` 是一个 **10000×10000 米、带物理碰撞的 Mesh**（世界 AABB 为 `(-5000,-5000,0) ~ (5000,5000,0)`），本意是给物理仿真兜底（防止物体掉下去）。

它**不在 USD 文件里**，而是运行时注入的。所以：
- 用 `Usd.Stage.Open()` 静态打开原始 USD 时看不到它，诊断出来的联合包围盒是正常的 ~324×143×52 米；
- 但在 `gen_data.py` 实际运行流程里（`load_usd_file` + `world.reset()` 之后），它存在于场景中。

#### 2. 它通过两条路污染 occupancy

`get_semantic_occupancy()` 里，occupied 和 free 都用 `_omap.Generator(physx, stage_id)` 生成。关键点：

> **`_omap.Generator` 是基于整个 PhysX 物理场景做体素扫描的，不是针对某个具体 mesh。** 传入的 mesh path 只用来决定"扫描哪个包围盒范围"和"给 occupied 体素贴 semantic_id"，并不能阻止 PhysX 场景里其它碰撞体（如 GroundPlane）参与该范围内的 free/occupied 判定。

于是 GroundPlane 通过两条路造成问题：

- **撑大联合包围盒**：free 的联合包围盒 `all_w_min/all_w_max` 是遍历带碰撞 mesh 累加出来的，GroundPlane 把它撑成 ±5000。
- **干扰体素扫描**：即使范围正常，`generate3d()` 在扫描时，z≈0 那一层仍会被 GroundPlane 的碰撞判成 occupied，污染 free 判定。

#### 3. 与碰撞处理工具无关

本问题与 `tools/usd/remove_collision_by_match.py`、`tools/usd/set_collision_approximation.py` 这些对场景 USD 的碰撞处理**无关**，纯粹是运行期注入的 GroundPlane 导致的。

### 为什么别的场景没暴露这个问题

GroundPlane 是 `load_usd_file()` 里**无条件注入**的，所以**每个场景都有**这个 ±5000、z≈0 的大地面。问题一直存在，只是是否"暴露成肉眼可见的错位 / 把包围盒撑爆"取决于场景自身的几何特征。AsianVillage 同时踩中了几个放大因素：

#### 1. 地形 Z 跨度极大（最关键）

AsianVillage 是海岸地形，occupied 的 Z 范围是 -45.5 ~ 6.5（跨度约 52 米），而 GroundPlane 在 z≈0，正好横穿在地形 Z 跨度的中间。

- 别的场景（室内、平地小场景）地形几乎都在 z≈0 一薄层，GroundPlane 与真实地面**几乎重合**，free 本来就该在那个高度，错位看不出来。
- AsianVillage 地形上下跨了 52 米，z=0 的大平面横穿地形中间，free 被它截成贴着 z=0 的一层薄片（修复前 free 仅 0.5~5.5），与真实地形脱节，错位肉眼可见。

#### 2. XY 范围本身就大且开阔

村庄 XY 跨度约 238×143 米，与 GroundPlane 的 10000×10000 差距约 40 倍，所以 free 包围盒被撑大后，那张大平面远超村庄轮廓，视觉上特别突兀。

#### 3. free 过滤步骤在别的场景里"恰好掩盖"了问题

`gen_path_3d` 里 free 会经过两道过滤（见 `sdg_utils/trajectory.py`）：

- `filter_free_by_obstacle_dilation`：剔除贴近障碍的 free；
- `filter_free_by_obstacle_envelope`：只保留落在"障碍外接 shape"内的 free。

在**封闭/紧凑**的场景里，envelope 过滤会把 GroundPlane 大平面那些远离真实建筑的 free 点剔掉（落在外接 shape 之外），问题被自动掩盖。但 AsianVillage 是**开阔的户外场景**（没有封闭外壳），envelope 过滤会"漏气"退化并自动回退（见 `filter_free_by_obstacle_envelope` 中点数为 0 时的回退逻辑），既保不住也滤不掉那张大平面，问题暴露。

### 小结

| 因素 | 别的场景 | AsianVillage |
|---|---|---|
| 地形 Z 跨度 | 小（z≈0 一薄层，GroundPlane 与真实地面重合） | 大（-45~6，GroundPlane 横穿地形中间） |
| XY 范围 | 小，比例差异不突出 | 大且开阔，大平面远超村庄轮廓 |
| 是否封闭 | 室内/紧凑 → envelope 过滤掩盖了大平面 | 开阔户外 → envelope 退化，掩盖不住 |

一句话：不是别的场景没这个 bug，而是 AsianVillage 的"大 Z 跨度 + 大开阔户外地形"把这个一直存在的隐患放大成了肉眼可见的错位。

### 为什么"只在 get_mesh_paths 里排除 GroundPlane"治标不治本

曾尝试在 `get_mesh_paths()` 里按路径前缀跳过 `/World/ground_plane`：这只挡住了"撑大包围盒"这条路（包围盒恢复正常），但 **GroundPlane 仍然在 PhysX 物理场景里带着碰撞**，`generate3d()` 扫描村庄范围时 z≈0 那层依旧被它干扰，错位问题复现。

结论：**GroundPlane 必须从物理层真正移除，光从 mesh 列表里剔除没用。**

### 解决方案

在 `sdg_utils/usd.py` 的 `load_usd_file()` 中，不再创建 GroundPlane（注释掉该行）：

```python
def load_usd_file(usd_file_path: str) -> tuple[World, Usd.Stage]:
    """加载USD文件, 并返回World和Stage"""
    assert os.path.exists(usd_file_path), f"场景文件不存在: {usd_file_path}"
    open_stage(usd_file_path)
    stage = get_current_stage()
    # 添加地面平面（不可见）
    # objects.GroundPlane("/World/ground_plane", visible=False)
    world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0, rendering_dt=1.0/60.0)
    return world, stage
```

该场景本身有地形（coast_land_rocks 等带碰撞 mesh），且相机是按预生成的路径点直接摆位、不依赖物理掉落兜底，因此去掉 GroundPlane 是安全的。

### 验证结果（修复后）

| | X 范围 | Y 范围 | Z 范围 |
|---|---|---|---|
| occupied | -98.5 ~ 140.5 | -190.5 ~ -47.5 | -45.5 ~ 6.5 |
| free | -98.5 ~ 140.5 | -190.5 ~ -47.5 | **-45.5 ~ 5.5** |

- 联合包围盒从 ±5000 恢复为村庄真实尺寸（约 238×143×52 米）。
- free 的 Z 范围从只有 0.5~5.5 的一层薄片，变成 -45.5~5.5 完整贴合整个地形，与 occupied 一致，错位消失。
- 运行时识别到的带碰撞 mesh 从（被 GroundPlane 干扰时的异常情况）恢复为场景真实的 1989 个。

### 排查辅助脚本

- `tools/usd/diagnose_collision_bbox.py`：静态 `Usd.Stage.Open` 打开 USD，统计带碰撞 mesh 的世界 AABB 分布（看 USD 文件自身有没有超大 mesh）。
- `tools/usd/diagnose_runtime_bbox.py`：复现 `load_usd_file + world.reset()` 运行时流程，揪出运行时才出现的离群超大 mesh（本 bug 中即定位到 `/World/ground_plane/geom`）。

两者对比可快速区分"USD 文件自带的几何问题"与"运行时注入的 prim 问题"。
