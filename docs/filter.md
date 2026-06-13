# 数据过滤工具 (tools/filter)

用于过滤采集数据中不可用的轨迹，并对结果做抽帧 / 拼视频以便人工核对。

相关脚本：

- `tools/filter/filter_trajectories.py` —— 过滤不可用轨迹，输出保留 / 丢弃名单
- `tools/filter/sample_trajectories.py` —— 从名单中按轨迹随机抽帧，可选拼成视频

运行环境：使用 `volc` conda 环境（已安装 numpy / Pillow / opencv）。

```bash
PY=/root/miniconda3/envs/volc/bin/python
```

## 数据组织

```
<root>/<场景文件夹>/rgb/<相机>/<轨迹序号>_<帧号>.jpg
```

- `<轨迹序号>`（xxxx）：一条轨迹的编号
- `<帧号>`（yyyy）：该轨迹中某个采集点

例如 `workdir_taobao08_01/taobao08_01_101_KitchenSet008_100_10_60/rgb/CAM_A/0000_0007.jpg`。

---

## 1. 过滤轨迹：filter_trajectories.py

把"色彩单一、色差小、大部分为白 / 黑"的不可用轨迹找出来，按轨迹粒度划分为保留 / 丢弃两份名单。

### 判定逻辑

对每张图，**只统计鱼眼有效圆形区域**（去掉四角固定背景），计算对比度（灰度标准差）、平均饱和度、近白 / 近黑像素占比。

单张图满足以下任一情况，记为"坏图"（色彩单一 / 信息量低）：

1. 近黑占比 `>= black-hard`（硬阈值，默认 0.60）—— 纯黑死区占主导，直接判坏。**不要求**低对比 / 低饱和，避免"大面积纯黑 + 一道过曝高对比 / 带色条带"把对比度 / 饱和度抬高从而绕过判定（如黑墙 + 一道强光缝隙的空洞画面）
2. 低对比 **且** 低饱和 —— 灰蒙蒙、单色一片
3. 大面积近白 **且** 低饱和 —— 基本全白
4. 大面积近黑（`>= black-ratio`）**且** 低对比 **且** 低饱和 —— 偏黑且无内容（加低饱和约束，避免误杀"暗调但有暖光 / 丰富色彩内容"的图）
5. 近黑 + 近白占比极高 **且** 低饱和 —— 黑白两极化、几乎无中间调与色彩（如黑底背景 + 过曝纯白植被 / 物体，细节丢失）

一条轨迹中坏图比例 `>= bad-ratio`（默认 0.6）则判为不可用 → 丢弃，否则保留。

另外，一条轨迹中点（帧）数量 `< min-points`（默认 30）也直接判为不可用 → 丢弃（轨迹太短、采集点过少）。

> 注意：按整条轨迹的比例判定，而非单帧。这样"只是起点贴墙偏白、后段正常"的轨迹会被正确保留。

### 用法

```bash
$PY tools/filter/filter_trajectories.py \
    --root workdir_taobao08_01 \
    --out-dir workdir_filter \
    --cameras CAM_A \
    --workers 32
```

### 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--root` | `workdir_taobao08_01` | 数据根目录 |
| `--out-dir` | `tools/filter` | 输出 csv 目录 |
| `--cameras` | `CAM_A` | 参与统计的相机，留空表示该轨迹下所有相机合并统计 |
| `--bad-ratio` | `0.6` | 轨迹丢弃阈值，坏图比例 `>=` 该值则丢弃 |
| `--min-points` | `40` | 轨迹点（帧）数量阈值，`<` 该值则整条轨迹丢弃 |
| `--std-thresh` | `20.0` | 对比度阈值 |
| `--sat-thresh` | `8.0` | 饱和度阈值 |
| `--white-ratio` | `0.55` | 近白占比阈值 |
| `--black-ratio` | `0.55` | 近黑占比阈值（配合低对比 / 低饱和判坏） |
| `--black-hard` | `0.60` | 近黑占比硬阈值，`>=` 则直接判坏（无需低对比 / 低饱和） |
| `--bw-ratio` | `0.85` | 近黑+近白占比阈值（黑白两极化），`>=` 且低饱和则判坏 |
| `--size` | `128` | 缩放后边长（加速统计） |
| `--workers` | `16` | 并行进程数 |

### 输出（均在 `--out-dir` 下）

- `<root目录名>.csv` —— 每条轨迹一行，列：`scene,traj_id,total,bad,bad_ratio,decision,reason`
  - 例如 `--root workdir_taobao08_01` 时输出 `workdir_taobao08_01.csv`
  - `decision` 为 `save`（保留）或 `discard`（丢弃）
  - `reason` 为丢弃原因：`few_points`（点数过少）/ `bad_ratio`（坏图过多），保留时为空

调参方向：想过滤更严，可调高 `--std-thresh` / `--sat-thresh`、调低 `--black-hard` 或调低 `--bad-ratio`；想更宽松则反之。其中 `--black-hard` 专门针对"大面积纯黑死区 + 少量过曝亮带"这类空洞画面（这类图对比度 / 饱和度会被亮带抬高，靠 `--std-thresh` / `--sat-thresh` 抓不到，需要靠近黑硬阈值）。

---

## 2. 抽帧 / 拼视频：sample_trajectories.py

从过滤结果 csv（或 txt）中的轨迹按帧号顺序每隔 `--step` 帧抽帧，把**每条轨迹抽出的若干帧拼接成一张网格大图**（一条轨迹一张），底部标注"场景文件夹名 + 轨迹序号 + 帧数"；可选地把**所有轨迹的拼接图合并到一个视频**，视频每一帧即一条轨迹的拼接图。

### 用法

每条轨迹拼一张大图（每隔 15 帧取一帧）：

```bash
$PY tools/filter/sample_trajectories.py \
    --list workdir_filter/workdir_taobao08_01.csv \
    --decision discard \
    --root workdir_taobao08_01 \
    --out-dir workdir_filter/sample_discard \
    --step 15
```

拼接图 + 把所有轨迹的拼接图合并成一个视频（每帧 = 一条轨迹拼接图）：

```bash
$PY tools/filter/sample_trajectories.py \
    --list workdir_filter/workdir_taobao08_01.csv \
    --decision discard \
    --root workdir_taobao08_01 \
    --out-dir workdir_filter/sample_discard \
    --step 15 --video --fps 2
```

`--decision save` 可查看保留轨迹；也兼容旧的 txt 列表文件。

### 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--list` | 必填 | 过滤结果 csv 或 txt 列表文件 |
| `--decision` | 无 | 读取 csv 时按 `decision` 列筛选：`save` / `discard` |
| `--root` | `workdir_taobao08_01` | 数据根目录 |
| `--out-dir` | 必填 | 输出目录 |
| `--camera` | `CAM_A` | 使用的相机 |
| `--step` | `10` | 按帧号顺序每隔多少帧取一帧 |
| `--cell` | `320` | 拼接图中每个单元格的边长（像素） |
| `--cols` | `0` | 拼接网格列数，`0` 表示自动取接近正方形 |
| `--video` | 关 | 是否把所有轨迹的拼接图合并到一个视频 |
| `--fps` | `2.0` | 视频帧率 |
| `--video-dir` | `<out-dir>/videos` | 视频输出目录 |
| `--video-name` | `all.mp4` | 合并视频的文件名 |

### 输出（均在 `--out-dir` 下）

- `images/` —— 每条轨迹一张拼接大图，命名 `<场景文件夹名>_<轨迹序号>.jpg`，按 `--step` 抽出的帧排成网格，底部半透明黑底白字标注场景名、轨迹序号与帧数
- `videos/` —— 一个合并视频（默认 `all.mp4`），把**所有轨迹的拼接图**依次作为视频帧（`--video` 时输出）

> 视频每一帧就是 `images/` 里的一张拼接图（一条轨迹一张，含该轨迹按 `--step` 抽出的全部帧），依次播放便于一次性快速过目所有轨迹。

---

## 典型流程

```bash
PY=/root/miniconda3/envs/volc/bin/python

# 1) 过滤轨迹，生成 csv
$PY tools/filter/filter_trajectories.py \
    --root workdir_taobao08_01 --out-dir workdir_filter \
    --cameras CAM_A --workers 32

# 2) 对丢弃轨迹拼接图 + 拼视频，人工核对是否误杀
$PY tools/filter/sample_trajectories.py \
    --list workdir_filter/workdir_taobao08_01.csv --decision discard \
    --root workdir_taobao08_01 \
    --out-dir workdir_filter/sample_discard --step 15 --video

# 3) 对保留轨迹同样拼接图 + 拼视频，抽查质量
$PY tools/filter/sample_trajectories.py \
    --list workdir_filter/workdir_taobao08_01.csv --decision save \
    --root workdir_taobao08_01 \
    --out-dir workdir_filter/sample_save --step 15 --video
```
