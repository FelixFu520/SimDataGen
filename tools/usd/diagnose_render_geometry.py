#!/usr/bin/env python3
"""静态扫描 USD 场景里的可渲染 mesh, 找出会让 RTX 渲染器(librtx.scenedb)崩溃的可疑几何。

背景:
- gen_data.py 第一次 `world.step(render=True)` 时, Isaac Sim 的 RTX 渲染器会为场景
  里所有可见 mesh 构建几何加速结构(BVH/scenedb)。
- 如果某些 mesh 含有非法/退化几何, RTX 在 C++ 层构建加速结构时会段错误崩溃
  (典型堆栈: librtx.scenedb.plugin.so 里 std::vector::_M_realloc_insert 反复递归),
  Python 侧只能看到 world.step 处崩溃, 拿不到具体是哪个 mesh。
- 本脚本静态遍历 USD, 逐 mesh 检查下面这些 RTX 常见崩溃诱因, 把可疑 mesh 列出来。

检查项:
  1. NaN / Inf 顶点坐标
  2. 顶点坐标量级异常大(疑似坐标溢出 / 单位错误)
  3. 空 mesh(无点 / 无面)
  4. faceVertexIndices 越界(索引 >= 点数, 会直接读越界内存)
  5. faceVertexCounts 与 indices 数量对不上
  6. 含 < 3 的面(退化面)
  7. 单个 mesh 三角面数量异常巨大(可能拖垮 BVH 构建)
  8. prim 层级深度异常(深层嵌套, 与崩溃栈里的深递归吻合)

注意: 静态打开看到的是 USD 文件自身几何, 不含运行时注入的 prim。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/diagnose_render_geometry.py \
        --usd_path assets_extern/TaoBao11_fix2/ForestHourse/Map_Houses_A.usd \
        --top 50
"""
import argparse
import math

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
from pxr import Usd, UsdGeom


# 顶点坐标(局部坐标系)绝对值超过该阈值视为"量级异常"(米)
LARGE_COORD_THRESHOLD = 1.0e6
# 单 mesh 三角面数超过该阈值视为"超重 mesh"
HEAVY_TRI_THRESHOLD = 5_000_000
# prim 路径深度(/ 分段数)超过该阈值视为"层级过深"
DEEP_PRIM_THRESHOLD = 40


def _get_points(mesh: UsdGeom.Mesh):
    attr = mesh.GetPointsAttr()
    if not attr or not attr.HasAuthoredValue():
        return None
    pts = attr.Get()
    if pts is None:
        return None
    return np.asarray(pts, dtype=np.float64)


def check_mesh(prim) -> list:
    """返回该 mesh 的问题列表(字符串). 空列表表示无可疑项。"""
    issues = []
    mesh = UsdGeom.Mesh(prim)

    pts = _get_points(mesh)
    counts_attr = mesh.GetFaceVertexCountsAttr()
    indices_attr = mesh.GetFaceVertexIndicesAttr()
    counts = counts_attr.Get() if counts_attr else None
    indices = indices_attr.Get() if indices_attr else None

    n_pts = 0 if pts is None else len(pts)
    n_counts = 0 if counts is None else len(counts)
    n_indices = 0 if indices is None else len(indices)

    # 1. 空 mesh
    if n_pts == 0:
        issues.append("空mesh(无顶点)")
    if n_counts == 0:
        issues.append("空mesh(无面)")

    # 2. NaN / Inf / 超大坐标
    if n_pts > 0:
        finite = np.isfinite(pts)
        if not finite.all():
            n_bad = int((~finite).any(axis=1).sum())
            issues.append(f"含NaN/Inf顶点x{n_bad}")
        else:
            max_abs = float(np.abs(pts).max())
            if max_abs > LARGE_COORD_THRESHOLD:
                issues.append(f"顶点坐标量级异常(max|v|={max_abs:.3e})")

    # 3. 索引越界
    if n_indices > 0 and n_pts > 0:
        idx = np.asarray(indices, dtype=np.int64)
        if idx.min() < 0:
            issues.append(f"faceVertexIndices含负值(min={int(idx.min())})")
        if idx.max() >= n_pts:
            issues.append(f"faceVertexIndices越界(max={int(idx.max())} >= 点数{n_pts})")

    # 4. counts 与 indices 总数对不上
    if n_counts > 0 and n_indices > 0:
        total = int(np.asarray(counts, dtype=np.int64).sum())
        if total != n_indices:
            issues.append(f"faceVertexCounts总和({total})!=indices数({n_indices})")

    # 5. 退化面(< 3 顶点)
    if n_counts > 0:
        c = np.asarray(counts, dtype=np.int64)
        n_degen = int((c < 3).sum())
        if n_degen > 0:
            issues.append(f"退化面(<3顶点)x{n_degen}")

    # 6. 超重 mesh(三角面数过大)
    if n_counts > 0:
        c = np.asarray(counts, dtype=np.int64)
        tri = int(np.clip(c - 2, 0, None).sum())
        if tri > HEAVY_TRI_THRESHOLD:
            issues.append(f"超重mesh(三角面≈{tri:,})")

    return issues


def run(usd_path: str, top: int) -> int:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[ERROR] 打开失败: {usd_path}", flush=True)
        return 1

    total_mesh = 0
    suspicious = []  # (path, issues)
    max_depth = 0
    deepest_path = ""

    for prim in stage.Traverse():
        depth = str(prim.GetPath()).count("/")
        if depth > max_depth:
            max_depth = depth
            deepest_path = str(prim.GetPath())

        if not prim.IsA(UsdGeom.Mesh):
            continue
        total_mesh += 1

        try:
            issues = check_mesh(prim)
        except Exception as e:  # noqa: BLE001
            issues = [f"检查抛异常: {type(e).__name__}: {e}"]

        if issues:
            suspicious.append((str(prim.GetPath()), issues))

    print("=" * 78, flush=True)
    print(f"文件: {usd_path}", flush=True)
    print(f"可渲染 mesh 总数: {total_mesh}", flush=True)
    print(f"最大 prim 路径深度: {max_depth}  ({deepest_path})", flush=True)
    if max_depth > DEEP_PRIM_THRESHOLD:
        print(f"  <== prim 层级过深(>{DEEP_PRIM_THRESHOLD}), 可能与 RTX 深递归崩溃相关", flush=True)
    print(f"可疑 mesh 数量: {len(suspicious)}", flush=True)
    print("=" * 78, flush=True)

    if not suspicious:
        print("未发现明显的几何问题。崩溃可能由材质/纹理/instancing 触发, 建议改用最小复现脚本逐步定位。", flush=True)
        return 0

    for path, issues in suspicious[:top]:
        print(f"  {path}", flush=True)
        for it in issues:
            print(f"      - {it}", flush=True)

    if len(suspicious) > top:
        print(f"  ... 还有 {len(suspicious) - top} 个未显示(增大 --top 查看)", flush=True)

    return 0


def main():
    parser = argparse.ArgumentParser(description="静态扫描可渲染 mesh, 找出会让 RTX 崩溃的可疑几何")
    parser.add_argument("--usd_path", required=True, help="目标 USD 文件路径")
    parser.add_argument("--top", type=int, default=50, help="最多打印前 N 个可疑 mesh")
    args = parser.parse_args()
    return run(args.usd_path, args.top)


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    raise SystemExit(rc)
