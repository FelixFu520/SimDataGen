#!/usr/bin/env python3
"""诊断 USD 场景里带物理碰撞的 mesh 的世界 AABB 尺寸分布。

背景:
- occupancy 只对带物理碰撞 (CollisionAPI + MeshCollisionAPI) 的 mesh 计算,
  且会把所有这类 mesh 的世界 AABB 求并集, 作为统一体素网格的范围。
- 如果其中混入了一个超大尺寸的平面 / 地面 / 水面 mesh (例如世界 AABB 横跨
  ±5000 米), 联合包围盒会被它撑爆, 在固定 resolution 下体素数量爆炸, 导致
  generate3d OOM / 卡死, 同时 free 与 occupied 错位。

本脚本遍历所有带碰撞的 mesh, 计算每个 mesh 的世界 AABB (跨度 / 体积),
按"最大轴跨度"降序打印 Top N, 帮助定位那个异常大的 mesh path。

注意: 本脚本用 Usd.Stage.Open 静态打开 USD, 看到的是 **USD 文件自身的几何**,
不包含运行时注入的 prim (例如 load_usd_file 注入的 GroundPlane)。若要排查运行时
才出现的超大 mesh, 用 diagnose_runtime_bbox.py。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/diagnose_collision_bbox.py \
        --usd_path assets_extern/TaoBao11_fix/AsianVillage/Asian_Village.usd \
        --top 30
"""
import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics


def run(usd_path: str, top: int) -> int:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[ERROR] 打开失败: {usd_path}", flush=True)
        return 1

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    rows = []  # (max_extent, ext_x, ext_y, ext_z, w_min, w_max, path_str)
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.HasAPI(UsdPhysics.MeshCollisionAPI)):
            continue

        rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        w_min = rng.GetMin()
        w_max = rng.GetMax()
        ext_x = float(w_max[0] - w_min[0])
        ext_y = float(w_max[1] - w_min[1])
        ext_z = float(w_max[2] - w_min[2])
        max_ext = max(ext_x, ext_y, ext_z)
        rows.append((max_ext, ext_x, ext_y, ext_z,
                     (float(w_min[0]), float(w_min[1]), float(w_min[2])),
                     (float(w_max[0]), float(w_max[1]), float(w_max[2])),
                     str(prim.GetPath())))

    if not rows:
        print("未找到任何带物理碰撞的 mesh。", flush=True)
        return 0

    rows.sort(key=lambda r: r[0], reverse=True)

    max_exts = np.array([r[0] for r in rows], dtype=np.float64)
    # 联合包围盒(所有带碰撞 mesh 的并集), 即 occupancy 统一网格实际会用的范围
    all_min = np.array([r[4] for r in rows], dtype=np.float64).min(axis=0)
    all_max = np.array([r[5] for r in rows], dtype=np.float64).max(axis=0)

    print("=" * 78, flush=True)
    print(f"文件: {usd_path}", flush=True)
    print(f"带物理碰撞的 mesh 总数: {len(rows)}", flush=True)
    print(f"最大轴跨度 统计(米): "
          f"min={max_exts.min():.2f}, median={np.median(max_exts):.2f}, "
          f"p95={np.percentile(max_exts, 95):.2f}, max={max_exts.max():.2f}", flush=True)
    print(f"联合包围盒(并集) world min: ({all_min[0]:.2f}, {all_min[1]:.2f}, {all_min[2]:.2f})", flush=True)
    print(f"联合包围盒(并集) world max: ({all_max[0]:.2f}, {all_max[1]:.2f}, {all_max[2]:.2f})", flush=True)
    union_ext = all_max - all_min
    print(f"联合包围盒 跨度(米): x={union_ext[0]:.2f}, y={union_ext[1]:.2f}, z={union_ext[2]:.2f}", flush=True)
    print("=" * 78, flush=True)

    median = float(np.median(max_exts))
    print(f"[Top {top}] 按最大轴跨度降序 (标注 >10x 中位数 的疑似超大 mesh):", flush=True)
    for r in rows[:top]:
        max_ext, ex, ey, ez, wmin, wmax, path = r
        flag = "  <== 疑似超大(离群)" if (median > 0 and max_ext > 10 * median) else ""
        print(f"  max_ext={max_ext:10.2f}  (x={ex:9.2f} y={ey:9.2f} z={ez:9.2f})  "
              f"min=({wmin[0]:.1f},{wmin[1]:.1f},{wmin[2]:.1f}) "
              f"max=({wmax[0]:.1f},{wmax[1]:.1f},{wmax[2]:.1f})  {path}{flag}", flush=True)

    return 0


def main():
    parser = argparse.ArgumentParser(description="诊断带物理碰撞 mesh 的世界 AABB 尺寸分布")
    parser.add_argument("--usd_path", required=True, help="目标 USD 文件路径")
    parser.add_argument("--top", type=int, default=30, help="打印最大的前 N 个 mesh")
    args = parser.parse_args()
    return run(args.usd_path, args.top)


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    raise SystemExit(rc)
