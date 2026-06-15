#!/usr/bin/env python3
"""在 gen_data 的真实运行时环境下诊断带碰撞 mesh 的世界 AABB, 揪出离群超大 mesh。

与 diagnose_collision_bbox.py(静态 Usd.Stage.Open) 的区别:
- 本脚本完全复现 gen_data 的加载流程: load_usd_file() + world.reset(),
  即包含运行时注入的 prim (例如 GroundPlane) 和物理初始化后的世界变换。
- 用来回答: 静态打开 USD 时联合包围盒正常(几百米), 但 gen_data 运行时却变成
  ±5000, 那个超大包围盒到底是哪个 prim 贡献的? 它在不在原始 USD 文件里?

判定逻辑:
- 逐个带碰撞 mesh 计算世界 AABB, 按最大轴跨度降序打印 Top N;
- 标注 XY 跨度 > --outlier_m (默认 1000m) 的离群 mesh;
- 对每个离群 mesh, 打印它在 stage 里的 prim 类型 / 是否由 reference/payload 引入 /
  其 prim path, 便于判断是 USD 文件自带还是运行时注入。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/diagnose_runtime_bbox.py \
        --scene_usd_url assets_extern/TaoBao11_fix/AsianVillage/Asian_Village.usd \
        --top 30 --outlier_m 1000
"""
import argparse

from isaacsim import SimulationApp

simulation_app = SimulationApp(launch_config={"headless": True})

from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.asset.gen.omap")
simulation_app.update()

import numpy as np
from pxr import UsdGeom, Usd, UsdPhysics

from sdg_utils.usd import load_usd_file


def _describe_prim(prim: Usd.Prim) -> str:
    """描述 prim 是否由 reference/payload 引入, 帮助判断来源(USD 文件 vs 运行时)。"""
    has_ref = False
    has_payload = False
    for spec in prim.GetPrimStack():
        try:
            if spec.referenceList and (
                list(spec.referenceList.prependedItems)
                + list(spec.referenceList.explicitItems)
                + list(spec.referenceList.appendedItems)
            ):
                has_ref = True
            if spec.payloadList and (
                list(spec.payloadList.prependedItems)
                + list(spec.payloadList.explicitItems)
                + list(spec.payloadList.appendedItems)
            ):
                has_payload = True
        except Exception:
            pass
    # 定义此 prim 的 layer(最强 spec 来源), 可看出是来自场景 usd 还是运行时内存层
    layers = [s.layer.identifier for s in prim.GetPrimStack() if s.layer]
    return f"type={prim.GetTypeName()} ref={has_ref} payload={has_payload} layers={layers}"


def run(scene_usd_url: str, top: int, outlier_m: float) -> int:
    world, stage = load_usd_file(scene_usd_url)
    world.reset()
    for _ in range(5):
        simulation_app.update()

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

    rows = []  # (max_ext, ex, ey, ez, wmin, wmax, prim)
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not (prim.HasAPI(UsdPhysics.CollisionAPI) and prim.HasAPI(UsdPhysics.MeshCollisionAPI)):
            continue

        rng = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
        w_min = rng.GetMin()
        w_max = rng.GetMax()
        ex = float(w_max[0] - w_min[0])
        ey = float(w_max[1] - w_min[1])
        ez = float(w_max[2] - w_min[2])
        rows.append((max(ex, ey, ez), ex, ey, ez,
                     (float(w_min[0]), float(w_min[1]), float(w_min[2])),
                     (float(w_max[0]), float(w_max[1]), float(w_max[2])),
                     prim))

    if not rows:
        print("未找到任何带物理碰撞的 mesh。", flush=True)
        return 0

    rows.sort(key=lambda r: r[0], reverse=True)
    max_exts = np.array([r[0] for r in rows])
    all_min = np.array([r[4] for r in rows]).min(axis=0)
    all_max = np.array([r[5] for r in rows]).max(axis=0)

    print("=" * 80, flush=True)
    print(f"[运行时] 场景: {scene_usd_url}", flush=True)
    print(f"带物理碰撞的 mesh 总数: {len(rows)}", flush=True)
    print(f"最大轴跨度(米): min={max_exts.min():.2f}, median={np.median(max_exts):.2f}, "
          f"p95={np.percentile(max_exts, 95):.2f}, max={max_exts.max():.2f}", flush=True)
    print(f"联合包围盒 world min: ({all_min[0]:.2f}, {all_min[1]:.2f}, {all_min[2]:.2f})", flush=True)
    print(f"联合包围盒 world max: ({all_max[0]:.2f}, {all_max[1]:.2f}, {all_max[2]:.2f})", flush=True)
    union = all_max - all_min
    print(f"联合包围盒 跨度(米): x={union[0]:.2f}, y={union[1]:.2f}, z={union[2]:.2f}", flush=True)
    print("=" * 80, flush=True)

    # 离群 mesh: x 或 y 跨度超过阈值
    outliers = [r for r in rows if r[1] > outlier_m or r[2] > outlier_m]
    print(f"[离群] XY 跨度 > {outlier_m:.0f}m 的 mesh: {len(outliers)} 个", flush=True)
    for r in outliers:
        _, ex, ey, ez, wmin, wmax, prim = r
        print(f"  >>> {prim.GetPath()}", flush=True)
        print(f"      跨度 x={ex:.2f} y={ey:.2f} z={ez:.2f}  "
              f"min=({wmin[0]:.1f},{wmin[1]:.1f},{wmin[2]:.1f}) "
              f"max=({wmax[0]:.1f},{wmax[1]:.1f},{wmax[2]:.1f})", flush=True)
        print(f"      {_describe_prim(prim)}", flush=True)

    print("-" * 80, flush=True)
    print(f"[Top {top}] 按最大轴跨度降序:", flush=True)
    for r in rows[:top]:
        max_ext, ex, ey, ez, wmin, wmax, prim = r
        print(f"  max_ext={max_ext:10.2f} (x={ex:9.2f} y={ey:9.2f} z={ez:9.2f})  {prim.GetPath()}", flush=True)

    return 0


def main():
    parser = argparse.ArgumentParser(description="运行时(load_usd_file+reset)诊断带碰撞 mesh 世界 AABB")
    parser.add_argument("--scene_usd_url", required=True, help="场景 USD 文件路径")
    parser.add_argument("--top", type=int, default=30)
    parser.add_argument("--outlier_m", type=float, default=1000.0, help="XY 跨度超过此值视为离群")
    args = parser.parse_args()
    return run(args.scene_usd_url, args.top, args.outlier_m)


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    raise SystemExit(rc)
