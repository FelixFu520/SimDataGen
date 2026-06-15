#!/usr/bin/env python3
"""把指定 USD 文件中所有已有 MeshCollisionAPI 的 Mesh 的
physics:approximation 批量改成目标值(默认 convexHull),用于在不改动
碰撞拓扑结构的前提下,显著降低 GPU PhysX cooking 的显存占用。

背景:
- approximation="none" 表示精确三角网格碰撞(triangle mesh)。对包含
  数千个 mesh / 数千万顶点、且大量使用 PointInstancer 的大场景(如
  AsianVillage),GPU 动力学会为其 cook 海量碰撞数据,极易爆显存。
- 改成 convexHull(凸包)或 boundingCube 等近似,可把碰撞显存降一个
  数量级,对静态环境的物理交互通常足够。

只修改"已经具有 MeshCollisionAPI 的 mesh"的 approximation 属性,不会
新增/删除碰撞,也不会触碰几何与材质。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/set_collision_approximation.py \
        --usd_path assets_extern/USD/AsianVillage/Asian_Village.usd \
        --approximation convexHull
"""
import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics

VALID = {
    "none",
    "convexHull",
    "convexDecomposition",
    "meshSimplification",
    "boundingCube",
    "boundingSphere",
}


def run(usd_path: str, approximation: str, dry_run: bool) -> int:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[ERROR] 打开失败: {usd_path}", flush=True)
        return 1

    before: dict = {}
    changed = 0
    total_meshcoll = 0

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            continue
        total_meshcoll += 1
        mc = UsdPhysics.MeshCollisionAPI(prim)
        attr = mc.GetApproximationAttr()
        cur = attr.Get() if attr else None
        before[str(cur)] = before.get(str(cur), 0) + 1

        if str(cur) == approximation:
            continue
        if dry_run:
            changed += 1
            continue
        if not attr:
            attr = mc.CreateApproximationAttr()
        attr.Set(approximation)
        changed += 1

    print("=" * 60, flush=True)
    print(f"文件: {usd_path}", flush=True)
    print(f"  含 MeshCollisionAPI 的 mesh: {total_meshcoll}", flush=True)
    print(f"  原 approximation 分布: {before}", flush=True)
    print(f"  目标 approximation: {approximation}", flush=True)
    print(f"  {'将要修改' if dry_run else '已修改'}: {changed} 个", flush=True)

    if dry_run:
        print("  [DRY-RUN] 未写入。去掉 --dry-run 即实际保存。", flush=True)
        return 0

    if changed > 0:
        stage.GetRootLayer().Save()
        print(f"  [SAVED] 已保存到 {usd_path}", flush=True)
    else:
        print("  无需修改,未保存。", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd_path", required=True)
    parser.add_argument("--approximation", default="convexHull", choices=sorted(VALID))
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    return run(args.usd_path, args.approximation, args.dry_run)


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    raise SystemExit(rc)
