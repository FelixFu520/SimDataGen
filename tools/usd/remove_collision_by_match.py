#!/usr/bin/env python3
"""按模糊匹配删除 USD 中指定 mesh / prim 的碰撞属性。

根据 prim 路径(prim path)做大小写不敏感的子串模糊匹配,把命中的 prim 上的
物理碰撞相关 schema 删掉:
  - UsdPhysics.CollisionAPI
  - UsdPhysics.MeshCollisionAPI
同时移除这些 API 引入的属性(physics:collisionEnabled / physics:approximation
等),并打印出实际删掉了哪些 prim 及其属性。

典型场景:
- 某个大场景里有一部分 mesh(例如 beach 海滩)不需要做物理碰撞,只想删掉这一
  部分的碰撞,而保留场景里其它 mesh 的碰撞。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/remove_collision_by_match.py \
        --usd_path assets_extern/TaoBao11_fix/AsianVillage/Asian_Village.usd \
        --match beach

    # 先预览不写入:
    ./app/python.sh tools/usd/remove_collision_by_match.py \
        --usd_path assets_extern/TaoBao11_fix/AsianVillage/Asian_Village.usd \
        --match beach --dry-run

    # 多个关键词(命中任意一个即删除),并区分大小写:
    ./app/python.sh tools/usd/remove_collision_by_match.py \
        --usd_path .../Asian_Village.usd \
        --match Beach Sand --case-sensitive
"""
import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics

# 碰撞相关 schema 引入的属性名,删 API 时一并清理,避免残留孤立属性。
_COLLISION_ATTR_NAMES = (
    "physics:collisionEnabled",
    "physics:approximation",
    "physxCollision:contactOffset",
    "physxCollision:restOffset",
    "physxConvexHullCollision:hullVertexLimit",
    "physxConvexHullCollision:minThickness",
    "physxConvexDecompositionCollision:hullVertexLimit",
    "physxConvexDecompositionCollision:maxConvexHulls",
)


def _matches(prim_path: str, prim_name: str, keywords: list[str], case_sensitive: bool) -> bool:
    hay_path = prim_path if case_sensitive else prim_path.lower()
    hay_name = prim_name if case_sensitive else prim_name.lower()
    for kw in keywords:
        needle = kw if case_sensitive else kw.lower()
        if needle in hay_path or needle in hay_name:
            return True
    return False


def run(
    usd_path: str,
    keywords: list[str],
    case_sensitive: bool,
    meshes_only: bool,
    dry_run: bool,
) -> int:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[ERROR] 打开失败: {usd_path}", flush=True)
        return 1

    removed: list[dict] = []
    matched_no_collision = 0

    for prim in stage.Traverse():
        if meshes_only and not prim.IsA(UsdGeom.Mesh):
            continue

        path_str = str(prim.GetPath())
        if not _matches(path_str, prim.GetName(), keywords, case_sensitive):
            continue

        has_coll = prim.HasAPI(UsdPhysics.CollisionAPI)
        has_mesh_coll = prim.HasAPI(UsdPhysics.MeshCollisionAPI)
        if not has_coll and not has_mesh_coll:
            matched_no_collision += 1
            continue

        removed_apis: list[str] = []
        removed_attrs: list[str] = []

        if dry_run:
            if has_coll:
                removed_apis.append("CollisionAPI")
            if has_mesh_coll:
                removed_apis.append("MeshCollisionAPI")
            for name in _COLLISION_ATTR_NAMES:
                if prim.HasAttribute(name):
                    removed_attrs.append(name)
        else:
            if has_mesh_coll:
                if prim.RemoveAPI(UsdPhysics.MeshCollisionAPI):
                    removed_apis.append("MeshCollisionAPI")
            if has_coll:
                if prim.RemoveAPI(UsdPhysics.CollisionAPI):
                    removed_apis.append("CollisionAPI")
            for name in _COLLISION_ATTR_NAMES:
                if prim.HasAttribute(name):
                    if prim.RemoveProperty(name):
                        removed_attrs.append(name)

        removed.append(
            {
                "path": path_str,
                "apis": removed_apis,
                "attrs": removed_attrs,
            }
        )

    print("=" * 60, flush=True)
    print(f"文件: {usd_path}", flush=True)
    print(f"  匹配关键词: {keywords} (case_sensitive={case_sensitive}, meshes_only={meshes_only})", flush=True)
    print(f"  命中并{'将要删除' if dry_run else '已删除'}碰撞的 prim: {len(removed)} 个", flush=True)
    if matched_no_collision:
        print(f"  命中但本来就没有碰撞属性(跳过): {matched_no_collision} 个", flush=True)

    for item in removed:
        print(f"    - {item['path']}", flush=True)
        if item["apis"]:
            print(f"        删除 API : {', '.join(item['apis'])}", flush=True)
        if item["attrs"]:
            print(f"        删除属性 : {', '.join(item['attrs'])}", flush=True)

    if dry_run:
        print("  [DRY-RUN] 未写入。去掉 --dry-run 即实际保存。", flush=True)
        return 0

    if removed:
        stage.GetRootLayer().Save()
        print(f"  [SAVED] 已保存到 {usd_path}", flush=True)
    else:
        print("  无命中,未保存。", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description="按模糊匹配删除 USD 中指定 prim 的碰撞属性")
    parser.add_argument("--usd_path", required=True, help="目标 USD 文件路径")
    parser.add_argument(
        "--match",
        nargs="+",
        required=True,
        help="一个或多个关键词,对 prim 路径/名字做子串模糊匹配,命中任意一个即删除其碰撞",
    )
    parser.add_argument(
        "--case-sensitive",
        action="store_true",
        help="区分大小写匹配(默认不区分)",
    )
    parser.add_argument(
        "--all-prims",
        dest="meshes_only",
        action="store_false",
        default=True,
        help="对所有 prim 都做匹配(默认只匹配 Mesh)",
    )
    parser.add_argument("--dry-run", action="store_true", help="只预览不写入")
    args = parser.parse_args()
    return run(
        args.usd_path,
        args.match,
        args.case_sensitive,
        args.meshes_only,
        args.dry_run,
    )


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    raise SystemExit(rc)
