"""统计 USD 中 Mesh 的碰撞属性分布(需要 isaacsim 环境)。

用法:
    ./app/python.sh tools/usd/_check_collision.py --usd_path <文件1> [<文件2> ...]
"""
import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics


def check_one(usd_path: str) -> None:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[ERR] 打开失败: {usd_path}", flush=True)
        return

    mesh_count = 0
    coll_count = 0
    meshcoll_count = 0
    approx_counts: dict = {}
    total_points = 0

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh_count += 1

        pts_attr = UsdGeom.Mesh(prim).GetPointsAttr()
        pts = pts_attr.Get() if pts_attr else None
        if pts is not None:
            total_points += len(pts)

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            coll_count += 1
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            meshcoll_count += 1
            mc = UsdPhysics.MeshCollisionAPI(prim)
            a = mc.GetApproximationAttr()
            val = a.Get() if a else None
            approx_counts[str(val)] = approx_counts.get(str(val), 0) + 1

    print("=" * 60, flush=True)
    print(f"文件: {usd_path}", flush=True)
    print(f"  Mesh 总数        : {mesh_count}", flush=True)
    print(f"  顶点总数(累计)   : {total_points:,}", flush=True)
    print(f"  含 CollisionAPI  : {coll_count}", flush=True)
    print(f"  含 MeshCollision : {meshcoll_count}", flush=True)
    print(f"  approximation 分布: {approx_counts}", flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd_path", nargs="+", required=True)
    args = parser.parse_args()
    for p in args.usd_path:
        check_one(p)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
