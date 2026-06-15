"""只 dump 根 layer 自身的结构(不展开引用/不打印几何点),用于看装配关系。

用法:
    ./app/python.sh tools/usd/_dump_structure.py --usd_path <file> [--max N]
"""
import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, UsdPhysics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--usd_path", required=True)
    parser.add_argument("--max", type=int, default=20)
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.usd_path)
    print("DefaultPrim:", stage.GetDefaultPrim(), flush=True)
    print("metersPerUnit:", UsdGeom.GetStageMetersPerUnit(stage), flush=True)

    root_layer = stage.GetRootLayer()
    print("=" * 60, flush=True)
    print("ROOT LAYER 自身定义的 prim(只看本层,不含引用展开):", flush=True)

    shown = 0
    for prim in stage.Traverse():
        spec = root_layer.GetPrimAtPath(prim.GetPath())
        if spec is None:
            continue  # 该 prim 不是在本层定义/over 的
        has_coll = prim.HasAPI(UsdPhysics.CollisionAPI)
        refs = prim.GetMetadata("references")
        line = f"[{spec.specifier}] {prim.GetPath()} type={prim.GetTypeName()} coll={has_coll}"
        if refs:
            line += "  <-REF"
        print(line, flush=True)
        shown += 1
        if shown >= args.max:
            print(f"... (只显示前 {args.max} 个本层 spec)", flush=True)
            break


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
