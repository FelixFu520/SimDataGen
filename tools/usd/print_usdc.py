"""Dump 相机组 USD 结构（独立运行，需要 isaacsim 环境）"""
import argparse
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import sys
from pxr import Usd, UsdGeom, Sdf


def main():
    parser = argparse.ArgumentParser(description="Dump camera group USD structure.")
    parser.add_argument("--usd_path", type=str, default="/home/fufa/projects2026/DataGen/assets/cameras/oak_camera_4lut_2H30YA.usd", help="USD file path.")
    args = parser.parse_args()
    usd_path = args.usd_path
    stage = Usd.Stage.Open(usd_path)

    print("=" * 70, flush=True)
    print("DefaultPrim:", stage.GetDefaultPrim(), flush=True)
    print("=" * 70, flush=True)
    for prim in stage.Traverse():
        print(f"Path: {prim.GetPath()} | TypeName: {prim.GetTypeName()}", flush=True)
        for attr in prim.GetAttributes():
            if attr.HasAuthoredValue():
                try:
                    val = attr.Get()
                    sval = repr(val)
                    if len(sval) > 240:
                        sval = sval[:240] + " ...(truncated)"
                    print(f"   - {attr.GetName()} = {sval}", flush=True)
                except Exception as e:
                    print(f"   - {attr.GetName()} = <err {e}>", flush=True)
        for rel in prim.GetRelationships():
            try:
                targets = rel.GetTargets()
                if targets:
                    print(f"   * rel {rel.GetName()} -> {targets}", flush=True)
            except Exception:
                pass

    print("=" * 70, flush=True)
    print("FULL USDA:", flush=True)
    print(stage.GetRootLayer().ExportToString(), flush=True)


if __name__ == "__main__":
    try:
        main()
    finally:
        simulation_app.close()
