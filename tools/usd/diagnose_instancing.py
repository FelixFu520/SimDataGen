#!/usr/bin/env python3
"""统计 USD 场景里的 instancing 规模(PointInstancer / scene-graph instance / 原型),
用于判断 RTX 渲染器是否因实例化几何过多/异常而在构建 scenedb 时崩溃。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/diagnose_instancing.py \
        --usd_path assets_extern/TaoBao11_fix2/ForestHourse/Map_Houses_A.usd \
        --top 30
"""
import argparse

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
from pxr import Usd, UsdGeom


def run(usd_path: str, top: int) -> int:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[ERROR] 打开失败: {usd_path}", flush=True)
        return 1

    n_point_instancer = 0
    n_scene_instance = 0       # prim.IsInstance() (scene-graph instancing)
    total_instances = 0        # PointInstancer 的实例总数
    pi_rows = []               # (count, path)
    bad_pi = []                # 实例数据异常的 PointInstancer

    for prim in stage.Traverse():
        if prim.IsInstance():
            n_scene_instance += 1

        if prim.IsA(UsdGeom.PointInstancer):
            n_point_instancer += 1
            pi = UsdGeom.PointInstancer(prim)

            proto_indices = pi.GetProtoIndicesAttr().Get()
            positions = pi.GetPositionsAttr().Get()
            proto_targets = pi.GetPrototypesRel().GetTargets()

            n_idx = 0 if proto_indices is None else len(proto_indices)
            n_pos = 0 if positions is None else len(positions)
            n_proto = 0 if proto_targets is None else len(proto_targets)
            total_instances += n_idx

            path = str(prim.GetPath())
            pi_rows.append((n_idx, path))

            problems = []
            if n_proto == 0:
                problems.append("无 prototype 目标")
            if n_idx > 0 and n_pos > 0 and n_idx != n_pos:
                problems.append(f"protoIndices数({n_idx})!=positions数({n_pos})")
            if n_idx > 0 and n_proto > 0:
                idx = np.asarray(proto_indices, dtype=np.int64)
                if idx.max() >= n_proto or idx.min() < 0:
                    problems.append(
                        f"protoIndices越界(范围[{int(idx.min())},{int(idx.max())}] vs 原型数{n_proto})")
            if positions is not None and n_pos > 0:
                pos = np.asarray(positions, dtype=np.float64)
                if not np.isfinite(pos).all():
                    problems.append("positions 含 NaN/Inf")
            if problems:
                bad_pi.append((path, n_idx, n_proto, problems))

    print("=" * 78, flush=True)
    print(f"文件: {usd_path}", flush=True)
    print(f"PointInstancer 数量: {n_point_instancer}", flush=True)
    print(f"PointInstancer 实例总数: {total_instances:,}", flush=True)
    print(f"scene-graph instance(prim.IsInstance) 数量: {n_scene_instance:,}", flush=True)
    print("=" * 78, flush=True)

    if bad_pi:
        print(f"[异常] 发现 {len(bad_pi)} 个数据异常的 PointInstancer:", flush=True)
        for path, n_idx, n_proto, problems in bad_pi[:top]:
            print(f"  {path}  (实例={n_idx:,}, 原型={n_proto})", flush=True)
            for p in problems:
                print(f"      - {p}", flush=True)
        print("=" * 78, flush=True)

    pi_rows.sort(key=lambda r: r[0], reverse=True)
    print(f"[Top {top}] 实例数最多的 PointInstancer:", flush=True)
    for n_idx, path in pi_rows[:top]:
        print(f"  实例数={n_idx:>10,}  {path}", flush=True)

    return 0


def main():
    parser = argparse.ArgumentParser(description="统计 USD 场景 instancing 规模")
    parser.add_argument("--usd_path", required=True)
    parser.add_argument("--top", type=int, default=30)
    args = parser.parse_args()
    return run(args.usd_path, args.top)


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    raise SystemExit(rc)
