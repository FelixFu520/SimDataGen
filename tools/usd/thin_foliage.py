#!/usr/bin/env python3
"""精简(下采样)USD 场景里的 PointInstancer 植被实例, 避免 RTX 渲染器在构建
几何加速结构(scenedb/BVH)时因实例数量过多而 native 崩溃。

背景:
- 从 UE 导出的场景常带巨量植被(草/树) PointInstancer。例如 ForestHourse 的
  `Landscape_1_Foliage` 单个 PointInstancer 就有 1428 万个实例。
- gen_data.py 第一次 `world.step(render=True)` 时, RTX 要为这些实例构建光追加速
  结构, 千万级实例会让 librtx.scenedb 在 C++ 层崩溃(见 docs/fix_bug.md)。
- 本工具对 PointInstancer 的 per-instance 数组(positions/orientations/scales/
  protoIndices)做同步随机下采样, 或直接停用(deactivate)整个 instancer, 从而把
  实例数降到 RTX 能承受的量级。

支持两种精简强度(按命中的 instancer 逐个判断, 满足任一上限即触发下采样):
  --ratio R          : 保留比例(0~1], 例如 0.02 表示只保留 2% 实例
  --max-instances N  : 每个 instancer 实例数上限, 超过则下采样到 N

也可整体移除:
  --remove           : 直接 deactivate 命中的 instancer(实例数归零, 不再渲染)

选择命中目标:
  --match KW [KW...] : 只处理路径/名字包含关键词的 instancer(默认处理全部)

输出控制(默认写到新文件, 保护原始 USD):
  --out PATH         : 另存为指定文件
  --in-place         : 原地覆盖写回 --usd_path
  --dry-run          : 只预览不写入
  (都不给时, 默认在原文件名后加 .thinned.usd 另存)

用法(在仓库根目录下):
    # 预览: 把所有植被下采样到每个 instancer 最多 20 万个
    ./app/python.sh tools/usd/thin_foliage.py \
        --usd_path assets_extern/TaoBao11_fix2/ForestHourse/Map_Houses_A.usd \
        --max-instances 200000 --dry-run

    # 实际执行(另存为 *.thinned.usd):
    ./app/python.sh tools/usd/thin_foliage.py \
        --usd_path assets_extern/TaoBao11_fix2/ForestHourse/Map_Houses_A.usd \
        --max-instances 200000

    # 按比例保留 1%, 原地写回:
    ./app/python.sh tools/usd/thin_foliage.py \
        --usd_path .../Map_Houses_A.usd --ratio 0.01 --in-place

    # 只精简草(Grass/Foliage), 其它保留:
    ./app/python.sh tools/usd/thin_foliage.py \
        --usd_path .../Map_Houses_A.usd --match Foliage Grass --max-instances 100000

    # 直接整体移除所有植被 instancer:
    ./app/python.sh tools/usd/thin_foliage.py \
        --usd_path .../Map_Houses_A.usd --remove
"""
import argparse
import os

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import numpy as np
from pxr import Usd, UsdGeom

# PointInstancer 上需要随同 protoIndices 一起下采样的 per-instance 数组属性名。
# 注意 extent 是整体 bounds(长度恒为 2), 不属于 per-instance, 不能动。
_PER_INSTANCE_ATTRS = (
    "positions",
    "orientations",
    "scales",
    "protoIndices",
    "ids",
    "velocities",
    "angularVelocities",
    "accelerations",
    "invisibleIds",
)


def _matches(path: str, name: str, keywords, case_sensitive: bool) -> bool:
    if not keywords:
        return True
    hay_p = path if case_sensitive else path.lower()
    hay_n = name if case_sensitive else name.lower()
    for kw in keywords:
        needle = kw if case_sensitive else kw.lower()
        if needle in hay_p or needle in hay_n:
            return True
    return False


def _target_keep(n: int, ratio, max_instances) -> int:
    """根据 ratio / max-instances 计算应保留的实例数(取更严格者)。"""
    keep = n
    if ratio is not None:
        keep = min(keep, max(1, int(round(n * ratio))))
    if max_instances is not None:
        keep = min(keep, max_instances)
    return keep


def _set_subset(prim, name: str, idx_list, n_instances: int, dry_run: bool):
    """对 per-instance 数组按下标列表取子集写回, 类型自动保持。返回 (old, new) 或 None。

    只对长度等于实例总数 n_instances 的数组下采样; 长度不符的(异常或非 per-instance)
    一律跳过, 防止写坏。
    """
    attr = prim.GetAttribute(name)
    if not attr or not attr.HasAuthoredValue():
        return None
    val = attr.Get()
    if val is None:
        return None
    if name == "invisibleIds":
        return None
    old_len = len(val)
    if old_len == 0:
        return None
    if old_len != n_instances:
        # 长度与实例总数不符(可能是 ids 缺省或异常), 不动, 防止写坏。
        return ("SKIP", old_len)

    vt_type = type(val)  # 例如 Vt.Vec3fArray / Vt.QuathArray / Vt.IntArray
    subset = [val[i] for i in idx_list]
    if dry_run:
        return (old_len, len(subset))
    attr.Set(vt_type(subset))
    return (old_len, len(subset))


def run(
    usd_path: str,
    keywords,
    case_sensitive: bool,
    ratio,
    max_instances,
    remove: bool,
    seed: int,
    out: str,
    in_place: bool,
    dry_run: bool,
) -> int:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[ERROR] 打开失败: {usd_path}", flush=True)
        return 1

    rng = np.random.default_rng(seed)
    processed = []
    total_before = 0
    total_after = 0

    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.PointInstancer):
            continue
        path = str(prim.GetPath())
        if not _matches(path, prim.GetName(), keywords, case_sensitive):
            continue

        pi = UsdGeom.PointInstancer(prim)
        proto_indices = pi.GetProtoIndicesAttr().Get()
        n = 0 if proto_indices is None else len(proto_indices)
        total_before += n

        if remove:
            if not dry_run:
                prim.SetActive(False)
            processed.append({"path": path, "before": n, "after": 0, "action": "deactivate"})
            continue

        keep = _target_keep(n, ratio, max_instances)
        if keep >= n:
            processed.append({"path": path, "before": n, "after": n, "action": "保持(未超阈值)"})
            total_after += n
            continue

        # 随机选 keep 个实例下标, 排序以保持原相对顺序(便于复现/对比)。
        sel = np.sort(rng.choice(n, size=keep, replace=False))
        idx_list = sel.tolist()

        details = []
        for name in _PER_INSTANCE_ATTRS:
            res = _set_subset(prim, name, idx_list, n, dry_run)
            if res is None:
                continue
            if res[0] == "SKIP":
                details.append(f"{name}(长度{res[1]}与实例数不符,跳过)")
            else:
                details.append(f"{name}:{res[0]}->{res[1]}")

        processed.append(
            {"path": path, "before": n, "after": keep, "action": "下采样", "details": details}
        )
        total_after += keep

    # ---- 打印汇总 ----
    print("=" * 78, flush=True)
    print(f"文件: {usd_path}", flush=True)
    mode = (
        "整体移除(deactivate)"
        if remove
        else f"下采样(ratio={ratio}, max_instances={max_instances}, seed={seed})"
    )
    print(f"模式: {mode}", flush=True)
    print(f"命中 PointInstancer: {len(processed)} 个", flush=True)
    print(f"实例总数: {total_before:,} -> {total_after:,}", flush=True)
    print("=" * 78, flush=True)
    for item in processed:
        print(f"  [{item['action']}] {item['path']}  ({item['before']:,} -> {item['after']:,})", flush=True)
        for d in item.get("details", []):
            print(f"      - {d}", flush=True)

    if dry_run:
        print("\n[DRY-RUN] 未写入。去掉 --dry-run 即实际保存。", flush=True)
        return 0

    if not processed or (total_before == total_after and not remove):
        print("\n无需修改(没有命中或都未超阈值), 未保存。", flush=True)
        return 0

    # ---- 决定输出路径 ----
    if in_place:
        save_path = usd_path
        stage.GetRootLayer().Save()
    else:
        if out:
            save_path = out
        else:
            base, ext = os.path.splitext(usd_path)
            save_path = f"{base}.thinned{ext}"
        stage.GetRootLayer().Export(save_path)

    print(f"\n[SAVED] 已写入: {save_path}", flush=True)
    if not in_place:
        print("提示: 用精简后的文件跑 gen_data.py(把 --scene_usd_url 指向它)。", flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description="精简(下采样/移除)USD 场景里的 PointInstancer 植被实例")
    parser.add_argument("--usd_path", required=True, help="目标 USD 文件路径")
    parser.add_argument("--match", nargs="+", default=None, help="只处理路径/名字含关键词的 instancer(默认全部)")
    parser.add_argument("--case-sensitive", action="store_true", help="区分大小写匹配(默认不区分)")
    parser.add_argument("--ratio", type=float, default=None, help="保留比例(0,1], 例如 0.02 表示保留 2%%")
    parser.add_argument("--max-instances", type=int, default=None, help="每个 instancer 实例数上限, 超过则下采样到此值")
    parser.add_argument("--remove", action="store_true", help="直接 deactivate 命中的 instancer(整体移除)")
    parser.add_argument("--seed", type=int, default=0, help="随机下采样种子(可复现)")
    parser.add_argument("--out", default=None, help="另存为指定文件路径")
    parser.add_argument("--in-place", action="store_true", help="原地覆盖写回 --usd_path")
    parser.add_argument("--dry-run", action="store_true", help="只预览不写入")
    args = parser.parse_args()

    if not args.remove and args.ratio is None and args.max_instances is None:
        print("[ERROR] 需指定 --ratio 或 --max-instances 之一, 或用 --remove 整体移除。", flush=True)
        return 2
    if args.ratio is not None and not (0.0 < args.ratio <= 1.0):
        print("[ERROR] --ratio 必须在 (0, 1] 区间。", flush=True)
        return 2

    return run(
        usd_path=args.usd_path,
        keywords=args.match,
        case_sensitive=args.case_sensitive,
        ratio=args.ratio,
        max_instances=args.max_instances,
        remove=args.remove,
        seed=args.seed,
        out=args.out,
        in_place=args.in_place,
        dry_run=args.dry_run,
    )


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        simulation_app.close()
    raise SystemExit(rc)
