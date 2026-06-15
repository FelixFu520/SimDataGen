"""在单进程内诊断 AsianVillage 所有 USD(主文件 + Props 子文件)的引用写法。

对每个 .usd 用 LoadNone 打开,提取 reference/payload 的原始 assetPath,
相对该文件目录解析,统计哪些解析不到磁盘(=坏引用),并打印坏引用的写法样例。

用法: ./app/python.sh tools/usd/_diagnose_refs.py <AsianVillage_dir> <out.txt>
"""
import os
import sys
from isaacsim import SimulationApp

root = os.path.abspath(sys.argv[1])
out_txt = sys.argv[2]

app = SimulationApp({"headless": True})
try:
    from pxr import Usd

    usd_files = []
    for dirpath, _, names in os.walk(root):
        if "_orig_backup" in dirpath:
            continue
        for n in names:
            if n.lower().endswith((".usd", ".usda", ".usdc")):
                usd_files.append(os.path.join(dirpath, n))
    usd_files.sort()

    def get_asset_paths(stage):
        out = []
        for prim in stage.Traverse():
            for spec in prim.GetPrimStack():
                rl = spec.referenceList
                if rl:
                    for r in list(rl.prependedItems) + list(rl.explicitItems) + list(rl.appendedItems):
                        if r.assetPath:
                            out.append(r.assetPath)
                pl = spec.payloadList
                if pl:
                    for r in list(pl.prependedItems) + list(pl.explicitItems) + list(pl.appendedItems):
                        if r.assetPath:
                            out.append(r.assetPath)
        return out

    report = []
    file_bad = {}  # file -> set(bad assetPath)
    total_bad = 0
    for uf in usd_files:
        try:
            stage = Usd.Stage.Open(uf, load=Usd.Stage.LoadNone)
        except Exception as e:
            report.append(f"[OPEN-ERR] {uf}: {e}")
            continue
        base_dir = os.path.dirname(uf)
        bad = set()
        for ap in get_asset_paths(stage):
            if ap.startswith("/") or "://" in ap:
                resolved = ap
            else:
                resolved = os.path.normpath(os.path.join(base_dir, ap))
            if not os.path.exists(resolved):
                bad.add(ap)
        if bad:
            file_bad[uf] = bad
            total_bad += len(bad)

    with open(out_txt, "w") as f:
        f.write(f"扫描 USD 文件数: {len(usd_files)}\n")
        f.write(f"含坏引用的文件数: {len(file_bad)}\n")
        f.write(f"坏引用总数(去重/文件内): {total_bad}\n")
        f.write("=" * 60 + "\n")
        for uf in sorted(file_bad):
            rel = os.path.relpath(uf, root)
            f.write(f"\n### {rel}  ({len(file_bad[uf])} bad)\n")
            for ap in sorted(file_bad[uf]):
                f.write(f"    '{ap}'\n")
    print(f"[DIAG] files={len(usd_files)} bad_files={len(file_bad)} bad_refs={total_bad} -> {out_txt}")
finally:
    app.close()
