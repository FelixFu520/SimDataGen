"""导出 USD 文件中所有引用/payload 的原始 assetPath(去重),写到 stdout 文件。

用法: ./app/python.sh tools/usd/_dump_refs.py <in.usd> <out.txt>
"""
import sys
from isaacsim import SimulationApp

in_usd = sys.argv[1]
out_txt = sys.argv[2]

app = SimulationApp({"headless": True})
try:
    from pxr import Usd, Sdf

    stage = Usd.Stage.Open(in_usd, load=Usd.Stage.LoadNone)
    paths = set()
    for prim in stage.Traverse():
        for spec in prim.GetPrimStack():
            rl = spec.referenceList
            if rl:
                for r in list(rl.prependedItems) + list(rl.explicitItems) + list(rl.appendedItems):
                    if r.assetPath:
                        paths.add(("ref", r.assetPath))
            pl = spec.payloadList
            if pl:
                for r in list(pl.prependedItems) + list(pl.explicitItems) + list(pl.appendedItems):
                    if r.assetPath:
                        paths.add(("payload", r.assetPath))
    with open(out_txt, "w") as f:
        for kind, ap in sorted(paths):
            f.write(f"{kind}\t{ap}\n")
    print(f"[DUMP] wrote {len(paths)} unique asset paths to {out_txt}")
finally:
    app.close()
