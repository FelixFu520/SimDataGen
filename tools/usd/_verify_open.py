"""无头方式加载一个 USD 文件,验证能否正常打开,并统计引用解析情况。

用法:
    ./app/python.sh tools/usd/_verify_open.py <path-to.usd>
"""

import os
import sys

from isaacsim import SimulationApp

usd_path = sys.argv[1] if len(sys.argv) > 1 else None
if not usd_path:
    print("[VERIFY][ERROR] 未提供 USD 路径")
    sys.exit(2)

sim_app = SimulationApp({"headless": True})

exit_code = 0
try:
    import omni.usd
    from pxr import Usd, Sdf

    ctx = omni.usd.get_context()
    print(f"[VERIFY] 正在打开: {usd_path}")
    ret = ctx.open_stage(usd_path)
    ok = ret[0] if isinstance(ret, tuple) else bool(ret)
    print(f"[VERIFY] open_stage 返回 ok={ok}")

    for _ in range(30):
        sim_app.update()

    stage = ctx.get_stage()
    if stage is None:
        print("[VERIFY][ERROR] stage 为 None,打开失败")
        exit_code = 1
    else:
        all_prims = list(stage.TraverseAll())
        dp = stage.GetDefaultPrim()
        print(f"[VERIFY] defaultPrim = {dp.GetPath() if dp else None}")
        print(f"[VERIFY] prim 总数 = {len(all_prims)}")

        layer_dir = os.path.dirname(os.path.abspath(usd_path))

        # 收集所有外部引用/payload 的 assetPath,并判断是否能在磁盘解析到
        ref_assets = set()
        for prim in stage.Traverse():
            stack = prim.GetPrimStack()
            for spec in stack:
                if spec.referenceList:
                    for r in spec.referenceList.prependedItems:
                        if r.assetPath:
                            ref_assets.add(r.assetPath)
                    for r in spec.referenceList.explicitItems:
                        if r.assetPath:
                            ref_assets.add(r.assetPath)
                if spec.payloadList:
                    for r in spec.payloadList.prependedItems:
                        if r.assetPath:
                            ref_assets.add(r.assetPath)

        missing = []
        for ap in sorted(ref_assets):
            # 相对当前 layer 解析
            candidate = ap if os.path.isabs(ap) else os.path.normpath(os.path.join(layer_dir, ap))
            if not os.path.exists(candidate):
                missing.append((ap, candidate))

        print(f"[VERIFY] 唯一外部引用资产数 = {len(ref_assets)}")
        print(f"[VERIFY] 解析失败(磁盘上不存在)的引用 = {len(missing)}")
        for ap, cand in missing[:20]:
            print(f"   [MISSING] 写法='{ap}'  ->  解析到不存在路径='{cand}'")
        if len(missing) > 20:
            print(f"   ... 还有 {len(missing) - 20} 个")

        if not ok or len(all_prims) < 2:
            exit_code = 1
            print("[VERIFY][ERROR] 打开异常")
        elif missing:
            exit_code = 3
            print("[VERIFY][WARN] 文件可打开,但存在缺失引用(部分资产不会显示)")
        else:
            print("[VERIFY][SUCCESS] USD 文件可正常打开,且所有引用均可解析。")
except Exception as e:
    import traceback
    traceback.print_exc()
    print(f"[VERIFY][ERROR] 异常: {type(e).__name__}: {e}")
    exit_code = 1
finally:
    sim_app.close()

sys.exit(exit_code)
