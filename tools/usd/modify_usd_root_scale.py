#!/usr/bin/env python3
"""
批量修改 USD 文件中 /root prim 的 scale 为指定值(默认 0.01)。

适用场景:
- 把项目根目录下的 USD(如 LV_Temple_Day.usd、Demonstration.usd 等)
  整体缩放为 0.01(例如把 cm -> m)。
- 仅修改每个项目目录下的"项目级 USD 文件",不修改 Props/、Materials/
  等子目录里的 USD 资产。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/modify_usd_root_scale.py \
        --root /home/fufa/projects2026/DataGen_omni/asset_extern/USD \
        --scale 0.01 \
        --batch-size 10

实现方式:
- 主进程扫描每个一级子目录下"非 Props/Materials 的 .usd 文件"。
- 子进程通过 isaacsim + pxr 打开 USD,定位 /root,改写 xformOp:scale
  并保证 xformOpOrder 中包含该 op;若 /root 不存在则尝试 stage 的
  defaultPrim(同样位于根层级)。
- 通过子进程隔离防止 USD 解析崩溃影响整体流程;批崩溃自动回退到逐文件。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


# 在每个项目目录下,下面这些目录里的 .usd 视为"组件资产",不修改。
EXCLUDED_SUBDIRS = {"Props", "Materials", "Textures", "Maps", "Materials_old"}


# ---------------------------------------------------------------------------
# Worker: 真正打开 USD 并修改 /root scale 的逻辑(在子进程中运行)
# ---------------------------------------------------------------------------
def _set_scale_on_prim(prim, scale_value: tuple[float, float, float]) -> bool:
    """在给定 Xformable prim 上设置 xformOp:scale = scale_value。返回是否修改成功。"""
    from pxr import Gf, UsdGeom

    if not prim or not prim.IsValid():
        return False

    xformable = UsdGeom.Xformable(prim)
    if not xformable:
        print(f"[WORKER][WARN] prim 不是 Xformable: {prim.GetPath()}", flush=True)
        return False

    target = Gf.Vec3f(*scale_value)

    existing_ops = list(xformable.GetOrderedXformOps())
    scale_op = None
    for op in existing_ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            scale_op = op
            break

    if scale_op is None:
        try:
            scale_op = xformable.AddScaleOp(
                precision=UsdGeom.XformOp.PrecisionFloat,
            )
        except Exception:
            scale_op = xformable.AddScaleOp()
        # AddScaleOp 已自动追加到 xformOpOrder,无需再手动设置。
        existing_ops.append(scale_op)

    scale_op.Set(target)

    current_names = [op.GetOpName() for op in xformable.GetOrderedXformOps()]
    if scale_op.GetOpName() not in current_names:
        xformable.SetXformOpOrder(existing_ops)

    return True


def _modify_one_with_pxr(usd_path: str, scale_value: tuple[float, float, float]) -> int:
    """打开单个 USD,把 /root(或 defaultPrim)的 scale 改为 scale_value。"""
    from pxr import Usd

    print(f"[WORKER] 打开 {usd_path}", flush=True)
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[WORKER][ERROR] 打开失败: {usd_path}", flush=True)
        return 3

    target_prim = stage.GetPrimAtPath("/root")
    used_path = "/root"

    if not target_prim or not target_prim.IsValid():
        default_prim = stage.GetDefaultPrim()
        if default_prim and default_prim.IsValid():
            target_prim = default_prim
            used_path = str(default_prim.GetPath())
            print(
                f"[WORKER][INFO] 未找到 /root,回退到 defaultPrim: {used_path}",
                flush=True,
            )
        else:
            print(
                f"[WORKER][WARN] 既无 /root 也无 defaultPrim,跳过: {usd_path}",
                flush=True,
            )
            return 0

    ok = _set_scale_on_prim(target_prim, scale_value)
    if not ok:
        print(f"[WORKER][ERROR] 设置 scale 失败: {usd_path} ({used_path})", flush=True)
        return 4

    try:
        stage.GetRootLayer().Save()
        print(
            f"[WORKER] 已保存({used_path} scale -> {scale_value}): {usd_path}",
            flush=True,
        )
        return 0
    except Exception as e:
        print(f"[WORKER][ERROR] 保存失败: {e}", flush=True)
        return 5


def _worker_modify_batch(
    usd_paths: list[str], scale_value: tuple[float, float, float]
) -> int:
    """子进程批量处理。整体崩溃时主进程会回退到单文件。"""
    simulation_app = None
    try:
        from isaacsim import SimulationApp  # type: ignore
        simulation_app = SimulationApp({"headless": True})
    except Exception as e:
        print(
            f"[WORKER][WARN] 启动 SimulationApp 失败,尝试直接 import pxr: {e}",
            flush=True,
        )

    try:
        from pxr import Usd  # noqa: F401
    except ImportError as e:
        print(f"[WORKER][ERROR] 无法导入 pxr: {e}", flush=True)
        if simulation_app is not None:
            try:
                simulation_app.close()
            except Exception:
                pass
        return 2

    worst_rc = 0
    try:
        for path in usd_paths:
            print(f"[WORKER] === 处理 {path} ===", flush=True)
            try:
                rc = _modify_one_with_pxr(path, scale_value)
            except Exception as e:
                print(f"[WORKER][ERROR] 处理异常: {e}", flush=True)
                rc = 6
            print(f"[WORKER] === 结束(rc={rc}) ===", flush=True)
            worst_rc = max(worst_rc, rc)
    finally:
        if simulation_app is not None:
            try:
                simulation_app.close()
            except Exception:
                pass
    return worst_rc


# ---------------------------------------------------------------------------
# Master: 扫描目录,找出"项目级 USD"
# ---------------------------------------------------------------------------
def find_project_usd_files(root: Path) -> list[Path]:
    """对 root 下每个一级子目录,找出该目录下直接放置的 .usd 文件。

    - 只取一级子目录"自身目录"下的 .usd(不进入 Props/Materials 等)。
    - 每个项目目录可能有 1 个或多个项目级 USD,全部加入处理列表。
    """
    usd_files: list[Path] = []
    if not root.exists():
        return usd_files

    for sub in sorted(root.iterdir()):
        if not sub.is_dir():
            continue
        if sub.name in EXCLUDED_SUBDIRS:
            continue

        for child in sorted(sub.iterdir()):
            if child.is_file() and child.suffix.lower() == ".usd":
                usd_files.append(child)

    return usd_files


def _resolve_python_launcher() -> list[str]:
    """优先 PYTHON_LAUNCHER;其次 ./app/python.sh;最后 sys.executable。"""
    env_launcher = os.environ.get("PYTHON_LAUNCHER")
    if env_launcher:
        return env_launcher.split()
    candidates = [
        Path(__file__).resolve().parent.parent / "app" / "python.sh",
        Path.cwd() / "app" / "python.sh",
    ]
    for c in candidates:
        if c.is_file() and os.access(c, os.X_OK):
            return [str(c)]
    return [sys.executable]


def run_in_subprocess(
    usd_paths: list[Path],
    scale_value: tuple[float, float, float],
    timeout: float,
) -> tuple[int, str]:
    launcher = _resolve_python_launcher()
    cmd = [
        *launcher,
        os.path.abspath(__file__),
        "--worker",
        "--scale", str(scale_value[0]),
        "--scale-y", str(scale_value[1]),
        "--scale-z", str(scale_value[2]),
        "--usd-list",
        *[str(p) for p in usd_paths],
    ]
    try:
        proc = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        return proc.returncode, proc.stdout.decode("utf-8", errors="replace")
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode("utf-8", errors="replace")
        return -1, f"[timeout after {timeout}s]\n{out}"
    except Exception as e:
        return -2, f"[subprocess error] {e}"


def _process_batch(
    batch: list[Path],
    scale_value: tuple[float, float, float],
    timeout_per_file: float,
    ok: list[Path],
    failed: list[Path],
) -> None:
    total_timeout = max(timeout_per_file * max(len(batch), 1), 60.0)
    print(
        f"[MASTER] >>> 批处理 {len(batch)} 个文件(超时 {total_timeout:.0f}s)",
        flush=True,
    )
    rc, output = run_in_subprocess(batch, scale_value, total_timeout)
    for line in output.splitlines():
        print(f"    {line}")

    if rc == 0:
        ok.extend(batch)
        return

    if len(batch) == 1:
        print(f"[MASTER] 单文件失败(rc={rc}),标记为跳过: {batch[0]}")
        failed.append(batch[0])
        return

    print(f"[MASTER] 批处理失败(rc={rc}),回退到逐文件处理...")
    for f in batch:
        print(f"[MASTER]   -> 单独处理 {f}")
        rc1, output1 = run_in_subprocess([f], scale_value, timeout_per_file)
        for line in output1.splitlines():
            print(f"        {line}")
        if rc1 == 0:
            ok.append(f)
        else:
            print(f"[MASTER]   -> 失败(rc={rc1}),跳过: {f}")
            failed.append(f)


def main_master(args: argparse.Namespace) -> int:
    root = Path(args.root).expanduser().resolve()
    if not root.is_dir():
        print(f"[ERROR] 目录不存在: {root}", file=sys.stderr)
        return 1

    files = find_project_usd_files(root)
    print(
        f"[MASTER] 共发现 {len(files)} 个项目级 USD 文件,开始处理"
        f"(scale={args.scale}/{args.scale_y}/{args.scale_z}, batch={args.batch_size})"
    )
    for f in files:
        print(f"  - {f}")

    if args.limit > 0:
        files = files[: args.limit]
        print(f"[MASTER] 受 --limit 限制,只处理前 {len(files)} 个")

    if args.dry_run:
        print("[MASTER] --dry-run,已列出待修改文件,不执行修改。")
        return 0

    if not files:
        print("[MASTER] 没有要处理的文件。")
        return 0

    ok: list[Path] = []
    failed: list[Path] = []
    t0 = time.time()
    scale_value = (args.scale, args.scale_y, args.scale_z)

    bs = max(1, args.batch_size)
    for i in range(0, len(files), bs):
        batch = files[i : i + bs]
        print(f"\n[MASTER] ({i + 1}-{i + len(batch)}/{len(files)}) batch:")
        for f in batch:
            print(f"    - {f}")
        _process_batch(batch, scale_value, args.timeout, ok, failed)

    dt = time.time() - t0
    print("\n" + "=" * 60)
    print(
        f"[MASTER] 完成。成功 {len(ok)},失败/跳过 {len(failed)},用时 {dt:.1f}s"
    )
    if failed:
        print("[MASTER] 失败/跳过的文件:")
        for f in failed:
            print(f"  - {f}")
    return 0 if not failed else 0


# ---------------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="批量把项目级 USD 的 /root scale 改为指定值(默认 0.01)"
    )
    p.add_argument(
        "--root",
        default="/home/fufa/projects2026/DataGen_omni/asset_extern/USD",
        help="包含若干 USD 项目子目录的根目录",
    )
    p.add_argument(
        "--scale",
        type=float,
        default=0.01,
        help="要设置的 scale 值(X 轴;同时也是 Y/Z 默认值)",
    )
    p.add_argument(
        "--scale-y",
        type=float,
        default=None,
        help="Y 轴 scale(默认与 --scale 一致)",
    )
    p.add_argument(
        "--scale-z",
        type=float,
        default=None,
        help="Z 轴 scale(默认与 --scale 一致)",
    )
    p.add_argument("--timeout", type=float, default=180.0,
                   help="单文件超时秒数(默认 180s,按 batch 累加)")
    p.add_argument("--batch-size", type=int, default=10,
                   help="一个子进程批量处理多少文件(默认 10)")
    p.add_argument("--limit", type=int, default=0,
                   help="最多处理多少个文件(0 表示不限制)")
    p.add_argument("--dry-run", action="store_true",
                   help="只列出会被修改的文件,不真的修改")
    p.add_argument("--worker", action="store_true",
                   help="(内部使用)以 worker 模式处理 USD 文件")
    p.add_argument("--usd-list", nargs="*", default=[],
                   help="(内部使用)worker 模式下要处理的 USD 文件路径列表")
    return p


def main() -> int:
    args = build_parser().parse_args()

    if args.scale_y is None:
        args.scale_y = args.scale
    if args.scale_z is None:
        args.scale_z = args.scale

    if args.worker:
        if not args.usd_list:
            print("[ERROR] --worker 需要 --usd-list", file=sys.stderr)
            return 2
        return _worker_modify_batch(
            list(args.usd_list),
            (args.scale, args.scale_y, args.scale_z),
        )
    return main_master(args)


if __name__ == "__main__":
    sys.exit(main())
