#!/usr/bin/env python3
"""
批量修改 USD 文件中所有 Light 的 intensity 为 4000。

用法(在仓库根目录下):
    ./app/python.sh tools/usd/modify_usd_light.py \
        --root /home/fufa/Downloads/blender/Interior01_usd \
        --intensity 4000 \
        --batch-size 10

注意:
- 通过子进程隔离来防崩溃:一个子进程一次处理 --batch-size 个文件；
  若该子进程整体崩溃,则自动回退到对该 batch 内每个文件单独处理,
  从而把崩溃的坏 USD 精确定位并跳过。
- 默认只修改每个一级子目录内同名的 .usd 文件；若找不到则递归查找。
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


# ---------------------------------------------------------------------------
# Worker: 真正打开 USD 并修改的逻辑(在子进程中运行)
# ---------------------------------------------------------------------------
def _modify_one_with_pxr(usd_path: str, intensity: float) -> int:
    """假设 pxr 已可导入,处理单个 USD 文件。返回 worker rc 风格的值。"""
    from pxr import Usd, UsdLux, Sdf  # 已确认可用

    print(f"[WORKER] 打开 {usd_path}", flush=True)
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print(f"[WORKER][ERROR] 打开失败:{usd_path}", flush=True)
        return 3

    modified = 0
    for prim in stage.Traverse():
        light = None
        try:
            if prim.HasAPI(UsdLux.LightAPI):
                light = UsdLux.LightAPI(prim)
            else:
                type_name = str(prim.GetTypeName() or "")
                if "Light" in type_name:
                    light = UsdLux.LightAPI(prim)
        except Exception:
            light = None
        if not light:
            continue

        try:
            attr = light.GetIntensityAttr()
            if not attr:
                attr = prim.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float)
            attr.Set(float(intensity))
            modified += 1
            print(
                f"[WORKER]   {prim.GetPath()} ({prim.GetTypeName()}) intensity -> {intensity}",
                flush=True,
            )
        except Exception as e:
            print(f"[WORKER][WARN]   设置 {prim.GetPath()} 失败:{e}", flush=True)

    if modified == 0:
        print(f"[WORKER] 没有发现 light,跳过保存:{usd_path}", flush=True)
        return 0
    try:
        stage.GetRootLayer().Save()
        print(f"[WORKER] 已保存(修改 {modified} 个 light):{usd_path}", flush=True)
        return 0
    except Exception as e:
        print(f"[WORKER][ERROR] 保存失败:{e}", flush=True)
        return 4


def _worker_modify_batch(usd_paths: list[str], intensity: float) -> int:
    """在一个子进程中批量处理多个 USD。崩溃时整个子进程退出,主进程会回退到单文件模式。"""
    simulation_app = None
    try:
        from isaacsim import SimulationApp  # type: ignore
        simulation_app = SimulationApp({"headless": True})
    except Exception as e:
        print(f"[WORKER][WARN] 启动 SimulationApp 失败,尝试直接 import pxr:{e}", flush=True)

    try:
        from pxr import Usd  # noqa: F401  # 探测 pxr 可用性
    except ImportError as e:
        print(f"[WORKER][ERROR] 无法导入 pxr:{e}", flush=True)
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
                rc = _modify_one_with_pxr(path, intensity)
            except Exception as e:
                print(f"[WORKER][ERROR] 处理异常:{e}", flush=True)
                rc = 5
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
# Master: 遍历目录,按文件分别 fork 子进程
# ---------------------------------------------------------------------------
def find_usd_files(root: Path, recursive: bool = True) -> list[Path]:
    """每个子目录内只取与子目录同名的 .usd 文件；若没有则取该目录下任意 .usd。"""
    usd_files: list[Path] = []
    if not root.exists():
        return usd_files

    for sub in sorted(root.iterdir()):
        if not sub.is_dir():
            continue
        same_name = sub / f"{sub.name}.usd"
        if same_name.is_file():
            usd_files.append(same_name)
            continue
        candidates = sorted(sub.glob("*.usd"))
        if candidates:
            usd_files.append(candidates[0])

    if not usd_files and recursive:
        usd_files = sorted(root.rglob("*.usd"))
    return usd_files


def _resolve_python_launcher() -> list[str]:
    """优先使用环境变量 PYTHON_LAUNCHER；否则用 isaacsim 的 ./app/python.sh；最后退到 sys.executable。"""
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


def run_in_subprocess(usd_paths: list[Path], intensity: float, timeout: float) -> tuple[int, str]:
    """以子进程方式批量处理一组文件。返回 (returncode, 合并 stdout)。"""
    launcher = _resolve_python_launcher()
    cmd = [
        *launcher,
        os.path.abspath(__file__),
        "--worker",
        "--intensity", str(intensity),
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
    intensity: float,
    timeout_per_file: float,
    ok: list[Path],
    failed: list[Path],
) -> None:
    """处理一个 batch。若整体崩溃且 batch>1,则回退到逐个处理以隔离坏文件。"""
    total_timeout = max(timeout_per_file * max(len(batch), 1), 60.0)
    print(f"[MASTER] >>> 批处理 {len(batch)} 个文件(超时 {total_timeout:.0f}s)", flush=True)
    rc, output = run_in_subprocess(batch, intensity, total_timeout)
    for line in output.splitlines():
        print(f"    {line}")

    if rc == 0:
        ok.extend(batch)
        return

    if len(batch) == 1:
        print(f"[MASTER] 单文件失败(rc={rc}),标记为跳过:{batch[0]}")
        failed.append(batch[0])
        return

    print(f"[MASTER] 批处理失败(rc={rc}),回退到逐文件处理...")
    for f in batch:
        print(f"[MASTER]   -> 单独处理 {f}")
        rc1, output1 = run_in_subprocess([f], intensity, timeout_per_file)
        for line in output1.splitlines():
            print(f"        {line}")
        if rc1 == 0:
            ok.append(f)
        else:
            print(f"[MASTER]   -> 失败(rc={rc1}),跳过:{f}")
            failed.append(f)


def main_master(args: argparse.Namespace) -> int:
    root = Path(args.root).expanduser().resolve()
    if not root.is_dir():
        print(f"[ERROR] 目录不存在:{root}", file=sys.stderr)
        return 1

    files = find_usd_files(root, recursive=args.recursive)
    print(f"[MASTER] 共发现 {len(files)} 个 USD 文件,开始处理(intensity={args.intensity},batch={args.batch_size})")
    if args.limit > 0:
        files = files[: args.limit]
        print(f"[MASTER] 受 --limit 限制,只处理前 {len(files)} 个")

    ok: list[Path] = []
    failed: list[Path] = []
    t0 = time.time()

    bs = max(1, args.batch_size)
    for i in range(0, len(files), bs):
        batch = files[i : i + bs]
        print(f"\n[MASTER] ({i + 1}-{i + len(batch)}/{len(files)}) batch:")
        for f in batch:
            print(f"    - {f}")
        _process_batch(batch, args.intensity, args.timeout, ok, failed)

    dt = time.time() - t0
    print("\n" + "=" * 60)
    print(f"[MASTER] 完成。成功 {len(ok)},失败/跳过 {len(failed)},用时 {dt:.1f}s")
    if failed:
        print("[MASTER] 失败/跳过的文件:")
        for f in failed:
            print(f"  - {f}")
    return 0


# ---------------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="批量修改 USD light 的 intensity")
    p.add_argument("--root", default="/home/fufa/Downloads/blender/Interior01_usd",
                   help="包含若干 USD 子目录的根目录")
    p.add_argument("--intensity", type=float, default=4000.0,
                   help="要设置的 light intensity 值(默认 4000)")
    p.add_argument("--timeout", type=float, default=360.0,
                   help="每个 USD 文件的处理超时秒数(默认 180s,按 batch 累加)")
    p.add_argument("--batch-size", type=int, default=10,
                   help="一个子进程批量处理多少文件,越大越快但崩溃时一起失败再回退(默认 10)")
    p.add_argument("--limit", type=int, default=0,
                   help="最多处理多少个文件(0 表示不限制)")
    p.add_argument("--recursive", action="store_true", default=True,
                   help="找不到一级子目录的 usd 时,递归查找所有 *.usd")
    p.add_argument("--no-recursive", dest="recursive", action="store_false")
    p.add_argument("--worker", action="store_true",
                   help="(内部使用)以 worker 模式批量处理 USD 文件")
    p.add_argument("--usd-list", nargs="*", default=[],
                   help="(内部使用)worker 模式下要处理的 USD 文件路径列表")
    return p


def main() -> int:
    args = build_parser().parse_args()
    if args.worker:
        if not args.usd_list:
            print("[ERROR] --worker 需要 --usd-list", file=sys.stderr)
            return 2
        return _worker_modify_batch(list(args.usd_list), args.intensity)
    return main_master(args)


if __name__ == "__main__":
    sys.exit(main())
