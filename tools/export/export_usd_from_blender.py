"""Batch export Blender assets to USD via Blender's command-line mode.

Scans an asset root directory for folders named like ``<index> - <name>`` or
``<index>_<name>`` (e.g. ``001_amazing``), locates the ``.blend`` file inside
each, and exports it to a USD file using
Blender's headless mode (``blender --background --python``).

Example:
    python tools/export/export_usd_from_blender.py \
        --blender /home/fufa/projects2026/blender/blender-5.1.0-linux-x64/blender \
        --assets  /home/fufa/Downloads/blender/Interior01 \
        --output  /home/fufa/Downloads/blender/Interior01_usd \
        --start 1 --end 190 --workers 2
"""

from __future__ import annotations

import argparse
import concurrent.futures as cf
import logging
import os
import re
import subprocess
import sys
import textwrap
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

DEFAULT_BLENDER = "/home/fufa/projects2026/blender/blender-5.1.0-linux-x64/blender"
DEFAULT_ASSETS = "/home/fufa/Downloads/blender/Interior01"
DEFAULT_OUTPUT = "/home/fufa/Downloads/export"

INDEX_DIR_RES = (
    re.compile(r"^(\d+)\s*-\s*(.+)$"),  # "1 - name"
    re.compile(r"^(\d+)_(.+)$"),        # "001_name"
)


# Script executed inside Blender. It receives the output USD path through the
# ``BLENDER_USD_OUT`` environment variable to avoid quoting/escaping problems.
BLENDER_INNER_SCRIPT = textwrap.dedent(
    """
    import os
    import sys
    import bpy

    out_path = os.environ.get("BLENDER_USD_OUT")
    if not out_path:
        print("[inner] BLENDER_USD_OUT not set", file=sys.stderr)
        sys.exit(2)

    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    # Make sure objects in hidden/excluded collections aren't silently dropped
    # by toggling them on before export.
    for vl in bpy.context.scene.view_layers:
        for layer_col in vl.layer_collection.children:
            layer_col.exclude = False

    # Filter kwargs against the operator's actual properties so this script
    # remains compatible across Blender versions (5.x dropped some flags,
    # renamed others).
    op_props = {p.identifier for p in bpy.ops.wm.usd_export.get_rna_type().properties}

    desired = dict(
        filepath=out_path,
        selected_objects_only=False,
        export_animation=False,
        export_hair=False,
        export_uvmaps=True,
        export_normals=True,
        export_materials=True,
        export_meshes=True,
        export_lights=True,
        export_cameras=True,
        export_curves=True,
        export_points=True,
        export_volumes=True,
        export_armatures=True,
        export_shapekeys=True,
        export_subdivision="BEST_MATCH",
        # 5.x: NEW writes textures next to the USD; PRESERVE keeps in-place;
        # KEEP leaves whatever path Blender already has.
        export_textures_mode="NEW",
        overwrite_textures=False,
        relative_paths=True,
        use_instancing=True,
        evaluation_mode="RENDER",
        generate_preview_surface=True,
        # Legacy flag for older Blender; ignored if not present.
        export_textures=True,
        visible_objects_only=False,
    )

    kwargs = {k: v for k, v in desired.items() if k in op_props}
    print(f"[inner] using kwargs: {sorted(kwargs)}")

    result = bpy.ops.wm.usd_export(**kwargs)

    print(f"[inner] usd_export result: {result}")
    if "FINISHED" not in result:
        sys.exit(3)
    """
).strip()


@dataclass
class AssetJob:
    index: int
    folder: Path
    blend_file: Path
    out_usd: Path


def setup_logger(log_file: Optional[Path]) -> logging.Logger:
    logger = logging.getLogger("export_usd")
    logger.setLevel(logging.INFO)
    fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S")

    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(fmt)
    logger.addHandler(sh)

    if log_file is not None:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        fh = logging.FileHandler(log_file, encoding="utf-8")
        fh.setFormatter(fmt)
        logger.addHandler(fh)

    return logger


def discover_jobs(
    assets_root: Path,
    output_root: Path,
    start: int,
    end: int,
    logger: logging.Logger,
) -> list[AssetJob]:
    if not assets_root.is_dir():
        raise FileNotFoundError(f"Assets root not found: {assets_root}")

    # Map index -> folder for quick lookup.
    index_to_folder: dict[int, Path] = {}
    for child in assets_root.iterdir():
        if not child.is_dir():
            continue
        m = None
        for pat in INDEX_DIR_RES:
            m = pat.match(child.name)
            if m:
                break
        if m is None:
            continue
        try:
            idx = int(m.group(1))
        except ValueError:
            continue
        # If duplicates exist, prefer the first encountered and warn.
        if idx in index_to_folder:
            logger.warning(
                "Duplicate index %d: keeping %s, ignoring %s",
                idx,
                index_to_folder[idx].name,
                child.name,
            )
            continue
        index_to_folder[idx] = child

    jobs: list[AssetJob] = []
    for idx in range(start, end + 1):
        folder = index_to_folder.get(idx)
        if folder is None:
            logger.warning("Index %d: no matching folder, skipping", idx)
            continue

        blend_files = sorted(folder.glob("**/*.blend"))
        if not blend_files:
            logger.warning("Index %d (%s): no .blend file, skipping", idx, folder.name)
            continue
        if len(blend_files) > 1:
            logger.info(
                "Index %d (%s): multiple .blend files, using %s",
                idx,
                folder.name,
                blend_files[0].name,
            )

        blend_file = blend_files[0]
        out_usd = output_root / folder.name / f"{folder.name}.usd"
        jobs.append(
            AssetJob(index=idx, folder=folder, blend_file=blend_file, out_usd=out_usd)
        )

    return jobs


def run_one(
    job: AssetJob,
    blender_bin: Path,
    overwrite: bool,
    timeout: int,
    logger: logging.Logger,
) -> tuple[AssetJob, bool, str]:
    if job.out_usd.exists() and not overwrite:
        return job, True, "skipped (already exists)"

    job.out_usd.parent.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env["BLENDER_USD_OUT"] = str(job.out_usd)

    cmd = [
        str(blender_bin),
        "--background",
        "--factory-startup",
        str(job.blend_file),
        "--python-expr",
        BLENDER_INNER_SCRIPT,
    ]

    logger.info("[%d] Exporting %s", job.index, job.blend_file.name)
    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return job, False, f"timeout after {timeout}s"

    elapsed = time.time() - t0
    ok = proc.returncode == 0 and job.out_usd.exists()
    tail = proc.stdout.decode("utf-8", errors="replace").splitlines()[-15:]
    log_tail = "\n    ".join(tail)
    msg = (
        f"rc={proc.returncode} in {elapsed:.1f}s -> {job.out_usd}\n    {log_tail}"
        if ok
        else f"FAILED rc={proc.returncode} in {elapsed:.1f}s\n    {log_tail}"
    )
    return job, ok, msg


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--blender", type=Path, default=Path(DEFAULT_BLENDER), help="Path to the blender executable")
    p.add_argument("--assets", type=Path, default=Path(DEFAULT_ASSETS), help="Root directory containing '<idx> - <name>' or '<idx>_<name>' folders")
    p.add_argument("--output", type=Path, default=Path(DEFAULT_OUTPUT), help="Directory to write USD output")
    p.add_argument("--start", type=int, default=1, help="First asset index (inclusive)")
    p.add_argument("--end", type=int, default=190, help="Last asset index (inclusive)")
    p.add_argument("--workers", type=int, default=1, help="Parallel Blender processes (each uses lots of RAM)")
    p.add_argument("--timeout", type=int, default=1800, help="Per-asset timeout in seconds")
    p.add_argument("--overwrite", action="store_true", help="Re-export even if the USD file already exists")
    p.add_argument("--dry-run", action="store_true", help="List jobs but do not run Blender")
    p.add_argument("--log-file", type=Path, default=None, help="Optional path to write a log file")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    log_file = args.log_file or (args.output / "_export.log")
    logger = setup_logger(log_file)

    if not args.blender.exists():
        logger.error("Blender binary not found: %s", args.blender)
        return 2

    args.output.mkdir(parents=True, exist_ok=True)

    jobs = discover_jobs(args.assets, args.output, args.start, args.end, logger)
    logger.info("Discovered %d jobs in range [%d, %d]", len(jobs), args.start, args.end)

    if args.dry_run:
        for job in jobs:
            logger.info("DRY-RUN [%d] %s -> %s", job.index, job.blend_file, job.out_usd)
        return 0

    successes: list[AssetJob] = []
    failures: list[tuple[AssetJob, str]] = []

    if args.workers <= 1:
        for job in jobs:
            _, ok, msg = run_one(job, args.blender, args.overwrite, args.timeout, logger)
            logger.info("[%d] %s", job.index, msg)
            if ok:
                successes.append(job)
            else:
                failures.append((job, msg))
    else:
        with cf.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {
                pool.submit(run_one, job, args.blender, args.overwrite, args.timeout, logger): job
                for job in jobs
            }
            for fut in cf.as_completed(futures):
                job, ok, msg = fut.result()
                logger.info("[%d] %s", job.index, msg)
                if ok:
                    successes.append(job)
                else:
                    failures.append((job, msg))

    logger.info("=" * 60)
    logger.info("Done. success=%d  failed=%d  total=%d", len(successes), len(failures), len(jobs))
    if failures:
        logger.info("Failed assets:")
        for job, msg in failures:
            logger.info("  [%d] %s :: %s", job.index, job.folder.name, msg.splitlines()[0])

    return 0 if not failures else 1


if __name__ == "__main__":
    sys.exit(main())
