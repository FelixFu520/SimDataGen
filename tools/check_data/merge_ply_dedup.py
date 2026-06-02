#!/usr/bin/env python3
"""Merge <prefix>*.ply files in a directory and remove duplicate vertices (by xyz)."""

import argparse
import glob
import os
import sys
import time

import numpy as np

PLY_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('r', 'u1'), ('g', 'u1'), ('b', 'u1'),
])
KEY_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])


def read_ply_binary(path: str) -> np.ndarray:
    with open(path, 'rb') as f:
        n = None
        while True:
            line = f.readline().decode('ascii').strip()
            if line.startswith('element vertex'):
                n = int(line.split()[-1])
            if line == 'end_header':
                break
        if n is None:
            raise ValueError(f"no vertex count in {path}")
        return np.fromfile(f, dtype=PLY_DTYPE, count=n)


def save_ply_binary(path: str, data: np.ndarray) -> None:
    n = len(data)
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {n}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        data.tofile(f)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_dir', help='directory containing all_*.ply files')
    parser.add_argument('--prefix', default='all_', help='prefix of the ply files')
    parser.add_argument('-o', '--output', default=None, help='output ply path')
    args = parser.parse_args()

    pattern = os.path.join(args.input_dir, f'{args.prefix}*.ply')
    files = sorted(glob.glob(pattern))
    if not files:
        print(f"no {args.prefix}*.ply files in {args.input_dir}", file=sys.stderr)
        sys.exit(1)

    out_path = args.output or os.path.join(args.input_dir, 'merged.ply')

    t0 = time.time()
    chunks = []
    total = 0
    for i, fpath in enumerate(files):
        data = read_ply_binary(fpath)
        chunks.append(data)
        total += len(data)
        print(f"[{i+1}/{len(files)}] {os.path.basename(fpath)}: {len(data):,}  (cum {total:,})")

    print(f"\nconcatenating {total:,} vertices ...")
    merged = np.concatenate(chunks)
    del chunks

    print("deduplicating by xyz ...")
    keys = np.empty(len(merged), dtype=KEY_DTYPE)
    keys['x'] = merged['x']
    keys['y'] = merged['y']
    keys['z'] = merged['z']
    _, unique_idx = np.unique(keys, return_index=True)
    unique_idx.sort()
    deduped = merged[unique_idx]
    del merged, keys

    removed = total - len(deduped)
    print(f"before: {total:,}  after: {len(deduped):,}  removed: {removed:,} ({100*removed/total:.1f}%)")

    print(f"writing {out_path} ...")
    save_ply_binary(out_path, deduped)
    elapsed = time.time() - t0
    print(f"done in {elapsed:.1f}s -> {out_path}")


if __name__ == '__main__':
    main()
