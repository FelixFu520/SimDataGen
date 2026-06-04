#!/usr/bin/env bash
# 按 demo 录制的 rig_poses_*.npy 轨迹采数（见 gen_data_from_trajectory.py）
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
exec "$ROOT/app/python.sh" "$ROOT/tools/demo_data/gen_data_from_trajectory.py" "$@"
