#!/usr/bin/env bash
# 按 demo 录制的 rig_poses_*.npy 轨迹采数（见 gen_data_from_trajectory.py）。
# 输出含 path/paths.npy、rgb/ 等，目录结构兼容 scripts/batch_vis_to_mcap.sh。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

OUTPUT_DIR=""
args=("$@")
i=0
while [[ $i -lt ${#args[@]} ]]; do
  if [[ "${args[$i]}" == "--output_dir" && $((i + 1)) -lt ${#args[@]} ]]; then
    OUTPUT_DIR="${args[$((i + 1))]}"
    i=$((i + 2))
  else
    i=$((i + 1))
  fi
done

"$ROOT/app/python.sh" "$ROOT/tools/demo_data/gen_data_from_trajectory.py" "$@"

if [[ -n "$OUTPUT_DIR" && -f "${OUTPUT_DIR}/path/paths.npy" && -d "${OUTPUT_DIR}/rgb" ]]; then
  echo ""
  echo "采数完成，目录已兼容 batch_vis_to_mcap，可执行:"
  echo "  ./scripts/batch_vis_to_mcap.sh ${OUTPUT_DIR}"
fi
