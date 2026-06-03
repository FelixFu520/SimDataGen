#!/usr/bin/env bash
# 批量对指定目录下每个子文件夹:
#   1. project_cloud.py 生成全部帧 vis/ 点云 (vis/ 已齐全则跳过)
#   2. path_vis_to_mcap.py 生成 MCAP
#
# 用法:
# # 基本用法：处理 workdir 下所有子目录
# ./scripts/batch_vis_to_mcap.sh workdir

# # MCAP 写到单独目录
# ./scripts/batch_vis_to_mcap.sh workdir --output output/mcaps

# # 调整投影 / MCAP 降采样
# ./scripts/batch_vis_to_mcap.sh workdir --project-downsample 2 --mcap-downsample 20

# # 首次运行可安装依赖（OpenEXR、mcap 等）
# ./scripts/batch_vis_to_mcap.sh workdir --install-deps

# # 透传 path_vis_to_mcap 参数
# ./scripts/batch_vis_to_mcap.sh workdir --output output/mcaps -- --fps 6
#
# 仅处理含 rgb/ 与 path/paths.npy 的直接子目录。

set -euo pipefail

usage() {
  cat <<'EOF'
用法: batch_vis_to_mcap.sh <parent_dir> [选项] [-- mcap 额外参数...]

选项:
  --output DIR              MCAP 输出目录 (传给 path_vis_to_mcap.py --output)
  --project-downsample N    project_cloud.py 降采样 (默认 1)
  --project-undistort-iters N
                            project_cloud.py RadTan 去畸变迭代次数 (默认 20)
  --mcap-downsample N       path_vis_to_mcap.py 点云降采样 (默认 10)
  --skip-project            跳过点云投影, 仅生成 MCAP
  --skip-mcap               仅做点云投影, 不生成 MCAP
  --install-deps            安装 OpenEXR / mcap 依赖 (默认不安装)
  -h, --help                显示帮助

示例:
  ./scripts/batch_vis_to_mcap.sh workdir
  ./scripts/batch_vis_to_mcap.sh workdir --output output/mcaps --mcap-downsample 20
  ./scripts/batch_vis_to_mcap.sh workdir -- --fps 6 --accumulate
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 ]]; then
  usage >&2
  exit 1
fi

PARENT_DIR="$(cd "$1" && pwd)"
shift

OUTPUT_DIR=""
PROJECT_DOWNSAMPLE=1
PROJECT_UNDISTORT_ITERS=20
MCAP_DOWNSAMPLE=10
SKIP_PROJECT=0
SKIP_MCAP=0
INSTALL_DEPS=0
MCAP_EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --project-downsample)
      PROJECT_DOWNSAMPLE="$2"
      shift 2
      ;;
    --project-undistort-iters)
      PROJECT_UNDISTORT_ITERS="$2"
      shift 2
      ;;
    --mcap-downsample)
      MCAP_DOWNSAMPLE="$2"
      shift 2
      ;;
    --skip-project)
      SKIP_PROJECT=1
      shift
      ;;
    --skip-mcap)
      SKIP_MCAP=1
      shift
      ;;
    --install-deps)
      INSTALL_DEPS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      MCAP_EXTRA_ARGS+=("$@")
      break
      ;;
    *)
      MCAP_EXTRA_ARGS+=("$1")
      shift
      ;;
  esac
done

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_DIR"

PYTHON="${PROJECT_DIR}/app/python.sh"

if [[ ! -d "$PARENT_DIR" ]]; then
  echo "错误: 目录不存在: $PARENT_DIR" >&2
  exit 1
fi

if [[ "$INSTALL_DEPS" -eq 1 ]]; then
  echo "安装依赖 ..."
  "$PYTHON" -m pip install -q OpenEXR==3.4.9 \
    mcap mcap-protobuf-support foxglove-schemas-protobuf
fi

count_rgb_frames() {
  list_rgb_frame_ids "$1" | wc -l
}

get_ref_cam() {
  local workdir="$1"
  local rgb_root="${workdir}/rgb"

  if [[ ! -d "$rgb_root" ]]; then
    return 1
  fi

  find "$rgb_root" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort | head -1
}

list_rgb_frame_ids() {
  local workdir="$1"
  local rgb_root="${workdir}/rgb"
  local ref_cam

  ref_cam="$(get_ref_cam "$workdir" || true)"
  if [[ -z "$ref_cam" ]]; then
    return
  fi

  find "${rgb_root}/${ref_cam}" -maxdepth 1 \
    \( -name '*.jpg' -o -name '*.jpeg' -o -name '*.png' \) -printf '%f\n' \
    | sed 's/\.[^.]*$//' \
    | sort
}

vis_frames_complete() {
  local workdir="$1"
  local vis_dir="${workdir}/vis"
  local frame_ids=()

  mapfile -t frame_ids < <(list_rgb_frame_ids "$workdir")
  if [[ ${#frame_ids[@]} -eq 0 ]]; then
    return 1
  fi

  for frame_id in "${frame_ids[@]}"; do
    if [[ ! -f "${vis_dir}/all_cameras_world_${frame_id}.ply" ]]; then
      return 1
    fi
  done
  return 0
}

is_workdir() {
  local workdir="$1"
  [[ -d "${workdir}/rgb" && -f "${workdir}/path/paths.npy" ]]
}

mapfile -t WORKDIRS < <(
  find "$PARENT_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%p\n' | sort
)

if [[ ${#WORKDIRS[@]} -eq 0 ]]; then
  echo "未找到子目录: $PARENT_DIR" >&2
  exit 1
fi

processed=0
skipped=0

for workdir in "${WORKDIRS[@]}"; do
  name="$(basename "$workdir")"

  if ! is_workdir "$workdir"; then
    echo "[跳过] ${name}: 不是有效数据目录 (需 rgb/ 与 path/paths.npy)"
    skipped=$((skipped + 1))
    continue
  fi

  echo ""
  echo "============================================================"
  echo "处理: ${workdir}"
  echo "============================================================"

  if [[ "$SKIP_PROJECT" -eq 0 ]]; then
    frame_count="$(count_rgb_frames "$workdir")"
    if [[ "$frame_count" -eq 0 ]]; then
      echo "[跳过] ${name}: rgb/ 下无图像帧"
      skipped=$((skipped + 1))
      continue
    fi

    vis_dir="${workdir}/vis"

    if vis_frames_complete "$workdir"; then
      echo ">> vis/ 已含全部 ${frame_count} 帧, 跳过 project_cloud"
    else
      mkdir -p "$vis_dir"
      echo ">> project_cloud.py: ${frame_count} 帧 -> ${vis_dir}"
      "$PYTHON" project_cloud.py \
        --data_dir "$workdir" \
        --output_dir "$vis_dir" \
        --show_num "$frame_count" \
        --downsample "$PROJECT_DOWNSAMPLE" \
        --undistort_iters "$PROJECT_UNDISTORT_ITERS"
    fi
  else
    echo ">> 跳过 project_cloud (--skip-project)"
  fi

  if [[ "$SKIP_MCAP" -eq 0 ]]; then
    mcap_cmd=(
      "$PYTHON" tools/check_data/path_vis_to_mcap.py
      "$workdir"
      --downsample "$MCAP_DOWNSAMPLE"
    )
    if [[ -n "$OUTPUT_DIR" ]]; then
      mcap_cmd+=(--output "$OUTPUT_DIR")
    fi
    if [[ ${#MCAP_EXTRA_ARGS[@]} -gt 0 ]]; then
      mcap_cmd+=("${MCAP_EXTRA_ARGS[@]}")
    fi

    echo ">> path_vis_to_mcap.py"
    "${mcap_cmd[@]}"
  else
    echo ">> 跳过 path_vis_to_mcap (--skip-mcap)"
  fi

  processed=$((processed + 1))
done

echo ""
echo "完成: 处理 ${processed} 个目录, 跳过 ${skipped} 个 (parent=${PARENT_DIR})"
