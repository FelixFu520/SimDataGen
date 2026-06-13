#!/usr/bin/env bash
# -*- coding: utf-8 -*-
#
# 批量过滤多个 workdir 目录, 并对 save / discard 名单各自抽帧拼图(可选拼视频)。
#
# 对每个 <workdir> 依次执行:
#   1) filter_trajectories.py  -> <out-dir>/<workdir>.csv
#   2) sample_trajectories.py --decision save     -> <out-dir>/<workdir>_save
#   3) sample_trajectories.py --decision discard  -> <out-dir>/<workdir>_discard
#
# 用法:
#   tools/filter/batch_filter.sh [选项] [workdir ...]
#
# 不传 workdir 时使用脚本内置的 DEFAULT_DIRS 列表。
#
# 选项(均有默认值):
#   --out-dir DIR    输出目录            (默认 workdir_filter)
#   --workers N      过滤并行进程数      (默认 32)
#   --step N         抽帧间隔            (默认 15)
#   --fps F          视频帧率            (默认 1, 仅 --video 时生效)
#   --video          额外拼视频(默认不拼)
#   --py PATH        python 解释器       (默认 /root/miniconda3/envs/volc/bin/python)
#   -h, --help       显示帮助
#
# 示例:
#   tools/filter/batch_filter.sh
#   tools/filter/batch_filter.sh workdir_taobao04 workdir_taobao05
#   tools/filter/batch_filter.sh --out-dir workdir_filter --workers 32 --step 15

set -u

# ----------------------------- 默认参数 ----------------------------- #
PY="/root/miniconda3/envs/volc/bin/python"
OUT_DIR="workdir_filter"
WORKERS=32
STEP=15
FPS=1
VIDEO=0

DEFAULT_DIRS=(
    workdir_intime
    workdir_kujiale
    workdir_slow
    workdir_taobao
    workdir_taobao02
    workdir_taobao03
    workdir_taobao04
    workdir_taobao05
    workdir_taobao06
    workdir_taobao07
    workdir_taobao08_00
    workdir_taobao08_01
    workdir_taobao09
    workdir_taobao10
    workdir_taobao11
)

# 脚本所在目录, 用于定位同目录下的 py 脚本
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILTER_PY="$SCRIPT_DIR/filter_trajectories.py"
SAMPLE_PY="$SCRIPT_DIR/sample_trajectories.py"

# ----------------------------- 解析参数 ----------------------------- #
DIRS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --out-dir) OUT_DIR="$2"; shift 2 ;;
        --workers) WORKERS="$2"; shift 2 ;;
        --step)    STEP="$2";    shift 2 ;;
        --fps)     FPS="$2";     shift 2 ;;
        --video)   VIDEO=1;      shift ;;
        --py)      PY="$2";      shift 2 ;;
        -h|--help)
            sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        -*)
            echo "未知选项: $1" >&2; exit 2 ;;
        *)
            DIRS+=("$1"); shift ;;
    esac
done

if [[ ${#DIRS[@]} -eq 0 ]]; then
    DIRS=("${DEFAULT_DIRS[@]}")
fi

VIDEO_FLAG=""
if [[ "$VIDEO" -eq 1 ]]; then
    VIDEO_FLAG="--video"
fi

mkdir -p "$OUT_DIR"

# ----------------------------- 主流程 ----------------------------- #
total=${#DIRS[@]}
idx=0
ok=()
skipped=()
failed=()

echo "========================================================"
echo "批量过滤: 共 $total 个目录"
echo "  out-dir=$OUT_DIR workers=$WORKERS step=$STEP fps=$FPS video=$VIDEO"
echo "  py=$PY"
echo "========================================================"

for d in "${DIRS[@]}"; do
    idx=$((idx + 1))
    # 去掉可能的结尾斜杠, 取目录名作为 csv 名
    name="$(basename "${d%/}")"
    csv="$OUT_DIR/$name.csv"

    echo ""
    echo "[$idx/$total] ===== $name ====="

    if [[ ! -d "$d" ]]; then
        echo "  [跳过] 目录不存在: $d"
        skipped+=("$name")
        continue
    fi

    # 1) 过滤
    echo "  -> 过滤 filter_trajectories.py"
    if ! "$PY" "$FILTER_PY" --root "$d" --out-dir "$OUT_DIR" --workers "$WORKERS"; then
        echo "  [失败] 过滤出错: $name"
        failed+=("$name")
        continue
    fi

    if [[ ! -f "$csv" ]]; then
        echo "  [失败] 未生成 csv: $csv"
        failed+=("$name")
        continue
    fi

    # 2) save 抽帧拼图
    echo "  -> 抽帧拼图(save) sample_trajectories.py"
    "$PY" "$SAMPLE_PY" --list "$csv" --decision save \
        --root "$d" --out-dir "$OUT_DIR/${name}_save" \
        --step "$STEP" --fps "$FPS" $VIDEO_FLAG \
        || echo "  [警告] save 抽帧出错(继续): $name"

    # 3) discard 抽帧拼图
    echo "  -> 抽帧拼图(discard) sample_trajectories.py"
    "$PY" "$SAMPLE_PY" --list "$csv" --decision discard \
        --root "$d" --out-dir "$OUT_DIR/${name}_discard" \
        --step "$STEP" --fps "$FPS" $VIDEO_FLAG \
        || echo "  [警告] discard 抽帧出错(继续): $name"

    ok+=("$name")
done

# ----------------------------- 汇总 ----------------------------- #
echo ""
echo "========================================================"
echo "完成: 成功 ${#ok[@]} / 跳过 ${#skipped[@]} / 失败 ${#failed[@]}"
[[ ${#ok[@]}      -gt 0 ]] && echo "  成功: ${ok[*]}"
[[ ${#skipped[@]} -gt 0 ]] && echo "  跳过: ${skipped[*]}"
[[ ${#failed[@]}  -gt 0 ]] && echo "  失败: ${failed[*]}"
echo "  输出目录: $OUT_DIR"
echo "========================================================"

[[ ${#failed[@]} -eq 0 ]]
