#!/bin/bash
# 批量测试 TaoBao08 下所有 blender 资产
# 用法:
#   ./test_blender_assets.sh                # 测试全部资产, 跳过已成功的
#   ./test_blender_assets.sh -f             # 强制重新测试 (不跳过)
#   ./test_blender_assets.sh -a TaoBao09    # 指定其它资产根目录子集

set -u

# ============== 默认参数 (与 .vscode/launch.json 16-32 保持一致) ==============
ASSET_ROOT="/home/fufa/projects2026/DataGen_omni/asset_extern/TaoBao08"
WORKDIR_ROOT="/home/fufa/projects2026/DataGen_omni/workdir"
LOG_ROOT="${WORKDIR_ROOT}/_test_logs/TaoBao08_$(date +%Y%m%d_%H%M%S)"

OCCUPANCY_RESOLUTION=0.25
NUM_POINTS=1
NUM_PATHS=10
MAX_ANGLE_DEVIATION=10.0
ERODE_ITERATIONS=1
OBSTACLE_DILATE_ITERATIONS=1
OBSTACLE_ENVELOPE_ITERATIONS=20
STEP_SIZE_XY=0.3
STEP_SIZE_Z=0.1
MAX_DZ_PER_STEP=0.1

# 单个资产超时时间 (秒); 0 表示不限制
ASSET_TIMEOUT=${ASSET_TIMEOUT:-3600}

FORCE_RERUN=0

# ============== 参数解析 ==============
while [[ $# -gt 0 ]]; do
    case "$1" in
        -f|--force)
            FORCE_RERUN=1
            shift
            ;;
        -a|--asset-root)
            ASSET_ROOT="$2"
            shift 2
            ;;
        -h|--help)
            sed -n '2,8p' "$0"
            exit 0
            ;;
        *)
            echo "未知参数: $1"
            exit 1
            ;;
    esac
done

# ============== 准备工作 ==============
# 切换到项目根目录 (gen_data.py 所在目录)
cd "$(dirname "$0")" || exit 1
PROJECT_ROOT="$(pwd)"

if [[ ! -f "${PROJECT_ROOT}/gen_data.py" ]]; then
    echo "[ERROR] 找不到 gen_data.py: ${PROJECT_ROOT}/gen_data.py"
    exit 1
fi

if [[ ! -d "${ASSET_ROOT}" ]]; then
    echo "[ERROR] 资产根目录不存在: ${ASSET_ROOT}"
    exit 1
fi

PYTHON_BIN="${PROJECT_ROOT}/app/python.sh"
if [[ ! -x "${PYTHON_BIN}" ]]; then
    PYTHON_BIN="${PROJECT_ROOT}/app/kit/python/bin/python3"
fi

mkdir -p "${LOG_ROOT}"
SUMMARY_FILE="${LOG_ROOT}/summary.csv"
CRASH_FILE="${LOG_ROOT}/crashed.txt"
FAIL_FILE="${LOG_ROOT}/failed.txt"
TIMEOUT_FILE="${LOG_ROOT}/timeout.txt"
echo "asset_name,status,exit_code,duration_sec,usd_path,log_file" > "${SUMMARY_FILE}"
: > "${CRASH_FILE}"
: > "${FAIL_FILE}"
: > "${TIMEOUT_FILE}"

# 用户 Ctrl-C 时优雅退出整个批处理 (默认信号会被传给前台子进程, 单次中断也会中断当前资产)
ABORTED=0
trap 'echo ""; echo "[ABORT] 用户中断, 停止后续资产测试"; ABORTED=1' INT TERM

echo "================================================================"
echo "项目根目录   : ${PROJECT_ROOT}"
echo "资产根目录   : ${ASSET_ROOT}"
echo "输出根目录   : ${WORKDIR_ROOT}"
echo "日志根目录   : ${LOG_ROOT}"
echo "Python 解释器: ${PYTHON_BIN}"
echo "强制重跑     : ${FORCE_RERUN}"
echo "单资产超时(s): ${ASSET_TIMEOUT}"
echo "================================================================"

# ============== 主循环 ==============
TOTAL=0
SUCCESS=0
FAIL=0
CRASH=0
TIMEOUT_CNT=0
SKIP=0

# 按名称排序; 使用 -- 防止以 - 开头的目录
for ASSET_DIR in "${ASSET_ROOT}"/*/; do
    [[ ${ABORTED} -eq 1 ]] && break
    [[ -d "${ASSET_DIR}" ]] || continue
    ASSET_NAME="$(basename "${ASSET_DIR}")"

    # 1) 优先用同名 usd; 否则取目录下第一个 *.usd / *.usdc / *.usda
    SCENE_USD_URL=""
    for cand in \
        "${ASSET_DIR}${ASSET_NAME}.usd" \
        "${ASSET_DIR}${ASSET_NAME}.usdc" \
        "${ASSET_DIR}${ASSET_NAME}.usda"; do
        if [[ -f "${cand}" ]]; then
            SCENE_USD_URL="${cand}"
            break
        fi
    done
    if [[ -z "${SCENE_USD_URL}" ]]; then
        SCENE_USD_URL="$(ls "${ASSET_DIR}"*.usd "${ASSET_DIR}"*.usdc "${ASSET_DIR}"*.usda 2>/dev/null | head -n1)"
    fi

    TOTAL=$((TOTAL + 1))

    if [[ -z "${SCENE_USD_URL}" ]]; then
        echo "[SKIP] [${TOTAL}] ${ASSET_NAME}: 找不到 .usd / .usdc / .usda 文件"
        echo "${ASSET_NAME},no_usd,,0,," >> "${SUMMARY_FILE}"
        SKIP=$((SKIP + 1))
        continue
    fi

    OUTPUT_DIR="${WORKDIR_ROOT}/${ASSET_NAME}"
    LOG_FILE="${LOG_ROOT}/${ASSET_NAME}.log"
    OK_MARK="${OUTPUT_DIR}/.test_ok"

    if [[ ${FORCE_RERUN} -eq 0 && -f "${OK_MARK}" ]]; then
        echo "[SKIP] [${TOTAL}] ${ASSET_NAME}: 已成功测试过 (${OK_MARK})"
        echo "${ASSET_NAME},skipped,,0,${SCENE_USD_URL},${OK_MARK}" >> "${SUMMARY_FILE}"
        SKIP=$((SKIP + 1))
        continue
    fi

    echo ""
    echo "----------------------------------------------------------------"
    echo "[RUN ] [${TOTAL}] ${ASSET_NAME}"
    echo "  usd : ${SCENE_USD_URL}"
    echo "  out : ${OUTPUT_DIR}"
    echo "  log : ${LOG_FILE}"
    echo "----------------------------------------------------------------"

    mkdir -p "${OUTPUT_DIR}"
    rm -f "${OK_MARK}"

    START_TS=$(date +%s)

    CMD=(
        "${PYTHON_BIN}" gen_data.py
        --scene_usd_url "${SCENE_USD_URL}"
        --output_dir    "${OUTPUT_DIR}"
        --occupancy_resolution "${OCCUPANCY_RESOLUTION}"
        --num_points    "${NUM_POINTS}"
        --num_paths     "${NUM_PATHS}"
        --max_angle_deviation "${MAX_ANGLE_DEVIATION}"
        --erode_iterations "${ERODE_ITERATIONS}"
        --obstacle_dilate_iterations "${OBSTACLE_DILATE_ITERATIONS}"
        --obstacle_envelope_iterations "${OBSTACLE_ENVELOPE_ITERATIONS}"
        --step_size_xy  "${STEP_SIZE_XY}"
        --step_size_z   "${STEP_SIZE_Z}"
        --max_dz_per_step "${MAX_DZ_PER_STEP}"
    )

    # 用子 shell + setsid 隔离, 即使 gen_data.py 段错误/被信号杀掉也只影响这一个资产
    # timeout -k 30s : SIGTERM 后再等 30s, 仍未退出则发 SIGKILL, 避免僵死阻塞批处理
    RET=0
    if [[ "${ASSET_TIMEOUT}" -gt 0 ]]; then
        setsid -w timeout -k 30s --foreground "${ASSET_TIMEOUT}" "${CMD[@]}" \
            >"${LOG_FILE}" 2>&1 || RET=$?
    else
        setsid -w "${CMD[@]}" >"${LOG_FILE}" 2>&1 || RET=$?
    fi

    END_TS=$(date +%s)
    DURATION=$((END_TS - START_TS))

    # 退出码分类:
    #   0          : 成功
    #   124        : timeout 触发的超时
    #   137        : SIGKILL (常见: OOM 杀掉, 或 timeout -k 升级杀掉)
    #   134/139/.. : 被信号终止的崩溃 (SIGABRT/SIGSEGV 等), >=128
    #   其它非 0   : 普通失败 (Python 异常等)
    if [[ ${RET} -eq 0 ]]; then
        touch "${OK_MARK}"
        echo "[ OK  ] ${ASSET_NAME} (${DURATION}s)"
        echo "${ASSET_NAME},ok,0,${DURATION},${SCENE_USD_URL},${LOG_FILE}" >> "${SUMMARY_FILE}"
        SUCCESS=$((SUCCESS + 1))
    elif [[ ${RET} -eq 124 ]]; then
        echo "[TIME ] ${ASSET_NAME} 超时 (${ASSET_TIMEOUT}s)"
        echo "${ASSET_NAME},timeout,${RET},${DURATION},${SCENE_USD_URL},${LOG_FILE}" >> "${SUMMARY_FILE}"
        echo "${ASSET_NAME}	${SCENE_USD_URL}	${LOG_FILE}" >> "${TIMEOUT_FILE}"
        TIMEOUT_CNT=$((TIMEOUT_CNT + 1))
    elif [[ ${RET} -ge 128 ]]; then
        SIG=$((RET - 128))
        echo "[CRASH] ${ASSET_NAME} 被信号 ${SIG} 杀掉 (exit=${RET}), 用时 ${DURATION}s, 详见 ${LOG_FILE}"
        echo "${ASSET_NAME},crash_sig${SIG},${RET},${DURATION},${SCENE_USD_URL},${LOG_FILE}" >> "${SUMMARY_FILE}"
        echo "${ASSET_NAME}	sig=${SIG}	${SCENE_USD_URL}	${LOG_FILE}" >> "${CRASH_FILE}"
        CRASH=$((CRASH + 1))
    else
        echo "[FAIL ] ${ASSET_NAME} 退出码=${RET}, 用时 ${DURATION}s, 详见 ${LOG_FILE}"
        echo "${ASSET_NAME},fail,${RET},${DURATION},${SCENE_USD_URL},${LOG_FILE}" >> "${SUMMARY_FILE}"
        echo "${ASSET_NAME}	exit=${RET}	${SCENE_USD_URL}	${LOG_FILE}" >> "${FAIL_FILE}"
        FAIL=$((FAIL + 1))
    fi
done

# ============== 汇总 ==============
echo ""
echo "================================================================"
echo "测试完成"
echo "  总计     : ${TOTAL}"
echo "  成功     : ${SUCCESS}"
echo "  失败     : ${FAIL}     (列表: ${FAIL_FILE})"
echo "  崩溃     : ${CRASH}    (列表: ${CRASH_FILE})"
echo "  超时     : ${TIMEOUT_CNT}    (列表: ${TIMEOUT_FILE})"
echo "  跳过     : ${SKIP}"
echo "  汇总表   : ${SUMMARY_FILE}"
[[ ${ABORTED} -eq 1 ]] && echo "  状态     : 用户中断"
echo "================================================================"

if [[ ${CRASH} -gt 0 ]]; then
    echo ""
    echo "[崩溃场景]"
    cat "${CRASH_FILE}"
fi

if [[ $((FAIL + CRASH + TIMEOUT_CNT)) -gt 0 ]]; then
    exit 1
fi
exit 0
