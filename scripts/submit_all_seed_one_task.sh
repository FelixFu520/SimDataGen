#!/usr/bin/env bash
# 按 seed 批量提交火山任务（单 task，多 seed）。
# 用法:
#   export VOLC_AK VOLC_SK VOLC_PASSWD
#   $0 <相机名> <seed起始> <seed数量> <路径数量> <点数> <task_name> <resource_queue_id> <is_flexible> <instance_type_id> <cpu_number> <memory_number> <gpu_type> <family> <gpu_number>
#
# task_name 与 scripts 下脚本对应: scripts/${task_name}.sh
#
# 示例:
#   $0 4cam-lut 0 10 10 10 taobao02_AIUE_V01_003 q-20251110132321-bx8th True ml.gni3.48xlarge 8 48 NVIDIA-L4 ml.gni3 1

set -euo pipefail

if [[ $# -ne 14 ]]; then
  echo "用法: $0 <相机名> <seed起始> <seed数量> <路径数量> <点数> <task_name> <resource_queue_id> <is_flexible> <instance_type_id> <cpu_number> <memory_number> <gpu_type> <family> <gpu_number>" >&2
  echo "示例: $0 4cam-lut 0 10 10 10 taobao02_AIUE_V01_003 q-20251110132321-bx8th True ml.gni3.48xlarge 8 48 NVIDIA-L4 ml.gni3 1" >&2
  exit 1
fi

CAMERA_NAME_ENV=$1
SEED_START=$2
SEED_COUNT=$3
NUM_PATHS_ENV=$4
NUM_POINTS_ENV=$5
TASK_NAME=$6
RESOURCE_QUEUE_ID=$7
IS_FLEXIBLE=$8
INSTANCE_TYPE_ID=${9}
CPU_NUMBER=${10}
MEMORY_NUMBER=${11}
GPU_TYPE=${12}
FAMILY=${13}
GPU_NUMBER=${14:-1}

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
SCRIPT_DIR="${PROJECT_DIR}/scripts"
TASK_SCRIPT="${SCRIPT_DIR}/${TASK_NAME}.sh"

if [[ ! -f "${TASK_SCRIPT}" ]]; then
  echo "错误: 未找到任务脚本 ${TASK_SCRIPT}" >&2
  exit 1
fi

if [[ -z "${VOLC_AK:-}" || -z "${VOLC_SK:-}" || -z "${VOLC_PASSWD:-}" ]]; then
  echo "错误: 请设置环境变量 VOLC_AK、VOLC_SK、VOLC_PASSWD" >&2
  exit 1
fi

cd "$(dirname "$0")"

for ((i = 0; i < SEED_COUNT; i++)); do
  SEED_ENV=$((SEED_START + i))
  python submit_volcengine.py --ak "${VOLC_AK}" --sk "${VOLC_SK}" --private_image_password "${VOLC_PASSWD}" \
    --task_name "${TASK_NAME}_${CAMERA_NAME_ENV}_seed${SEED_ENV}_paths${NUM_PATHS_ENV}_points${NUM_POINTS_ENV}" \
    --command "${TASK_SCRIPT} ${SEED_ENV} ${NUM_PATHS_ENV} ${NUM_POINTS_ENV} ${CAMERA_NAME_ENV}" \
    --resource_queue_id "${RESOURCE_QUEUE_ID}" \
    --is_flexible "${IS_FLEXIBLE}" \
    --instance_type_id "${INSTANCE_TYPE_ID}" \
    --cpu_number "${CPU_NUMBER}" \
    --memory_number "${MEMORY_NUMBER}" \
    --gpu_type "${GPU_TYPE}" \
    --family "${FAMILY}" \
    --gpu_count "${GPU_NUMBER}"

done
