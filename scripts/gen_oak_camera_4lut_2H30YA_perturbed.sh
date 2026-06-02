#!/usr/bin/env bash
# 一键生成 OAK 4lut + 2H30YA 扰动相机资产，输出到 assets/cameras/perturbed_camera/
#
# Usage:
#   ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant small_change [--seed 0]
#   ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant pinhole_like --seed 1
#   ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant fisheye_like
#   ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant extrinsics_change --seed 0
#   ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant small_change --with-extrinsics --seed 0
#   ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant small_change_extrinsics --seed 0
#
# 输出（每次运行前清空 perturbed_camera/）:
#   assets/cameras/perturbed_camera/fisheye_cams.yaml
#   assets/cameras/perturbed_camera/texture/*.exr
#   assets/cameras/perturbed_camera/oak_camera_4lut_2H30YA_perturbed.usd
#   assets/cameras/perturbed_camera/profile.txt

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PYTHON="${ROOT}/app/python.sh"
BASE_USD="${ROOT}/assets/cameras/oak_camera_4lut_2H30YA.usd"
BASE_TEXTURE="${ROOT}/assets/cameras/oak_camera_texture"
OUT_DIR="${ROOT}/assets/cameras/perturbed_camera"
OUT_USD="${OUT_DIR}/oak_camera_4lut_2H30YA_perturbed.usd"
OUT_YAML="${OUT_DIR}/fisheye_cams.yaml"
OUT_TEXTURE="${OUT_DIR}/texture"
PROFILE_FILE="${OUT_DIR}/profile.txt"

VARIANT=""
SEED=0
WITH_EXTRINSICS=0

usage() {
  cat <<'EOF'
Usage: gen_oak_camera_4lut_2H30YA_perturbed.sh --variant NAME [options]

Options:
  --variant NAME       扰动类别（必填）:
                         small_change | pinhole_like | fisheye_like
                         extrinsics_change
                         small_change_extrinsics | pinhole_like_extrinsics | fisheye_like_extrinsics
  --seed N             外参随机种子，默认 0（仅 extrinsics_change / --with-extrinsics 时使用）
  --with-extrinsics    在内参 profile 上叠加外参小幅随机扰动
  -h, --help           显示帮助

示例:
  ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant small_change
  ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant extrinsics_change --seed 42
  ./scripts/gen_oak_camera_4lut_2H30YA_perturbed.sh --variant pinhole_like --with-extrinsics --seed 0
EOF
}

log() { echo "[gen_perturbed] $*" >&2; }
die() { echo "[gen_perturbed] ERROR: $*" >&2; exit 1; }

run_py() {
  log ">>> $PYTHON $*"
  "$PYTHON" "$@"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --variant)
      VARIANT="${2:-}"
      shift 2
      ;;
    --seed)
      SEED="${2:-}"
      shift 2
      ;;
    --with-extrinsics)
      WITH_EXTRINSICS=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1 (use --help)"
      ;;
  esac
done

[[ -n "$VARIANT" ]] || { usage; die "--variant is required"; }
[[ -f "$PYTHON" ]] || die "missing $PYTHON (Isaac Sim python wrapper)"
[[ -f "$BASE_USD" ]] || die "missing base USD: $BASE_USD"
[[ -d "$BASE_TEXTURE" ]] || die "missing base LUT dir: $BASE_TEXTURE"

# 解析 variant → 内参 profile / 是否 bake 外参
INTRINSICS_PROFILE=""
DO_EXTRINSICS=0
DO_INTRINSICS_BAKE=1
DO_LUT_GEN=1

case "$VARIANT" in
  small_change|pinhole_like|fisheye_like)
    INTRINSICS_PROFILE="$VARIANT"
    DO_EXTRINSICS=$WITH_EXTRINSICS
    ;;
  extrinsics_change)
    DO_EXTRINSICS=1
    DO_INTRINSICS_BAKE=0
    DO_LUT_GEN=0
    WITH_EXTRINSICS=0
    ;;
  small_change_extrinsics|pinhole_like_extrinsics|fisheye_like_extrinsics)
    INTRINSICS_PROFILE="${VARIANT%_extrinsics}"
    DO_EXTRINSICS=1
    WITH_EXTRINSICS=1
    ;;
  *)
    die "invalid --variant: $VARIANT"
    ;;
esac

if [[ "$VARIANT" == "extrinsics_change" && "$WITH_EXTRINSICS" -eq 1 ]]; then
  die "--with-extrinsics cannot combine with --variant extrinsics_change"
fi

log "variant=$VARIANT intrinsics_profile=${INTRINSICS_PROFILE:-none} seed=$SEED do_extrinsics=$DO_EXTRINSICS"

# 清空并重建输出目录
log "clean $OUT_DIR"
rm -rf "$OUT_DIR"
mkdir -p "$OUT_TEXTURE"

# ① 生成扰动 yaml
YAML_ARGS=(tools/cameras/oak_generate_perturbed_yaml.py --output "$OUT_YAML")
if [[ "$VARIANT" == "extrinsics_change" ]]; then
  YAML_ARGS+=(--profile extrinsics_change --seed "$SEED")
elif [[ "$DO_EXTRINSICS" -eq 1 ]]; then
  YAML_ARGS+=(--profile "$INTRINSICS_PROFILE" --perturb-extrinsics --seed "$SEED")
else
  YAML_ARGS+=(--profile "$INTRINSICS_PROFILE")
fi
run_py "${YAML_ARGS[@]}"

# ② LUT EXR
if [[ "$DO_LUT_GEN" -eq 1 ]]; then
  run_py tools/cameras/oak_generate_lut_textures.py \
    --yaml "$OUT_YAML" \
    --output_dir "$OUT_TEXTURE"
else
  log "copy base LUT textures (extrinsics_change)"
  cp -a "$BASE_TEXTURE/." "$OUT_TEXTURE/"
fi

# ③ 复制 USD
log "copy base USD -> $OUT_USD"
cp "$BASE_USD" "$OUT_USD"

# ③.2 bake 内参
if [[ "$DO_INTRINSICS_BAKE" -eq 1 ]]; then
  run_py tools/cameras/oak_bake_camera_intrinsics.py \
    --usd "$OUT_USD" \
    --yaml "$OUT_YAML" \
    --texture_dir "$OUT_TEXTURE" \
    --mask_center calibration \
    --resolution CAM_Front=1920x1200 \
    --resolution CAM_Back=1920x1200
fi

# ③.3 LUT 路径（相对 USD 目录 -> texture/）
run_py tools/cameras/oak_set_camera_lut_texture_paths.py \
  --usd "$OUT_USD" \
  --texture_dir "$OUT_TEXTURE"

# ③.4 bake 外参
if [[ "$DO_EXTRINSICS" -eq 1 ]]; then
  run_py tools/cameras/oak_bake_camera_extrinsics.py \
    --usd "$OUT_USD" \
    --yaml "$OUT_YAML" \
    --perturb-pinholes \
    --seed "$SEED"
fi

# 记录本次生成参数
cat > "$PROFILE_FILE" <<EOF
variant=${VARIANT}
intrinsics_profile=${INTRINSICS_PROFILE:-none}
seed=${SEED}
with_extrinsics=${WITH_EXTRINSICS}
do_extrinsics_bake=${DO_EXTRINSICS}
usd=${OUT_USD#${ROOT}/}
yaml=${OUT_YAML#${ROOT}/}
texture_dir=${OUT_TEXTURE#${ROOT}/}
generated_at=$(date -Iseconds)
EOF

log "done"
log "  profile: $PROFILE_FILE"
log "  yaml:    $OUT_YAML"
log "  texture: $OUT_TEXTURE"
log "  usd:     $OUT_USD"
log "gen_data: --camera_usd_url $OUT_USD"
