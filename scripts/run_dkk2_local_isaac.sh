#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/root/data1/kdi/workspace/isaac/isaac-sim}"
ISAAC_ASSETS_DIR="${ISAAC_ASSETS_DIR:-/root/data1/kdi/workspace/isaac/isaac-sim-assets}"
SCENE_PATH="${SCENE_PATH:-/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_2.usd}"
OUTPUT_PATH="${OUTPUT_PATH:-$ROOT_DIR/outputs/dkk_2_local_isaac_manifest.json}"
EXPORT_STAGE="${EXPORT_STAGE:-}"
STEPS="${STEPS:-120}"
WARMUP_STEPS="${WARMUP_STEPS:-30}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
ACTIVE_GPU="${ACTIVE_GPU:-0}"
MULTI_GPU="${MULTI_GPU:-false}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
  echo "Isaac Sim python.sh not found or not executable: $ISAAC_SIM_DIR/python.sh" >&2
  exit 1
fi

if [[ ! -f "$SCENE_PATH" ]]; then
  echo "Scene not found: $SCENE_PATH" >&2
  exit 1
fi

mkdir -p "$(dirname "$OUTPUT_PATH")"

MULTI_GPU_FLAG="--no-multi-gpu"
if [[ "$MULTI_GPU" == "true" || "$MULTI_GPU" == "1" || "$MULTI_GPU" == "yes" ]]; then
  MULTI_GPU_FLAG="--multi-gpu"
fi

EXPORT_ARGS=()
if [[ -n "$EXPORT_STAGE" ]]; then
  mkdir -p "$(dirname "$EXPORT_STAGE")"
  EXPORT_ARGS=(--export-stage "$EXPORT_STAGE")
fi

env -u CONDA_PREFIX \
  ISAAC_SIM_DIR="$ISAAC_SIM_DIR" \
  ISAAC_ASSETS_DIR="$ISAAC_ASSETS_DIR" \
  "$ISAAC_SIM_DIR/python.sh" "$ROOT_DIR/scripts/simulate_dkk2_local.py" \
    --scene "$SCENE_PATH" \
    --output "$OUTPUT_PATH" \
    "${EXPORT_ARGS[@]}" \
    --steps "$STEPS" \
    --warmup-steps "$WARMUP_STEPS" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --active-gpu "$ACTIVE_GPU" \
    "$MULTI_GPU_FLAG" \
    --headless
