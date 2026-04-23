#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/root/data1/kdi/workspace/isaac/isaac-sim}"
SCENE_PATH="${SCENE_PATH:-/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5.usd}"
RUN_STAMP="${RUN_STAMP:-$(date -u +%Y%m%d_%H%M%S)}"
RUN_DIR="${RUN_DIR:-$ROOT_DIR/outputs/dkk_5_diff_motion_${RUN_STAMP}}"
OUTPUT_DIR="${OUTPUT_DIR:-$RUN_DIR/frames}"
VIDEO_PATH="${VIDEO_PATH:-$RUN_DIR/dkk_5_diff_motion_video.mp4}"
MANIFEST_PATH="${MANIFEST_PATH:-$RUN_DIR/dkk_5_diff_motion_manifest.json}"
FRAMES="${FRAMES:-480}"
WARMUP_STEPS="${WARMUP_STEPS:-30}"
FPS="${FPS:-24}"
SETTLE_SECONDS="${SETTLE_SECONDS:-5}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
ACTIVE_GPU="${ACTIVE_GPU:-0}"
MULTI_GPU="${MULTI_GPU:-false}"
CAMERA_POSITION="${CAMERA_POSITION:--4.0,4.0,2.8}"
LOOK_AT="${LOOK_AT:--1.6,0.3,1.1}"
WHEEL_SPEED="${WHEEL_SPEED:-0.8}"
ANGULAR_SPEED="${ANGULAR_SPEED:-0.0}"
WHEEL_RADIUS="${WHEEL_RADIUS:-0.075}"
WHEEL_BASE="${WHEEL_BASE:-0.6284}"
MAX_WHEEL_SPEED="${MAX_WHEEL_SPEED:-2.5}"
ANIMATE_ARMS="${ANIMATE_ARMS:-false}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
  echo "Isaac Sim python.sh not found or not executable: $ISAAC_SIM_DIR/python.sh" >&2
  exit 1
fi

if [[ ! -f "$SCENE_PATH" ]]; then
  echo "Scene not found: $SCENE_PATH" >&2
  exit 1
fi

mkdir -p "$RUN_DIR" "$OUTPUT_DIR" "$(dirname "$VIDEO_PATH")" "$(dirname "$MANIFEST_PATH")"

MULTI_GPU_FLAG="--no-multi-gpu"
if [[ "$MULTI_GPU" == "true" || "$MULTI_GPU" == "1" || "$MULTI_GPU" == "yes" ]]; then
  MULTI_GPU_FLAG="--multi-gpu"
fi

ANIMATE_ARMS_FLAG="--no-animate-arms"
if [[ "$ANIMATE_ARMS" == "true" || "$ANIMATE_ARMS" == "1" || "$ANIMATE_ARMS" == "yes" ]]; then
  ANIMATE_ARMS_FLAG="--animate-arms"
fi

echo "run_dir=$RUN_DIR"
echo "video=$VIDEO_PATH"
echo "manifest=$MANIFEST_PATH"

env -u CONDA_PREFIX \
  "$ISAAC_SIM_DIR/python.sh" "$ROOT_DIR/scripts/record_dkk5_diff_motion_video.py" \
    --scene "$SCENE_PATH" \
    --output-dir "$OUTPUT_DIR" \
    --video "$VIDEO_PATH" \
    --manifest "$MANIFEST_PATH" \
    --frames "$FRAMES" \
    --warmup-steps "$WARMUP_STEPS" \
    --fps "$FPS" \
    --settle-seconds "$SETTLE_SECONDS" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --active-gpu "$ACTIVE_GPU" \
    --wheel-speed "$WHEEL_SPEED" \
    --angular-speed "$ANGULAR_SPEED" \
    --wheel-radius "$WHEEL_RADIUS" \
    --wheel-base "$WHEEL_BASE" \
    --max-wheel-speed "$MAX_WHEEL_SPEED" \
    "$MULTI_GPU_FLAG" \
    "$ANIMATE_ARMS_FLAG" \
    --camera-position="$CAMERA_POSITION" \
    --look-at="$LOOK_AT" \
    --headless
