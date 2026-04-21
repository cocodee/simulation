#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/root/data1/kdi/workspace/isaac/isaac-sim}"
SCENE_PATH="${SCENE_PATH:-/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_2.usd}"
OUTPUT_DIR="${OUTPUT_DIR:-$ROOT_DIR/outputs/dkk_2_motion_frames}"
VIDEO_PATH="${VIDEO_PATH:-$ROOT_DIR/outputs/dkk_2_motion_video.mp4}"
MANIFEST_PATH="${MANIFEST_PATH:-$ROOT_DIR/outputs/dkk_2_motion_manifest.json}"
FRAMES="${FRAMES:-144}"
WARMUP_STEPS="${WARMUP_STEPS:-30}"
FPS="${FPS:-24}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
ACTIVE_GPU="${ACTIVE_GPU:-0}"
MULTI_GPU="${MULTI_GPU:-false}"
CAMERA_POSITION="${CAMERA_POSITION:--4.0,4.0,2.8}"
LOOK_AT="${LOOK_AT:--1.6,0.3,1.1}"
WHEEL_SPEED="${WHEEL_SPEED:-7.5}"
TURN_SPEED="${TURN_SPEED:-3.6}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
  echo "Isaac Sim python.sh not found or not executable: $ISAAC_SIM_DIR/python.sh" >&2
  exit 1
fi

if [[ ! -f "$SCENE_PATH" ]]; then
  echo "Scene not found: $SCENE_PATH" >&2
  exit 1
fi

mkdir -p "$OUTPUT_DIR" "$(dirname "$VIDEO_PATH")" "$(dirname "$MANIFEST_PATH")"

MULTI_GPU_FLAG="--no-multi-gpu"
if [[ "$MULTI_GPU" == "true" || "$MULTI_GPU" == "1" || "$MULTI_GPU" == "yes" ]]; then
  MULTI_GPU_FLAG="--multi-gpu"
fi

env -u CONDA_PREFIX \
  "$ISAAC_SIM_DIR/python.sh" "$ROOT_DIR/scripts/record_dkk2_motion_video.py" \
    --scene "$SCENE_PATH" \
    --output-dir "$OUTPUT_DIR" \
    --video "$VIDEO_PATH" \
    --manifest "$MANIFEST_PATH" \
    --frames "$FRAMES" \
    --warmup-steps "$WARMUP_STEPS" \
    --fps "$FPS" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --active-gpu "$ACTIVE_GPU" \
    --wheel-speed "$WHEEL_SPEED" \
    --turn-speed "$TURN_SPEED" \
    "$MULTI_GPU_FLAG" \
    --camera-position="$CAMERA_POSITION" \
    --look-at="$LOOK_AT" \
    --headless
