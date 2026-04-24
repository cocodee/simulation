#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

ISAAC_SIM_DIR="${ISAAC_SIM_DIR:-/root/data1/kdi/workspace/isaac/isaac-sim}"
SCENE_PATH="${SCENE_PATH:-/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5.usd}"
RUN_STAMP="${RUN_STAMP:-$(date -u +%Y%m%d_%H%M%S)}"
RUN_DIR="${RUN_DIR:-$ROOT_DIR/outputs/dkk_5_teleop_${RUN_STAMP}}"
OUTPUT_DIR="${OUTPUT_DIR:-$RUN_DIR/frames}"
VIDEO_PATH="${VIDEO_PATH:-$RUN_DIR/dkk_5_teleop_video.mp4}"
MANIFEST_PATH="${MANIFEST_PATH:-$RUN_DIR/dkk_5_teleop_manifest.json}"
ACTION_LOG_PATH="${ACTION_LOG_PATH:-$RUN_DIR/dkk_5_teleop_actions.jsonl}"
FPS="${FPS:-24}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
ACTIVE_GPU="${ACTIVE_GPU:-0}"
MULTI_GPU="${MULTI_GPU:-false}"
CAMERA_POSITION="${CAMERA_POSITION:--4.0,4.0,2.8}"
LOOK_AT="${LOOK_AT:--1.6,0.3,1.1}"
MAX_LINEAR_SPEED="${MAX_LINEAR_SPEED:-0.25}"
MAX_ANGULAR_SPEED="${MAX_ANGULAR_SPEED:-0.8}"
EE_LINEAR_STEP="${EE_LINEAR_STEP:-0.015}"
EE_ANGULAR_STEP="${EE_ANGULAR_STEP:-0.06}"

if [[ ! -x "$ISAAC_SIM_DIR/python.sh" ]]; then
  echo "Isaac Sim python.sh not found or not executable: $ISAAC_SIM_DIR/python.sh" >&2
  exit 1
fi

if [[ ! -f "$SCENE_PATH" ]]; then
  echo "Scene not found: $SCENE_PATH" >&2
  exit 1
fi

mkdir -p "$RUN_DIR" "$OUTPUT_DIR" "$(dirname "$VIDEO_PATH")" "$(dirname "$MANIFEST_PATH")" "$(dirname "$ACTION_LOG_PATH")"

MULTI_GPU_FLAG="--no-multi-gpu"
if [[ "$MULTI_GPU" == "true" || "$MULTI_GPU" == "1" || "$MULTI_GPU" == "yes" ]]; then
  MULTI_GPU_FLAG="--multi-gpu"
fi

echo "run_dir=$RUN_DIR"
echo "video=$VIDEO_PATH"
echo "manifest=$MANIFEST_PATH"
echo "action_log=$ACTION_LOG_PATH"

env -u CONDA_PREFIX \
  "$ISAAC_SIM_DIR/python.sh" "$ROOT_DIR/scripts/teleop_dkk5_diff_video.py" \
    --scene="$SCENE_PATH" \
    --output-dir="$OUTPUT_DIR" \
    --video="$VIDEO_PATH" \
    --manifest="$MANIFEST_PATH" \
    --action-log="$ACTION_LOG_PATH" \
    --fps="$FPS" \
    --width="$WIDTH" \
    --height="$HEIGHT" \
    --active-gpu="$ACTIVE_GPU" \
    --max-linear-speed="$MAX_LINEAR_SPEED" \
    --max-angular-speed="$MAX_ANGULAR_SPEED" \
    --ee-linear-step="$EE_LINEAR_STEP" \
    --ee-angular-step="$EE_ANGULAR_STEP" \
    "$MULTI_GPU_FLAG" \
    --camera-position="$CAMERA_POSITION" \
    --look-at="$LOOK_AT" \
    --no-headless
