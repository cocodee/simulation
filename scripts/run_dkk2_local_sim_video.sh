#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

STEPS="${STEPS:-120}"
WARMUP_STEPS="${WARMUP_STEPS:-30}"
FPS="${FPS:-8}"
DURATION="${DURATION:-2}"
WIDTH="${WIDTH:-640}"
HEIGHT="${HEIGHT:-360}"
MAX_TRIANGLES="${MAX_TRIANGLES:-80000}"
CRF="${CRF:-20}"

MANIFEST_PATH="${MANIFEST_PATH:-$ROOT_DIR/outputs/dkk_2_local_sim_video_manifest.json}"
EXPORTED_STAGE="${EXPORTED_STAGE:-$ROOT_DIR/outputs/dkk_2_after_local_sim.usd}"
FRAMES_DIR="${FRAMES_DIR:-$ROOT_DIR/outputs/dkk_2_local_sim_video_frames}"
VIDEO_PATH="${VIDEO_PATH:-$ROOT_DIR/outputs/dkk_2_local_sim_video.mp4}"

STEPS="$STEPS" \
WARMUP_STEPS="$WARMUP_STEPS" \
WIDTH="$WIDTH" \
HEIGHT="$HEIGHT" \
OUTPUT_PATH="$MANIFEST_PATH" \
EXPORT_STAGE="$EXPORTED_STAGE" \
"$ROOT_DIR/scripts/run_dkk2_local_isaac.sh"

python "$ROOT_DIR/scripts/render_dkk2_software.py" \
  --scene "$EXPORTED_STAGE" \
  --output-dir "$FRAMES_DIR" \
  --video "$VIDEO_PATH" \
  --fps "$FPS" \
  --duration "$DURATION" \
  --width "$WIDTH" \
  --height "$HEIGHT" \
  --max-triangles "$MAX_TRIANGLES" \
  --crf "$CRF"

echo "[local_sim_video] manifest=$MANIFEST_PATH"
echo "[local_sim_video] exported_stage=$EXPORTED_STAGE"
echo "[local_sim_video] video=$VIDEO_PATH"
