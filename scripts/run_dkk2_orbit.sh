#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "$ROOT_DIR/.." && pwd)"

IMAGE="${IMAGE:-wheel-loading-sim:latest}"
BACKEND="${BACKEND:-software}"
CONTAINER_WORKSPACE="${CONTAINER_WORKSPACE:-/workspace}"
SCENE_PATH="${SCENE_PATH:-$CONTAINER_WORKSPACE/supre_robot_assets/scenes/dkk_2.usd}"
HOST_SCENE_PATH="${HOST_SCENE_PATH:-$WORKSPACE_DIR/supre_robot_assets/scenes/dkk_2.usd}"
OUTPUT_DIR="${OUTPUT_DIR:-$ROOT_DIR/outputs/dkk_2_orbit_frames}"
VIDEO_PATH="${VIDEO_PATH:-$ROOT_DIR/outputs/dkk_2_orbit.mp4}"
FPS="${FPS:-24}"
DURATION="${DURATION:-8}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
CRF="${CRF:-20}"
WARMUP_STEPS="${WARMUP_STEPS:-90}"
ORBIT_DEGREES="${ORBIT_DEGREES:-360}"
MAX_TRIANGLES="${MAX_TRIANGLES:-80000}"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

CONTAINER_OUTPUT_DIR="$CONTAINER_WORKSPACE/sim1/outputs/$(basename "$OUTPUT_DIR")"

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [record|encode|all]

Environment variables:
  IMAGE          Isaac Sim Docker image. Default: $IMAGE
  BACKEND        Render backend: software or isaac. Default: $BACKEND
  FPS            Output video fps. Default: $FPS
  DURATION       Video duration in seconds. Default: $DURATION
  WIDTH          Frame width. Default: $WIDTH
  HEIGHT         Frame height. Default: $HEIGHT
  OUTPUT_DIR     Host frame output directory. Default: $OUTPUT_DIR
  VIDEO_PATH     Host MP4 output path. Default: $VIDEO_PATH
  CRF            libx264 CRF. Default: $CRF
  WARMUP_STEPS   Isaac warmup steps. Default: $WARMUP_STEPS
  ORBIT_DEGREES  Camera orbit angle. Default: $ORBIT_DEGREES
  MAX_TRIANGLES  Software renderer triangle budget. Default: $MAX_TRIANGLES

Examples:
  DURATION=2 FPS=12 $(basename "$0")
  VIDEO_PATH=/tmp/dkk_2.mp4 $(basename "$0") all
EOF
}

run_record() {
  mkdir -p "$OUTPUT_DIR"
  if [[ "$BACKEND" == "software" ]]; then
    python "$ROOT_DIR/scripts/render_dkk2_software.py" \
      --scene "$HOST_SCENE_PATH" \
      --output-dir "$OUTPUT_DIR" \
      --video "$VIDEO_PATH" \
      --fps "$FPS" \
      --duration "$DURATION" \
      --width "$WIDTH" \
      --height "$HEIGHT" \
      --orbit-degrees "$ORBIT_DEGREES" \
      --max-triangles "$MAX_TRIANGLES" \
      --crf "$CRF"
    return
  fi

  if [[ "$BACKEND" != "isaac" ]]; then
    echo "Unknown BACKEND: $BACKEND" >&2
    exit 2
  fi

  docker run --rm --gpus all --network host \
    --entrypoint bash \
    --user root \
    -v "$WORKSPACE_DIR:$CONTAINER_WORKSPACE" \
    -w "$CONTAINER_WORKSPACE/sim1" \
    "$IMAGE" \
    -lc "/isaac-sim/python.sh scripts/record_dkk2_orbit.py \
      --scene '$SCENE_PATH' \
      --output-dir '$CONTAINER_OUTPUT_DIR' \
      --fps '$FPS' \
      --duration '$DURATION' \
      --width '$WIDTH' \
      --height '$HEIGHT' \
      --warmup-steps '$WARMUP_STEPS' \
      --orbit-degrees '$ORBIT_DEGREES' \
      --headless && \
      chown -R '$HOST_UID:$HOST_GID' '$CONTAINER_OUTPUT_DIR'"
}

run_encode() {
  python "$ROOT_DIR/scripts/encode_video.py" "$OUTPUT_DIR" \
    --output "$VIDEO_PATH" \
    --fps "$FPS" \
    --crf "$CRF" \
    --overwrite
}

MODE="${1:-all}"
case "$MODE" in
  all)
    run_record
    if [[ "$BACKEND" == "isaac" ]]; then
      run_encode
    fi
    ;;
  record)
    run_record
    ;;
  encode)
    run_encode
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown mode: $MODE" >&2
    usage >&2
    exit 2
    ;;
esac
