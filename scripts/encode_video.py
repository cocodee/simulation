#!/usr/bin/env python3
"""Encode an MP4 from rgb_*.png frames."""

from __future__ import annotations

import argparse
import subprocess
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("frames_dir", help="Directory containing rgb_*.png frames.")
    parser.add_argument("--output", required=True, help="Output MP4 path.")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--crf", type=int, default=20)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    frames_dir = Path(args.frames_dir).resolve()
    output = Path(args.output).resolve()
    frames = sorted(frames_dir.glob("rgb_*.png"))
    if not frames:
        raise FileNotFoundError(f"No rgb_*.png frames found in {frames_dir}")

    list_path = frames_dir / "ffmpeg_list.txt"
    with list_path.open("w", encoding="utf-8") as file:
        for frame in frames:
            file.write(f"file '{frame.resolve()}'\n")

    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg",
        "-y" if args.overwrite else "-n",
        "-r",
        str(args.fps),
        "-f",
        "concat",
        "-safe",
        "0",
        "-i",
        str(list_path),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        str(args.crf),
        str(output),
    ]
    subprocess.run(cmd, check=True)
    print(f"Encoded {len(frames)} frames -> {output}", flush=True)


if __name__ == "__main__":
    main()
