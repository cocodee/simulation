#!/usr/bin/env python3
"""Render dkk_2.usd with Isaac Sim Replicator in headless mode."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import time
from pathlib import Path

os.environ.setdefault("PYTHONUNBUFFERED", "1")

try:
    from isaacsim.simulation_app import SimulationApp
except ModuleNotFoundError:  # pragma: no cover - compatibility with older Isaac Sim.
    from omni.isaac.kit import SimulationApp


def parse_tuple(value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Expected three comma-separated values, e.g. -3,3,2.5")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        default="/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_2.usd",
        help="Absolute path to dkk_2.usd.",
    )
    parser.add_argument(
        "--output-dir",
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_2_replicator_frames",
        help="Directory where Replicator BasicWriter will write rgb_*.png frames.",
    )
    parser.add_argument(
        "--video",
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_2_replicator_video.mp4",
        help="MP4 output path.",
    )
    parser.add_argument(
        "--manifest",
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_2_replicator_manifest.json",
        help="JSON manifest path.",
    )
    parser.add_argument("--frames", type=int, default=96)
    parser.add_argument("--warmup-steps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--crf", type=int, default=18)
    parser.add_argument("--camera-position", type=parse_tuple, default=(-4.0, 4.0, 2.8))
    parser.add_argument("--look-at", type=parse_tuple, default=(-1.6, 0.3, 1.1))
    parser.add_argument("--reference-prim", default="/World/DKK2")
    parser.add_argument("--dome-intensity", type=float, default=1800.0)
    parser.add_argument("--sun-intensity", type=float, default=900.0)
    parser.add_argument("--active-gpu", type=int, default=0)
    parser.add_argument("--multi-gpu", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--no-video", action="store_true", help="Only write frames and manifest.")
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
        "width": args.width,
        "height": args.height,
        "active_gpu": args.active_gpu,
        "multi_gpu": args.multi_gpu,
    }
)

import carb  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402


def log(message: str) -> None:
    print(f"[dkk2_replicator] {message}", flush=True)


def configure_render_settings() -> None:
    settings = carb.settings.get_settings()
    settings.set("/rtx/pathtracing/spp", 64)
    settings.set("/rtx/pathtracing/totalSpp", 256)
    settings.set("/rtx/pathtracing/optixDenoiser/enabled", True)
    settings.set("/rtx/post/dlss/execMode", 1)
    settings.set("/rtx/post/aa/op", 2)


def add_reference(stage, scene_path: Path, prim_path: str) -> None:
    log(f"adding_reference={scene_path} target={prim_path}")
    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(scene_path))
    stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))
    log("reference_added")


def add_lights(stage) -> None:
    log("adding_lights")
    dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(args.dome_intensity)

    sun_light = stage.DefinePrim("/World/DirectionalLight", "DistantLight")
    sun_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(args.sun_intensity)
    sun_light.CreateAttribute("inputs:angle", Sdf.ValueTypeNames.Float).Set(0.5)
    log("lights_added")


def count_scene(stage) -> tuple[int, int]:
    prim_count = 0
    mesh_count = 0
    for prim in stage.Traverse():
        prim_count += 1
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
    return prim_count, mesh_count


def encode_video(frames_dir: Path, video_path: Path, max_frames: int | None = None) -> int:
    frames = sorted(frames_dir.glob("rgb_*.png"))
    if not frames:
        raise FileNotFoundError(f"No rgb_*.png frames found in {frames_dir}")
    if max_frames is not None and len(frames) > max_frames:
        frames = frames[-max_frames:]

    list_path = frames_dir / "ffmpeg_list.txt"
    with list_path.open("w", encoding="utf-8") as file:
        for frame in frames:
            file.write(f"file '{frame.resolve()}'\n")

    video_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg",
        "-y",
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
        str(video_path),
    ]
    subprocess.run(cmd, check=True)
    return len(frames)


def clean_output_dir(output_dir: Path) -> None:
    for pattern in ("rgb_*.png", "ffmpeg_list.txt"):
        for path in output_dir.glob(pattern):
            if path.is_file():
                path.unlink()


def main() -> None:
    scene_path = Path(args.scene).resolve()
    output_dir = Path(args.output_dir).resolve()
    video_path = Path(args.video).resolve()
    manifest_path = Path(args.manifest).resolve()

    if not scene_path.exists():
        raise FileNotFoundError(f"Scene not found: {scene_path}")

    output_dir.mkdir(parents=True, exist_ok=True)
    clean_output_dir(output_dir)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)

    configure_render_settings()

    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    add_lights(stage)
    add_reference(stage, scene_path, args.reference_prim)

    log(f"scene_reference={scene_path} prim={args.reference_prim}")
    for _ in range(args.warmup_steps):
        simulation_app.update()

    prim_count, mesh_count = count_scene(stage)
    log(f"prims={prim_count} meshes={mesh_count}")

    camera = rep.create.camera(position=args.camera_position, look_at=args.look_at)
    render_product = rep.create.render_product(camera, (args.width, args.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(output_dir), rgb=True)

    world.reset()
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(args.warmup_steps):
        simulation_app.update()

    writer.attach([render_product])
    start = time.time()
    for index in range(args.frames):
        simulation_app.update()
        rep.orchestrator.step()
        if index == 0 or (index + 1) % max(1, args.frames // 6) == 0:
            log(f"captured {index + 1}/{args.frames}")

    rep.orchestrator.wait_until_complete()
    timeline.stop()
    simulation_app.update()
    elapsed = time.time() - start

    frame_count = len(list(output_dir.glob("rgb_*.png")))
    encoded_frame_count = 0
    encoded_video = None
    if not args.no_video:
        encoded_frame_count = encode_video(output_dir, video_path, max_frames=args.frames)
        encoded_video = str(video_path)
        log(f"video={video_path}")

    manifest = {
        "scene": str(scene_path),
        "output_dir": str(output_dir),
        "video": encoded_video,
        "manifest": str(manifest_path),
        "reference_prim": args.reference_prim,
        "prim_count": prim_count,
        "mesh_count": mesh_count,
        "frame_count": frame_count,
        "encoded_frame_count": encoded_frame_count,
        "requested_frames": args.frames,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "warmup_steps": args.warmup_steps,
        "elapsed_seconds": elapsed,
        "camera_position": args.camera_position,
        "look_at": args.look_at,
        "headless": args.headless,
        "active_gpu": args.active_gpu,
        "multi_gpu": args.multi_gpu,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    log(f"manifest={manifest_path}")


try:
    main()
finally:
    simulation_app.close()
