#!/usr/bin/env python3
"""Record an orbiting Isaac Sim preview for supre_robot_assets/scenes/dkk_2.usd."""

from __future__ import annotations

import argparse
import asyncio
import math
import shutil
from pathlib import Path

try:
    from isaacsim.simulation_app import SimulationApp
except ModuleNotFoundError:  # pragma: no cover - compatibility with older Isaac Sim.
    from omni.isaac.kit import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        default="/workspace/supre_robot_assets/scenes/dkk_2.usd",
        help="USD scene path inside the Isaac Sim environment.",
    )
    parser.add_argument(
        "--output-dir",
        default="/workspace/sim1/outputs/dkk_2_orbit_frames",
        help="Directory for rendered rgb_*.png frames.",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--warmup-steps", type=int, default=90)
    parser.add_argument("--orbit-degrees", type=float, default=360.0)
    parser.add_argument("--radius-scale", type=float, default=1.75)
    parser.add_argument("--min-radius", type=float, default=6.0)
    parser.add_argument("--height-scale", type=float, default=0.55)
    parser.add_argument("--capture-timeout-frames", type=int, default=240)
    parser.add_argument("--active-gpu", type=int, default=0)
    parser.add_argument("--multi-gpu", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--clean", action=argparse.BooleanOptionalAction, default=True)
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
print("[record_dkk2] SimulationApp created", flush=True)

import carb  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "omni.kit.capture.viewport",
    True,
)

from omni.kit.capture.viewport import CaptureExtension, CaptureOptions, CaptureRenderPreset  # noqa: E402
from omni.kit.viewport.utility import get_active_viewport  # noqa: E402
from pxr import Usd, UsdGeom, UsdLux  # noqa: E402

print("[record_dkk2] Isaac modules imported", flush=True)


def log(message: str) -> None:
    print(f"[record_dkk2] {message}", flush=True)


def configure_renderer() -> None:
    settings = carb.settings.get_settings()
    settings.set("/rtx/post/aa/op", 2)


def add_lights(stage: Usd.Stage) -> None:
    dome = UsdLux.DomeLight.Define(stage, "/World/DKK2OrbitDome")
    dome.CreateIntensityAttr(2500)
    key = UsdLux.DistantLight.Define(stage, "/World/DKK2OrbitKey")
    key.CreateIntensityAttr(1800)
    key.CreateAngleAttr(0.35)


def open_scene_stage(scene_path: Path) -> Usd.Stage:
    context = omni.usd.get_context()
    if not context.open_stage(str(scene_path)):
        raise RuntimeError(f"Failed to open USD stage: {scene_path}")

    for _ in range(60):
        simulation_app.update()
        stage = context.get_stage()
        if stage is not None:
            return stage

    raise RuntimeError(f"Timed out opening USD stage: {scene_path}")


def compute_scene_frame(stage: Usd.Stage) -> tuple[tuple[float, float, float], float, float]:
    prim = stage.GetPrimAtPath("/World/RJ2506")
    if not prim or not prim.IsValid():
        prim = stage.GetPrimAtPath("/World")
    if not prim or not prim.IsValid():
        prim = stage.GetPseudoRoot()

    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    min_v = box.GetMin()
    max_v = box.GetMax()

    values = [min_v[0], min_v[1], min_v[2], max_v[0], max_v[1], max_v[2]]
    if not all(math.isfinite(float(value)) for value in values) or box.IsEmpty():
        return (0.0, 0.0, 1.0), args.min_radius, 3.0

    center = box.GetMidpoint()
    size = max_v - min_v
    horizontal = max(float(size[0]), float(size[1]), 1.0)
    vertical = max(float(size[2]), 1.0)
    radius = min(max(args.min_radius, horizontal * args.radius_scale), 10.0)
    height = max(float(center[2]) + vertical * args.height_scale, 2.2)
    target_z = float(center[2]) + vertical * 0.32
    return (float(center[0]), float(center[1]), target_z), radius, height


def camera_position_for_frame(
    *,
    frame_index: int,
    frame_count: int,
    target: tuple[float, float, float],
    radius: float,
    height: float,
) -> tuple[float, float, float]:
    angle = math.radians(args.orbit_degrees) * frame_index / max(frame_count, 1)
    return (
        target[0] + radius * math.cos(angle),
        target[1] + radius * math.sin(angle),
        height,
    )


def capture_viewport_frame(output_path: Path, eye: tuple[float, float, float], target: tuple[float, float, float]) -> None:
    viewport = get_active_viewport()
    if viewport is None:
        raise RuntimeError("Active viewport is not available")
    viewport.updates_enabled = True

    set_camera_view(
        eye=list(eye),
        target=list(target),
        camera_prim_path=viewport.get_active_camera(),
        viewport_api=viewport,
    )

    loop = asyncio.get_event_loop()
    for _ in range(8):
        loop.run_until_complete(simulation_app.app.next_update_async())

    capture_dir = output_path.parent / f"_{output_path.stem}_capture"
    if capture_dir.exists():
        shutil.rmtree(capture_dir)
    capture_dir.mkdir(parents=True)

    options = CaptureOptions()
    options.file_type = ".png"
    options.output_folder = str(capture_dir)
    options.file_name = "Capture"
    options.overwrite_existing_frames = True
    options.hdr_output = False
    options.camera = viewport.camera_path.pathString
    options.res_width = args.width
    options.res_height = args.height
    options.render_preset = CaptureRenderPreset.RAY_TRACE
    options.real_time_settle_latency_frames = 0
    options.path_trace_spp = 1

    capture = CaptureExtension().get_instance()
    capture.options = options
    capture.start()
    for _ in range(args.capture_timeout_frames):
        loop.run_until_complete(simulation_app.app.next_update_async())
        if capture.done:
            break
    capture_done = capture.done
    outputs = capture.get_outputs() if capture_done else []
    capture.options = None
    if not capture_done:
        capture.cancel()
        raise RuntimeError(f"Viewport capture timed out for {output_path}")

    candidates = [Path(path) for path in outputs if str(path).endswith(".png")]
    if not candidates:
        candidates = sorted(capture_dir.rglob("*.png"))
    if not candidates:
        raise RuntimeError(f"Viewport capture did not write a PNG in {capture_dir}")
    shutil.copy2(candidates[0], output_path)
    shutil.rmtree(capture_dir, ignore_errors=True)


def main() -> None:
    log("configuring renderer")
    configure_renderer()
    log("renderer configured")

    log("resolving paths")
    scene_path = Path(args.scene)
    output_dir = Path(args.output_dir)
    log(f"resolved scene={scene_path} exists={scene_path.exists()}")
    if not scene_path.exists():
        raise FileNotFoundError(f"Scene not found: {scene_path}")
    log(f"resolved output_dir={output_dir}")
    if args.clean and output_dir.exists():
        log("cleaning output dir")
        shutil.rmtree(output_dir)
    log("creating output dir")
    output_dir.mkdir(parents=True, exist_ok=True)

    log(f"scene={scene_path}")
    log(f"output_dir={output_dir}")
    stage = open_scene_stage(scene_path)
    add_lights(stage)
    for _ in range(args.warmup_steps):
        simulation_app.update()

    target, radius, height = compute_scene_frame(stage)
    frame_count = max(1, int(round(args.fps * args.duration)))
    log(f"target={target} radius={radius:.3f} height={height:.3f} frames={frame_count}")

    for frame_index in range(frame_count):
        simulation_app.update()
        position = camera_position_for_frame(
            frame_index=frame_index,
            frame_count=frame_count,
            target=target,
            radius=radius,
            height=height,
        )
        capture_viewport_frame(output_dir / f"rgb_{frame_index:04d}.png", position, target)
        if frame_index == 0 or (frame_index + 1) % max(1, frame_count // 8) == 0:
            log(f"captured {frame_index + 1}/{frame_count}")

    frames = sorted(output_dir.rglob("rgb_*.png"))
    if not frames:
        raise RuntimeError(f"Viewport capture did not write any rgb_*.png frames in {output_dir}")

    log(f"done frames={len(frames)}")


print("[record_dkk2] entering main", flush=True)
try:
    main()
except BaseException as exc:
    print(f"[record_dkk2] ERROR {type(exc).__name__}: {exc}", flush=True)
    raise
finally:
    print("[record_dkk2] closing SimulationApp", flush=True)
    simulation_app.close()
