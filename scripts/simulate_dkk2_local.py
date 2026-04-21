#!/usr/bin/env python3
"""Run dkk_2.usd in a local Isaac Sim installation without Docker."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

try:
    from isaacsim.simulation_app import SimulationApp
except ModuleNotFoundError:  # pragma: no cover - compatibility with older Isaac Sim.
    from omni.isaac.kit import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        default="/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_2.usd",
        help="Absolute path to dkk_2.usd.",
    )
    parser.add_argument(
        "--output",
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_2_local_isaac_manifest.json",
        help="JSON manifest path.",
    )
    parser.add_argument(
        "--export-stage",
        default="",
        help="Optional USD path to export the stage after simulation.",
    )
    parser.add_argument("--steps", type=int, default=120, help="Number of Isaac app updates to run with timeline playing.")
    parser.add_argument("--warmup-steps", type=int, default=30, help="Updates after stage open before timeline play.")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--active-gpu", type=int, default=0)
    parser.add_argument("--multi-gpu", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


args = parse_args()
simulation_app = SimulationApp(
    {
        "headless": args.headless,
        "renderer": "RaytracedLighting",
        "width": args.width,
        "height": args.height,
        "active_gpu": args.active_gpu,
        "multi_gpu": args.multi_gpu,
    }
)

import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402


def log(message: str) -> None:
    print(f"[local_dkk2] {message}", flush=True)


def wait_for_stage(context: object, max_updates: int = 240) -> object:
    for _ in range(max_updates):
        simulation_app.update()
        stage = context.get_stage()
        if stage is not None:
            return stage
    raise RuntimeError("Timed out waiting for USD stage")


def main() -> None:
    scene_path = Path(args.scene).resolve()
    output_path = Path(args.output).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    if not scene_path.exists():
        raise FileNotFoundError(f"Scene not found: {scene_path}")

    context = omni.usd.get_context()
    log(f"opening scene={scene_path}")
    if not context.open_stage(str(scene_path)):
        raise RuntimeError(f"Failed to open stage: {scene_path}")

    stage = wait_for_stage(context)
    for _ in range(args.warmup_steps):
        simulation_app.update()

    prim_count = 0
    mesh_count = 0
    for prim in stage.Traverse():
        prim_count += 1
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    start = time.time()
    for index in range(args.steps):
        simulation_app.update()
        if index == 0 or (index + 1) % max(1, args.steps // 5) == 0:
            log(f"simulated {index + 1}/{args.steps}")
    elapsed = time.time() - start
    timeline.stop()
    simulation_app.update()

    exported_stage = None
    if args.export_stage:
        export_path = Path(args.export_stage).resolve()
        export_path.parent.mkdir(parents=True, exist_ok=True)
        stage.Flatten().Export(str(export_path))
        exported_stage = str(export_path)
        log(f"exported_stage={export_path}")

    manifest = {
        "scene": str(scene_path),
        "output": str(output_path),
        "exported_stage": exported_stage,
        "default_prim": str(stage.GetDefaultPrim().GetPath()) if stage.GetDefaultPrim() else None,
        "prim_count": prim_count,
        "mesh_count": mesh_count,
        "warmup_steps": args.warmup_steps,
        "sim_steps": args.steps,
        "elapsed_seconds": elapsed,
        "headless": args.headless,
        "active_gpu": args.active_gpu,
        "multi_gpu": args.multi_gpu,
    }
    output_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    log(f"done manifest={output_path}")


try:
    main()
finally:
    simulation_app.close()
