#!/usr/bin/env python3
"""Simulate dkk_2.usd, drive RJ2506 wheels and arms, and encode a motion video."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path

import numpy as np

os.environ.setdefault("PYTHONUNBUFFERED", "1")

try:
    from isaacsim.simulation_app import SimulationApp
except ModuleNotFoundError:  # pragma: no cover
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
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_2_motion_frames",
        help="Directory where Replicator BasicWriter will write rgb_*.png frames.",
    )
    parser.add_argument(
        "--video",
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_2_motion_video.mp4",
        help="MP4 output path.",
    )
    parser.add_argument(
        "--manifest",
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_2_motion_manifest.json",
        help="JSON manifest path.",
    )
    parser.add_argument("--frames", type=int, default=144)
    parser.add_argument("--warmup-steps", type=int, default=30)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--crf", type=int, default=18)
    parser.add_argument("--camera-position", type=parse_tuple, default=(-4.0, 4.0, 2.8))
    parser.add_argument("--look-at", type=parse_tuple, default=(-1.6, 0.3, 1.1))
    parser.add_argument("--reference-prim", default="/World/DKK2")
    parser.add_argument("--robot-prim", default="/World/DKK2/RJ2506/base_link")
    parser.add_argument("--dome-intensity", type=float, default=1800.0)
    parser.add_argument("--sun-intensity", type=float, default=900.0)
    parser.add_argument("--wheel-speed", type=float, default=7.5, help="Drive wheel angular speed in rad/s.")
    parser.add_argument("--turn-speed", type=float, default=3.6, help="Wheel angular speed used for turning phases.")
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
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402


LEFT_ARM_JOINTS = [f"left_arm_joint{i}" for i in range(6)]
RIGHT_ARM_JOINTS = [f"right_arm_joint{i}" for i in range(6)]
FINGER_JOINTS = [
    "left_hand_finger1_joint",
    "left_hand_finger2_joint",
    "right_hand_finger1_joint",
    "right_hand_finger2_joint",
]
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]


def log(message: str) -> None:
    print(f"[dkk2_motion] {message}", flush=True)


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
    dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
    dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(args.dome_intensity)

    sun_light = stage.DefinePrim("/World/DirectionalLight", "DistantLight")
    sun_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(args.sun_intensity)
    sun_light.CreateAttribute("inputs:angle", Sdf.ValueTypeNames.Float).Set(0.5)


def count_scene(stage) -> tuple[int, int]:
    prim_count = 0
    mesh_count = 0
    for prim in stage.Traverse():
        prim_count += 1
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
    return prim_count, mesh_count


def clean_output_dir(output_dir: Path) -> None:
    for pattern in ("rgb_*.png", "ffmpeg_list.txt"):
        for path in output_dir.glob(pattern):
            if path.is_file():
                path.unlink()


def encode_video(frames_dir: Path, video_path: Path, fps: int, crf: int, max_frames: int | None = None) -> int:
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
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-r",
            str(fps),
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
            str(crf),
            str(video_path),
        ],
        check=True,
    )
    return len(frames)


def clamp(values: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    return np.minimum(np.maximum(values, lower), upper)


def safe_float_list(values) -> list[float | None]:
    array = np.asarray(values, dtype=np.float64).reshape(-1)
    result: list[float | None] = []
    for value in array:
        result.append(float(value) if np.isfinite(value) else None)
    return result


def finite_or_none(values) -> list[float | None]:
    return safe_float_list(values)


def build_motion_targets(robot: Robot) -> dict[str, object]:
    dof_names = list(robot.dof_names)
    dof_count = len(dof_names)
    joint_index = {name: dof_names.index(name) for name in dof_names}
    initial_positions = np.array(robot.get_joint_positions(), dtype=np.float64)
    lower_limits = np.array(robot.dof_properties["lower"], dtype=np.float64)
    upper_limits = np.array(robot.dof_properties["upper"], dtype=np.float64)

    for name in WHEEL_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + FINGER_JOINTS:
        if name not in joint_index:
            raise RuntimeError(f"Joint {name} was not found on articulation {args.robot_prim}. Available: {dof_names}")

    return {
        "dof_names": dof_names,
        "dof_count": dof_count,
        "joint_index": joint_index,
        "initial_positions": initial_positions,
        "lower_limits": lower_limits,
        "upper_limits": upper_limits,
    }


def configure_control_modes(robot: Robot, motion: dict[str, object]) -> None:
    controller = robot.get_articulation_controller()
    joint_index = motion["joint_index"]

    for name in WHEEL_JOINTS:
        controller.switch_dof_control_mode(joint_index[name], "velocity")

    for name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + FINGER_JOINTS:
        controller.switch_dof_control_mode(joint_index[name], "position")


def wheel_command(phase: float) -> tuple[float, float]:
    if phase < 0.42:
        return args.wheel_speed, args.wheel_speed
    if phase < 0.50:
        return -args.turn_speed, args.turn_speed
    if phase < 0.82:
        return -args.wheel_speed, -args.wheel_speed
    if phase < 0.90:
        return args.turn_speed, -args.turn_speed
    return args.wheel_speed, args.wheel_speed


def arm_targets(phase: float, motion: dict[str, object]) -> np.ndarray:
    joint_index = motion["joint_index"]
    initial_positions = motion["initial_positions"]
    lower_limits = motion["lower_limits"]
    upper_limits = motion["upper_limits"]
    targets = np.array(initial_positions, copy=True)

    theta = 2.0 * math.pi * phase
    secondary = 4.0 * math.pi * phase

    left_offsets = np.array(
        [
            0.30 * math.sin(theta),
            -0.45 + 0.20 * math.sin(theta),
            0.35 * math.sin(theta + 0.8),
            -0.25 + 0.18 * math.sin(secondary),
            0.22 * math.cos(theta),
            0.18 * math.sin(theta + 1.2),
        ],
        dtype=np.float64,
    )
    right_offsets = np.array(
        [
            -0.30 * math.sin(theta),
            -0.45 + 0.18 * math.sin(theta + 0.5),
            -0.35 * math.sin(theta + 0.8),
            0.25 - 0.18 * math.sin(secondary),
            -0.22 * math.cos(theta),
            -0.18 * math.sin(theta + 1.2),
        ],
        dtype=np.float64,
    )

    for name, offset in zip(LEFT_ARM_JOINTS, left_offsets):
        index = joint_index[name]
        targets[index] = initial_positions[index] + offset

    for name, offset in zip(RIGHT_ARM_JOINTS, right_offsets):
        index = joint_index[name]
        targets[index] = initial_positions[index] + offset

    open_amount = 0.018 + 0.008 * (0.5 + 0.5 * math.sin(theta))
    for name in FINGER_JOINTS:
        targets[joint_index[name]] = open_amount

    return clamp(targets, lower_limits, upper_limits)


def apply_motion(robot: Robot, frame_index: int, frame_count: int, motion: dict[str, object]) -> tuple[np.ndarray, np.ndarray]:
    phase = frame_index / max(frame_count - 1, 1)
    positions = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    velocities = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    joint_index = motion["joint_index"]

    left_speed, right_speed = wheel_command(phase)
    velocities[joint_index["left_wheel_joint"]] = left_speed
    velocities[joint_index["right_wheel_joint"]] = right_speed

    position_targets = arm_targets(phase, motion)
    for name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + FINGER_JOINTS:
        index = joint_index[name]
        positions[index] = position_targets[index]

    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=positions, joint_velocities=velocities)
    )
    return positions, velocities


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

    for _ in range(args.warmup_steps):
        simulation_app.update()

    prim_count, mesh_count = count_scene(stage)
    log(f"prims={prim_count} meshes={mesh_count}")

    camera = rep.create.camera(position=args.camera_position, look_at=args.look_at)
    render_product = rep.create.render_product(camera, (args.width, args.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(output_dir), rgb=True)

    world.reset()
    robot = Robot(prim_path=args.robot_prim, name="dkk2_robot")
    robot.initialize()
    robot.post_reset()
    motion = build_motion_targets(robot)
    configure_control_modes(robot, motion)
    initial_joint_positions = np.array(robot.get_joint_positions(), dtype=np.float64)
    initial_world_pose = robot.get_world_pose()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(args.warmup_steps):
        simulation_app.update()

    writer.attach([render_product])
    start = time.time()
    last_commanded_positions = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    last_commanded_velocities = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    for index in range(args.frames):
        last_commanded_positions, last_commanded_velocities = apply_motion(robot, index, args.frames, motion)
        simulation_app.update()
        rep.orchestrator.step()
        if index == 0 or (index + 1) % max(1, args.frames // 6) == 0:
            log(f"captured {index + 1}/{args.frames}")

    rep.orchestrator.wait_until_complete()
    final_world_pose = robot.get_world_pose()
    timeline.stop()
    simulation_app.update()
    elapsed = time.time() - start

    frame_count = len(list(output_dir.glob("rgb_*.png")))
    encoded_frame_count = 0
    encoded_video = None
    if not args.no_video:
        encoded_frame_count = encode_video(output_dir, video_path, args.fps, args.crf, max_frames=args.frames)
        encoded_video = str(video_path)
        log(f"video={video_path}")

    manifest = {
        "scene": str(scene_path),
        "output_dir": str(output_dir),
        "video": encoded_video,
        "manifest": str(manifest_path),
        "reference_prim": args.reference_prim,
        "robot_prim": args.robot_prim,
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
        "wheel_speed": args.wheel_speed,
        "turn_speed": args.turn_speed,
        "headless": args.headless,
        "active_gpu": args.active_gpu,
        "multi_gpu": args.multi_gpu,
        "wheel_joints": WHEEL_JOINTS,
        "left_arm_joints": LEFT_ARM_JOINTS,
        "right_arm_joints": RIGHT_ARM_JOINTS,
        "finger_joints": FINGER_JOINTS,
        "dof_names": motion["dof_names"],
        "initial_base_position": np.asarray(initial_world_pose[0]).tolist(),
        "initial_base_orientation_wxyz": np.asarray(initial_world_pose[1]).tolist(),
        "final_base_position": np.asarray(final_world_pose[0]).tolist(),
        "final_base_orientation_wxyz": np.asarray(final_world_pose[1]).tolist(),
        "initial_joint_positions": safe_float_list(initial_joint_positions),
        "last_commanded_joint_positions": finite_or_none(last_commanded_positions),
        "last_commanded_joint_velocities": finite_or_none(last_commanded_velocities),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    log(f"manifest={manifest_path}")


try:
    main()
finally:
    simulation_app.close()
