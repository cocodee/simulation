#!/usr/bin/env python3
"""Record a DKK5 video with differential-drive base motion and arm articulation carry pose control."""

from __future__ import annotations

import argparse
import json
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


def parse_joint_vector(value: str, expected_len: int, label: str) -> np.ndarray:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != expected_len:
        raise argparse.ArgumentTypeError(f"{label} expects {expected_len} comma-separated values.")
    return np.asarray([float(part) for part in parts], dtype=np.float64)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scene",
        default="/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5.usd",
        help="Absolute path to the DKK scene USD.",
    )
    parser.add_argument("--output-dir", required=True, help="Directory where rgb_*.png frames will be written.")
    parser.add_argument("--video", required=True, help="MP4 output path.")
    parser.add_argument("--manifest", required=True, help="JSON manifest path.")
    parser.add_argument("--frames", type=int, default=480)
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
    parser.add_argument("--wheel-speed", type=float, default=0.8)
    parser.add_argument("--angular-speed", type=float, default=0.0)
    parser.add_argument("--wheel-radius", type=float, default=0.075)
    parser.add_argument("--wheel-base", type=float, default=0.6284)
    parser.add_argument("--max-wheel-speed", type=float, default=2.5)
    parser.add_argument("--settle-seconds", type=float, default=5.0)
    parser.add_argument(
        "--arm-transition-seconds",
        type=float,
        default=4.0,
        help="Seconds used to interpolate from the initial arm pose to the carry pose.",
    )
    parser.add_argument(
        "--left-carry-pose",
        type=lambda value: parse_joint_vector(value, 6, "left-carry-pose"),
        default=np.asarray([0.55, -0.42, 0.30, -1.10, 0.08, 0.10], dtype=np.float64),
        help="Six joint target values for the left arm carry pose.",
    )
    parser.add_argument(
        "--right-carry-pose",
        type=lambda value: parse_joint_vector(value, 6, "right-carry-pose"),
        default=np.asarray([-0.55, -0.42, -0.30, 1.10, -0.08, -0.10], dtype=np.float64),
        help="Six joint target values for the right arm carry pose.",
    )
    parser.add_argument(
        "--finger-carry-position",
        type=float,
        default=0.004,
        help="Finger joint position held during the carry pose.",
    )
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
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController  # noqa: E402
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
WHEEL_VELOCITY_SIGNS = {"left_wheel_joint": -1.0, "right_wheel_joint": 1.0}


def log(message: str) -> None:
    print(f"[dkk5_diff_carry] {message}", flush=True)


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


def smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


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

    carry_targets = np.array(initial_positions, copy=True)
    for name, target in zip(LEFT_ARM_JOINTS, args.left_carry_pose):
        carry_targets[joint_index[name]] = target
    for name, target in zip(RIGHT_ARM_JOINTS, args.right_carry_pose):
        carry_targets[joint_index[name]] = target
    for name in FINGER_JOINTS:
        carry_targets[joint_index[name]] = args.finger_carry_position
    carry_targets = clamp(carry_targets, lower_limits, upper_limits)

    return {
        "dof_names": dof_names,
        "dof_count": dof_count,
        "joint_index": joint_index,
        "initial_positions": initial_positions,
        "lower_limits": lower_limits,
        "upper_limits": upper_limits,
        "carry_targets": carry_targets,
    }


def configure_control_modes(robot: Robot, motion: dict[str, object]) -> None:
    controller = robot.get_articulation_controller()
    joint_index = motion["joint_index"]

    for name in WHEEL_JOINTS:
        controller.switch_dof_control_mode(joint_index[name], "velocity")

    for name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + FINGER_JOINTS:
        controller.switch_dof_control_mode(joint_index[name], "position")


def build_drive_command() -> np.ndarray:
    linear_speed = args.wheel_speed * args.wheel_radius
    return np.array([linear_speed, args.angular_speed], dtype=np.float64)


def arm_targets(frame_index: int, frame_count: int, motion: dict[str, object]) -> tuple[np.ndarray, float]:
    initial_positions = motion["initial_positions"]
    carry_targets = motion["carry_targets"]
    settle_frames = min(frame_count, max(0, int(round(args.settle_seconds * args.fps))))
    transition_frames = max(1, int(round(args.arm_transition_seconds * args.fps)))

    if frame_index < settle_frames:
        blend = 0.0
    else:
        transition_index = min(frame_index - settle_frames, transition_frames)
        blend = smoothstep(transition_index / transition_frames)

    targets = (1.0 - blend) * initial_positions + blend * carry_targets
    return targets, blend


def apply_motion(
    robot: Robot,
    frame_index: int,
    frame_count: int,
    motion: dict[str, object],
    diff_controller: DifferentialController,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    settle_frames = min(frame_count, max(0, int(round(args.settle_seconds * args.fps))))
    positions = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    velocities = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    joint_index = motion["joint_index"]

    if frame_index < settle_frames:
        drive_command = np.array([0.0, 0.0], dtype=np.float64)
        velocities[joint_index["left_wheel_joint"]] = 0.0
        velocities[joint_index["right_wheel_joint"]] = 0.0
    else:
        drive_command = build_drive_command()
        wheel_action = diff_controller.forward(drive_command)
        velocities[joint_index["left_wheel_joint"]] = (
            WHEEL_VELOCITY_SIGNS["left_wheel_joint"] * float(wheel_action.joint_velocities[0])
        )
        velocities[joint_index["right_wheel_joint"]] = (
            WHEEL_VELOCITY_SIGNS["right_wheel_joint"] * float(wheel_action.joint_velocities[1])
        )

    position_targets, arm_blend = arm_targets(frame_index, frame_count, motion)
    for name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + FINGER_JOINTS:
        index = joint_index[name]
        positions[index] = position_targets[index]

    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=positions, joint_velocities=velocities)
    )
    return positions, velocities, drive_command, arm_blend


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
    robot = Robot(prim_path=args.robot_prim, name="dkk5_diff_carry_robot")
    robot.initialize()
    robot.post_reset()
    motion = build_motion_targets(robot)
    configure_control_modes(robot, motion)
    diff_controller = DifferentialController(
        name="dkk5_diff_controller",
        wheel_radius=args.wheel_radius,
        wheel_base=args.wheel_base,
        max_wheel_speed=args.max_wheel_speed,
    )
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
    last_drive_command = np.zeros(2, dtype=np.float64)
    last_arm_blend = 0.0
    for index in range(args.frames):
        (
            last_commanded_positions,
            last_commanded_velocities,
            last_drive_command,
            last_arm_blend,
        ) = apply_motion(robot, index, args.frames, motion, diff_controller)
        simulation_app.update()
        rep.orchestrator.step()
        if index == 0 or (index + 1) % max(1, args.frames // 6) == 0:
            log(f"captured {index + 1}/{args.frames} arm_blend={last_arm_blend:.2f}")

    rep.orchestrator.wait_until_complete()
    final_world_pose = robot.get_world_pose()
    final_joint_positions = np.array(robot.get_joint_positions(), dtype=np.float64)
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
        "settle_seconds": args.settle_seconds,
        "arm_transition_seconds": args.arm_transition_seconds,
        "elapsed_seconds": elapsed,
        "camera_position": args.camera_position,
        "look_at": args.look_at,
        "wheel_speed": args.wheel_speed,
        "angular_speed": args.angular_speed,
        "wheel_radius": args.wheel_radius,
        "wheel_base": args.wheel_base,
        "max_wheel_speed": args.max_wheel_speed,
        "finger_carry_position": args.finger_carry_position,
        "headless": args.headless,
        "active_gpu": args.active_gpu,
        "multi_gpu": args.multi_gpu,
        "wheel_joints": WHEEL_JOINTS,
        "wheel_velocity_signs": WHEEL_VELOCITY_SIGNS,
        "left_arm_joints": LEFT_ARM_JOINTS,
        "right_arm_joints": RIGHT_ARM_JOINTS,
        "finger_joints": FINGER_JOINTS,
        "left_carry_pose": safe_float_list(args.left_carry_pose),
        "right_carry_pose": safe_float_list(args.right_carry_pose),
        "dof_names": motion["dof_names"],
        "initial_base_position": np.asarray(initial_world_pose[0]).tolist(),
        "initial_base_orientation_wxyz": np.asarray(initial_world_pose[1]).tolist(),
        "final_base_position": np.asarray(final_world_pose[0]).tolist(),
        "final_base_orientation_wxyz": np.asarray(final_world_pose[1]).tolist(),
        "initial_joint_positions": safe_float_list(initial_joint_positions),
        "target_carry_joint_positions": safe_float_list(motion["carry_targets"]),
        "final_joint_positions": safe_float_list(final_joint_positions),
        "last_commanded_joint_positions": safe_float_list(last_commanded_positions),
        "last_commanded_joint_velocities": safe_float_list(last_commanded_velocities),
        "last_drive_command": safe_float_list(last_drive_command),
        "last_arm_blend": last_arm_blend,
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    log(f"manifest={manifest_path}")


try:
    main()
finally:
    simulation_app.close()
