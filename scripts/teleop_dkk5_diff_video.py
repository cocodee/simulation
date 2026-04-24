#!/usr/bin/env python3
"""Keyboard teleoperation for the DKK5 differential-drive dual-arm robot."""

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


ROOT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_URDF = "/root/data1/kdi/workspace/supre_robot_assets/assets/robots/RJ2506/urdf/relpath_RJ2506.urdf"


def parse_tuple(value: str) -> tuple[float, float, float]:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("Expected three comma-separated values, e.g. -3,3,2.5")
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5.usd")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--video", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--action-log", required=True)
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
    parser.add_argument("--wheel-radius", type=float, default=0.075)
    parser.add_argument("--wheel-base", type=float, default=0.6284)
    parser.add_argument("--max-wheel-speed", type=float, default=4.5)
    parser.add_argument("--max-linear-speed", type=float, default=0.25)
    parser.add_argument("--max-angular-speed", type=float, default=0.8)
    parser.add_argument("--fast-multiplier", type=float, default=2.0)
    parser.add_argument("--ee-linear-step", type=float, default=0.015)
    parser.add_argument("--ee-angular-step", type=float, default=0.06)
    parser.add_argument("--gripper-step", type=float, default=0.002)
    parser.add_argument("--left-ee-frame", default="left_hand_tcp")
    parser.add_argument("--right-ee-frame", default="right_hand_base")
    parser.add_argument("--left-lula-config", default=str(ROOT_DIR / "configs/rj2506_left_arm_lula.yaml"))
    parser.add_argument("--right-lula-config", default=str(ROOT_DIR / "configs/rj2506_right_arm_lula.yaml"))
    parser.add_argument("--robot-urdf", default=DEFAULT_ROBOT_URDF)
    parser.add_argument("--position-tolerance", type=float, default=0.02)
    parser.add_argument("--orientation-tolerance", type=float, default=0.25)
    parser.add_argument("--active-gpu", type=int, default=0)
    parser.add_argument("--multi-gpu", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--headless", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--no-video", action="store_true")
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
import carb.input  # noqa: E402
import omni.appwindow  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.robots import Robot  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController  # noqa: E402
from pxr import Sdf, UsdGeom  # noqa: E402

try:
    from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver  # noqa: E402
except Exception:  # pragma: no cover
    ArticulationKinematicsSolver = None
    LulaKinematicsSolver = None


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

KEY_BINDINGS = {
    "W/S": "base forward/backward",
    "A/D": "base turn left/right",
    "LEFT_SHIFT/RIGHT_SHIFT": "fast multiplier",
    "1/2/3": "select left/right/both arms",
    "I/K": "selected end effector +X/-X",
    "J/L": "selected end effector +Y/-Y",
    "U/O": "selected end effector +Z/-Z",
    "Z/X": "selected end effector roll +/-",
    "C/V": "selected end effector pitch +/-",
    "B/N": "selected end effector yaw +/-",
    "T/G": "selected gripper open/close",
    "R": "reset selected end effector targets to current FK",
    "SPACE": "zero base command",
    "Q/ESCAPE": "finish recording and quit",
}


def log(message: str) -> None:
    print(f"[dkk5_teleop] {message}", flush=True)


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


def encode_video(frames_dir: Path, video_path: Path, fps: int, crf: int) -> int:
    frames = sorted(frames_dir.glob("rgb_*.png"))
    if not frames:
        raise FileNotFoundError(f"No rgb_*.png frames found in {frames_dir}")
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
    return [float(value) if np.isfinite(value) else None for value in array]


def rotation_matrix_to_quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(m))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return np.array([0.25 * scale, (m[2, 1] - m[1, 2]) / scale, (m[0, 2] - m[2, 0]) / scale, (m[1, 0] - m[0, 1]) / scale])
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        scale = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return np.array([(m[2, 1] - m[1, 2]) / scale, 0.25 * scale, (m[0, 1] + m[1, 0]) / scale, (m[0, 2] + m[2, 0]) / scale])
    if m[1, 1] > m[2, 2]:
        scale = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return np.array([(m[0, 2] - m[2, 0]) / scale, (m[0, 1] + m[1, 0]) / scale, 0.25 * scale, (m[1, 2] + m[2, 1]) / scale])
    scale = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return np.array([(m[1, 0] - m[0, 1]) / scale, (m[0, 2] + m[2, 0]) / scale, (m[1, 2] + m[2, 1]) / scale, 0.25 * scale])


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm < 1e-9 or abs(angle) < 1e-9:
        return np.eye(3)
    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return np.array(
        [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ],
        dtype=np.float64,
    )


def build_motion_targets(robot: Robot) -> dict[str, object]:
    dof_names = list(robot.dof_names)
    joint_index = {name: dof_names.index(name) for name in dof_names}
    for name in WHEEL_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + FINGER_JOINTS:
        if name not in joint_index:
            raise RuntimeError(f"Joint {name} was not found on articulation {args.robot_prim}. Available: {dof_names}")
    return {
        "dof_names": dof_names,
        "dof_count": len(dof_names),
        "joint_index": joint_index,
        "lower_limits": np.array(robot.dof_properties["lower"], dtype=np.float64),
        "upper_limits": np.array(robot.dof_properties["upper"], dtype=np.float64),
    }


def configure_control_modes(robot: Robot, motion: dict[str, object]) -> None:
    controller = robot.get_articulation_controller()
    joint_index = motion["joint_index"]
    for name in WHEEL_JOINTS:
        controller.switch_dof_control_mode(joint_index[name], "velocity")
    for name in LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + FINGER_JOINTS:
        controller.switch_dof_control_mode(joint_index[name], "position")


class KeyboardTeleop:
    def __init__(self) -> None:
        self.keys: set[str] = set()
        self.quit_requested = False
        self.arm_mode = "both"
        self._input = None
        self._keyboard = None
        self._subscription = None

    def setup(self) -> None:
        appwindow = omni.appwindow.get_default_app_window()
        if appwindow is None:
            raise RuntimeError("No Isaac Sim app window is available. Run with --no-headless for keyboard teleop.")
        self._keyboard = appwindow.get_keyboard()
        self._input = carb.input.acquire_input_interface()
        self._subscription = self._input.subscribe_to_keyboard_events(self._keyboard, self._on_keyboard_event)
        log("keyboard_ready")

    def cleanup(self) -> None:
        if self._input is not None and self._keyboard is not None and self._subscription is not None:
            self._input.unsubscribe_to_keyboard_events(self._keyboard, self._subscription)
        self._subscription = None

    def selected_arms(self) -> list[str]:
        if self.arm_mode == "left":
            return ["left"]
        if self.arm_mode == "right":
            return ["right"]
        return ["left", "right"]

    def multiplier(self) -> float:
        if "LEFT_SHIFT" in self.keys or "RIGHT_SHIFT" in self.keys:
            return args.fast_multiplier
        return 1.0

    def base_command(self) -> np.ndarray:
        linear = (("W" in self.keys) - ("S" in self.keys)) * args.max_linear_speed
        angular = (("A" in self.keys) - ("D" in self.keys)) * args.max_angular_speed
        if "SPACE" in self.keys:
            linear = 0.0
            angular = 0.0
        return np.array([linear, angular], dtype=np.float64) * self.multiplier()

    def ee_delta(self) -> tuple[np.ndarray, np.ndarray]:
        step = args.ee_linear_step * self.multiplier()
        angular_step = args.ee_angular_step * self.multiplier()
        translation = np.array(
            [
                (("I" in self.keys) - ("K" in self.keys)) * step,
                (("J" in self.keys) - ("L" in self.keys)) * step,
                (("U" in self.keys) - ("O" in self.keys)) * step,
            ],
            dtype=np.float64,
        )
        roll = (("Z" in self.keys) - ("X" in self.keys)) * angular_step
        pitch = (("C" in self.keys) - ("V" in self.keys)) * angular_step
        yaw = (("B" in self.keys) - ("N" in self.keys)) * angular_step
        rotation = axis_angle_to_matrix(np.array([1.0, 0.0, 0.0]), roll)
        rotation = axis_angle_to_matrix(np.array([0.0, 1.0, 0.0]), pitch) @ rotation
        rotation = axis_angle_to_matrix(np.array([0.0, 0.0, 1.0]), yaw) @ rotation
        return translation, rotation

    def gripper_delta(self) -> float:
        return (("T" in self.keys) - ("G" in self.keys)) * args.gripper_step * self.multiplier()

    def _on_keyboard_event(self, event, *args_, **kwargs_) -> bool:
        name = event.input.name
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self.keys.add(name)
            if name in {"Q", "ESCAPE"}:
                self.quit_requested = True
            elif name == "KEY_1":
                self.arm_mode = "left"
                log("arm_mode=left")
            elif name == "KEY_2":
                self.arm_mode = "right"
                log("arm_mode=right")
            elif name == "KEY_3":
                self.arm_mode = "both"
                log("arm_mode=both")
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self.keys.discard(name)
        return True


def init_ik_solver(robot: Robot, config_path: Path, urdf_path: Path, ee_frame: str):
    if LulaKinematicsSolver is None or ArticulationKinematicsSolver is None:
        raise RuntimeError("Isaac motion_generation Lula IK modules are not importable.")
    kinematics = LulaKinematicsSolver(robot_description_path=str(config_path), urdf_path=str(urdf_path))
    solver = ArticulationKinematicsSolver(robot, kinematics, ee_frame)
    return solver


def init_ik(robot: Robot) -> tuple[dict[str, object], dict[str, str]]:
    solvers: dict[str, object] = {}
    errors: dict[str, str] = {}
    specs = {
        "left": (Path(args.left_lula_config).resolve(), Path(args.robot_urdf).resolve(), args.left_ee_frame),
        "right": (Path(args.right_lula_config).resolve(), Path(args.robot_urdf).resolve(), args.right_ee_frame),
    }
    for side, (config_path, urdf_path, ee_frame) in specs.items():
        try:
            solvers[side] = init_ik_solver(robot, config_path, urdf_path, ee_frame)
            log(f"{side}_ik_ready frame={ee_frame}")
        except Exception as exc:
            errors[side] = str(exc)
            log(f"{side}_ik_unavailable error={exc}")
    return solvers, errors


def current_ee_targets(solvers: dict[str, object], errors: dict[str, str]) -> dict[str, dict[str, np.ndarray]]:
    targets: dict[str, dict[str, np.ndarray]] = {}
    for side, solver in list(solvers.items()):
        try:
            position, rotation = solver.compute_end_effector_pose()
            targets[side] = {
                "position": np.asarray(position, dtype=np.float64),
                "rotation": np.asarray(rotation, dtype=np.float64),
            }
        except Exception as exc:
            errors[side] = f"initial_fk_failed: {exc}"
            solvers.pop(side, None)
            log(f"{side}_ik_disabled error={exc}")
    return targets


def reset_selected_targets(teleop: KeyboardTeleop, solvers: dict[str, object], targets: dict[str, dict[str, np.ndarray]]) -> None:
    for side in teleop.selected_arms():
        if side in solvers:
            position, rotation = solvers[side].compute_end_effector_pose()
            targets[side] = {"position": np.asarray(position, dtype=np.float64), "rotation": np.asarray(rotation, dtype=np.float64)}
    log(f"targets_reset mode={teleop.arm_mode}")


def apply_teleop(
    robot: Robot,
    teleop: KeyboardTeleop,
    motion: dict[str, object],
    diff_controller: DifferentialController,
    ik_solvers: dict[str, object],
    ee_targets: dict[str, dict[str, np.ndarray]],
    gripper_targets: dict[str, float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict[str, bool]]:
    dof_count = motion["dof_count"]
    joint_index = motion["joint_index"]
    lower_limits = motion["lower_limits"]
    upper_limits = motion["upper_limits"]
    positions = np.full(dof_count, np.nan, dtype=np.float64)
    velocities = np.full(dof_count, np.nan, dtype=np.float64)
    drive_command = teleop.base_command()

    wheel_action = diff_controller.forward(drive_command)
    velocities[joint_index["left_wheel_joint"]] = WHEEL_VELOCITY_SIGNS["left_wheel_joint"] * float(wheel_action.joint_velocities[0])
    velocities[joint_index["right_wheel_joint"]] = WHEEL_VELOCITY_SIGNS["right_wheel_joint"] * float(wheel_action.joint_velocities[1])

    translation_delta, rotation_delta = teleop.ee_delta()
    ik_success: dict[str, bool] = {}
    for side in teleop.selected_arms():
        if side not in ik_solvers or side not in ee_targets:
            ik_success[side] = False
            continue
        candidate_position = np.array(ee_targets[side]["position"], copy=True)
        candidate_rotation = np.array(ee_targets[side]["rotation"], copy=True)
        if np.linalg.norm(translation_delta) > 0.0 or not np.allclose(rotation_delta, np.eye(3)):
            candidate_position = candidate_position + translation_delta
            candidate_rotation = rotation_delta @ candidate_rotation

        target_quat = rotation_matrix_to_quat_wxyz(candidate_rotation)
        action, success = ik_solvers[side].compute_inverse_kinematics(
            target_position=candidate_position,
            target_orientation=target_quat,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=args.orientation_tolerance,
        )
        ik_success[side] = bool(success)
        if success and action.joint_positions is not None and action.joint_indices is not None:
            ee_targets[side]["position"] = candidate_position
            ee_targets[side]["rotation"] = candidate_rotation
            for index, value in zip(action.joint_indices, action.joint_positions):
                positions[int(index)] = float(value)

    grip_delta = teleop.gripper_delta()
    if grip_delta != 0.0:
        for side in teleop.selected_arms():
            gripper_targets[side] = float(gripper_targets[side] + grip_delta)

    finger_names = {
        "left": ["left_hand_finger1_joint", "left_hand_finger2_joint"],
        "right": ["right_hand_finger1_joint", "right_hand_finger2_joint"],
    }
    for side, names in finger_names.items():
        for name in names:
            index = joint_index[name]
            gripper_targets[side] = float(np.clip(gripper_targets[side], lower_limits[index], upper_limits[index]))
            positions[index] = gripper_targets[side]

    positions = clamp(positions, lower_limits, upper_limits)
    robot.get_articulation_controller().apply_action(ArticulationAction(joint_positions=positions, joint_velocities=velocities))
    return positions, velocities, drive_command, ik_success


def make_log_record(
    frame_index: int,
    start_time: float,
    teleop: KeyboardTeleop,
    drive_command: np.ndarray,
    positions: np.ndarray,
    velocities: np.ndarray,
    ik_success: dict[str, bool],
    ee_targets: dict[str, dict[str, np.ndarray]],
    gripper_targets: dict[str, float],
) -> dict[str, object]:
    return {
        "frame": frame_index,
        "elapsed_seconds": time.time() - start_time,
        "arm_mode": teleop.arm_mode,
        "keys": sorted(teleop.keys),
        "drive_command": safe_float_list(drive_command),
        "ik_success": ik_success,
        "ee_targets": {
            side: {
                "position": safe_float_list(target["position"]),
                "orientation_wxyz": safe_float_list(rotation_matrix_to_quat_wxyz(target["rotation"])),
            }
            for side, target in ee_targets.items()
        },
        "gripper_targets": gripper_targets.copy(),
        "commanded_joint_positions": safe_float_list(positions),
        "commanded_joint_velocities": safe_float_list(velocities),
    }


def main() -> None:
    scene_path = Path(args.scene).resolve()
    output_dir = Path(args.output_dir).resolve()
    video_path = Path(args.video).resolve()
    manifest_path = Path(args.manifest).resolve()
    action_log_path = Path(args.action_log).resolve()
    if not scene_path.exists():
        raise FileNotFoundError(f"Scene not found: {scene_path}")

    output_dir.mkdir(parents=True, exist_ok=True)
    clean_output_dir(output_dir)
    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    action_log_path.parent.mkdir(parents=True, exist_ok=True)
    if action_log_path.exists():
        action_log_path.unlink()

    configure_render_settings()

    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    add_lights(stage)
    add_reference(stage, scene_path, args.reference_prim)
    for _ in range(30):
        simulation_app.update()

    prim_count, mesh_count = count_scene(stage)
    log(f"prims={prim_count} meshes={mesh_count}")

    camera = rep.create.camera(position=args.camera_position, look_at=args.look_at)
    render_product = rep.create.render_product(camera, (args.width, args.height))
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=str(output_dir), rgb=True)

    world.reset()
    robot = Robot(prim_path=args.robot_prim, name="dkk5_teleop_robot")
    robot.initialize()
    robot.post_reset()
    motion = build_motion_targets(robot)
    configure_control_modes(robot, motion)
    diff_controller = DifferentialController(
        name="dkk5_teleop_diff_controller",
        wheel_radius=args.wheel_radius,
        wheel_base=args.wheel_base,
        max_wheel_speed=args.max_wheel_speed,
    )
    ik_solvers, ik_errors = init_ik(robot)
    ee_targets = current_ee_targets(ik_solvers, ik_errors)

    joint_index = motion["joint_index"]
    current_positions = np.array(robot.get_joint_positions(), dtype=np.float64)
    gripper_targets = {
        "left": float(current_positions[joint_index["left_hand_finger1_joint"]]),
        "right": float(current_positions[joint_index["right_hand_finger1_joint"]]),
    }
    initial_joint_positions = np.array(robot.get_joint_positions(), dtype=np.float64)
    initial_world_pose = robot.get_world_pose()

    teleop = KeyboardTeleop()
    teleop.setup()
    log(f"key_bindings={json.dumps(KEY_BINDINGS, ensure_ascii=False)}")

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(30):
        simulation_app.update()
    writer.attach([render_product])

    start = time.time()
    frame_index = 0
    ik_success_count = 0
    ik_failure_count = 0
    last_commanded_positions = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    last_commanded_velocities = np.full(motion["dof_count"], np.nan, dtype=np.float64)
    last_drive_command = np.zeros(2, dtype=np.float64)
    try:
        with action_log_path.open("w", encoding="utf-8") as action_log:
            while simulation_app.is_running() and not teleop.quit_requested:
                if "R" in teleop.keys:
                    reset_selected_targets(teleop, ik_solvers, ee_targets)
                    teleop.keys.discard("R")
                last_commanded_positions, last_commanded_velocities, last_drive_command, ik_success = apply_teleop(
                    robot, teleop, motion, diff_controller, ik_solvers, ee_targets, gripper_targets
                )
                ik_success_count += sum(1 for value in ik_success.values() if value)
                ik_failure_count += sum(1 for value in ik_success.values() if not value)
                simulation_app.update()
                rep.orchestrator.step()
                record = make_log_record(
                    frame_index,
                    start,
                    teleop,
                    last_drive_command,
                    last_commanded_positions,
                    last_commanded_velocities,
                    ik_success,
                    ee_targets,
                    gripper_targets,
                )
                action_log.write(json.dumps(record, separators=(",", ":")) + "\n")
                if frame_index == 0 or (frame_index + 1) % max(1, args.fps * 5) == 0:
                    log(f"captured {frame_index + 1} arm_mode={teleop.arm_mode}")
                frame_index += 1
    finally:
        teleop.cleanup()

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
        encoded_frame_count = encode_video(output_dir, video_path, args.fps, args.crf)
        encoded_video = str(video_path)
        log(f"video={video_path}")

    manifest = {
        "teleop": True,
        "scene": str(scene_path),
        "output_dir": str(output_dir),
        "video": encoded_video,
        "manifest": str(manifest_path),
        "action_log": str(action_log_path),
        "reference_prim": args.reference_prim,
        "robot_prim": args.robot_prim,
        "prim_count": prim_count,
        "mesh_count": mesh_count,
        "frame_count": frame_count,
        "encoded_frame_count": encoded_frame_count,
        "recorded_action_frames": frame_index,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "elapsed_seconds": elapsed,
        "camera_position": args.camera_position,
        "look_at": args.look_at,
        "headless": args.headless,
        "active_gpu": args.active_gpu,
        "multi_gpu": args.multi_gpu,
        "key_bindings": KEY_BINDINGS,
        "max_linear_speed": args.max_linear_speed,
        "max_angular_speed": args.max_angular_speed,
        "ee_linear_step": args.ee_linear_step,
        "ee_angular_step": args.ee_angular_step,
        "left_ee_frame": args.left_ee_frame,
        "right_ee_frame": args.right_ee_frame,
        "ik_available": sorted(ik_solvers.keys()),
        "ik_errors": ik_errors,
        "ik_success_count": ik_success_count,
        "ik_failure_count": ik_failure_count,
        "dof_names": motion["dof_names"],
        "initial_base_position": np.asarray(initial_world_pose[0]).tolist(),
        "initial_base_orientation_wxyz": np.asarray(initial_world_pose[1]).tolist(),
        "final_base_position": np.asarray(final_world_pose[0]).tolist(),
        "final_base_orientation_wxyz": np.asarray(final_world_pose[1]).tolist(),
        "initial_joint_positions": safe_float_list(initial_joint_positions),
        "final_joint_positions": safe_float_list(final_joint_positions),
        "last_commanded_joint_positions": safe_float_list(last_commanded_positions),
        "last_commanded_joint_velocities": safe_float_list(last_commanded_velocities),
        "last_drive_command": safe_float_list(last_drive_command),
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    log(f"manifest={manifest_path}")


try:
    main()
finally:
    simulation_app.close()
