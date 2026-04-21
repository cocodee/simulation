#!/usr/bin/env python3
"""Render a lightweight orbit preview directly from supre_robot_assets/scenes/dkk_2.usd."""

from __future__ import annotations

import argparse
import hashlib
import math
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np
from pxr import Usd, UsdGeom


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="../supre_robot_assets/scenes/dkk_2.usd")
    parser.add_argument("--output-dir", default="outputs/dkk_2_orbit_frames")
    parser.add_argument("--video", default="outputs/dkk_2_orbit.mp4")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--orbit-degrees", type=float, default=360.0)
    parser.add_argument("--min-radius", type=float, default=6.0)
    parser.add_argument("--max-triangles", type=int, default=80000)
    parser.add_argument("--crf", type=int, default=20)
    parser.add_argument("--clean", action=argparse.BooleanOptionalAction, default=True)
    return parser.parse_args()


def mesh_color(path: str) -> tuple[int, int, int]:
    digest = hashlib.sha1(path.encode("utf-8")).digest()
    base = np.array([digest[0], digest[1], digest[2]], dtype=np.float32)
    base = 95.0 + (base / 255.0) * 120.0
    if "RJ2506" in path:
        base = np.array([205.0, 205.0, 215.0])
    elif "Conveyor" in path:
        base = np.array([70.0, 105.0, 145.0])
    elif "Pallet" in path or "pallet" in path:
        base = np.array([155.0, 120.0, 85.0])
    return tuple(int(v) for v in base[::-1])


def transform_points(points: np.ndarray, matrix) -> np.ndarray:
    rows = [[float(matrix[i][j]) for j in range(4)] for i in range(4)]
    mat = np.array(rows, dtype=np.float64)
    hom = np.c_[points, np.ones(len(points), dtype=np.float64)]
    return hom @ mat


def load_triangles(
    scene_path: Path,
) -> tuple[list[tuple[np.ndarray, tuple[int, int, int], str]], np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None]:
    stage = Usd.Stage.Open(str(scene_path))
    if stage is None:
        raise RuntimeError(f"Failed to open USD stage: {scene_path}")

    cache = UsdGeom.XformCache()
    triangles: list[tuple[np.ndarray, tuple[int, int, int], str]] = []
    all_points: list[np.ndarray] = []
    robot_points: list[np.ndarray] = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        prim_path = str(prim.GetPath())
        if prim_path.startswith("/World/GroundPlane"):
            continue
        mesh = UsdGeom.Mesh(prim)
        points_attr = mesh.GetPointsAttr()
        counts_attr = mesh.GetFaceVertexCountsAttr()
        indices_attr = mesh.GetFaceVertexIndicesAttr()
        if not points_attr or not counts_attr or not indices_attr:
            continue

        points_value = points_attr.Get()
        counts = counts_attr.Get()
        indices = indices_attr.Get()
        if not points_value or not counts or not indices:
            continue

        local = np.array([[p[0], p[1], p[2]] for p in points_value], dtype=np.float64)
        world = transform_points(local, cache.GetLocalToWorldTransform(prim))[:, :3]
        all_points.append(world)
        if "RJ2506" in prim_path:
            robot_points.append(world)
        color = mesh_color(prim_path)

        cursor = 0
        for count in counts:
            face = indices[cursor : cursor + count]
            cursor += count
            if count < 3:
                continue
            for i in range(1, count - 1):
                tri = world[[face[0], face[i], face[i + 1]]]
                triangles.append((tri, color, prim_path))

    if not triangles or not all_points:
        raise RuntimeError(f"No renderable meshes found in {scene_path}")
    points = np.vstack(all_points)
    if robot_points:
        robot = np.vstack(robot_points)
        return triangles, points.min(axis=0), points.max(axis=0), robot.min(axis=0), robot.max(axis=0)
    return triangles, points.min(axis=0), points.max(axis=0), None, None


def camera_basis(eye: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    up /= np.linalg.norm(up)
    return right, up, forward


def render_frame(
    triangles: list[tuple[np.ndarray, tuple[int, int, int], str]],
    eye: np.ndarray,
    target: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (32, 34, 38)
    right, up, forward = camera_basis(eye, target)
    focal = 1.35 * width
    projected: list[tuple[float, np.ndarray, tuple[int, int, int], float]] = []
    light = np.array([-0.35, -0.55, 0.75], dtype=np.float64)
    light /= np.linalg.norm(light)

    for tri, color, _path in triangles:
        rel = tri - eye
        z = rel @ forward
        if np.any(z <= 0.05):
            continue
        x = rel @ right
        y = rel @ up
        pts = np.column_stack((width * 0.5 + focal * x / z, height * 0.52 - focal * y / z))
        if np.all((pts[:, 0] < -50) | (pts[:, 0] > width + 50)) or np.all((pts[:, 1] < -50) | (pts[:, 1] > height + 50)):
            continue
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        shade = 0.35 + 0.65 * max(0.0, float(normal @ light))
        projected.append((float(np.mean(z)), pts.astype(np.int32), color, shade))

    for _depth, pts, color, shade in sorted(projected, key=lambda item: item[0], reverse=True):
        shaded = tuple(int(max(0, min(255, c * shade))) for c in color)
        cv2.fillConvexPoly(image, pts, shaded, lineType=cv2.LINE_AA)
        cv2.polylines(image, [pts], True, (24, 25, 28), 1, lineType=cv2.LINE_AA)
    return image


def encode_video(frames_dir: Path, video_path: Path, fps: int, crf: int) -> None:
    video_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(fps),
        "-i",
        str(frames_dir / "rgb_%04d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        str(crf),
        str(video_path),
    ]
    subprocess.run(cmd, check=True)


def main() -> None:
    args = parse_args()
    scene = Path(args.scene).resolve()
    output_dir = Path(args.output_dir).resolve()
    video = Path(args.video).resolve()
    if args.clean and output_dir.exists():
        shutil.rmtree(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    triangles, min_v, max_v, focus_min, focus_max = load_triangles(scene)
    if args.max_triangles > 0 and len(triangles) > args.max_triangles:
        step_indices = np.linspace(0, len(triangles) - 1, args.max_triangles, dtype=np.int64)
        triangles = [triangles[int(index)] for index in step_indices]
    if focus_min is not None and focus_max is not None:
        min_v, max_v = focus_min, focus_max
    center = (min_v + max_v) * 0.5
    size = max_v - min_v
    target = np.array([center[0], center[1], center[2] + size[2] * 0.1], dtype=np.float64)
    radius = min(max(args.min_radius, float(max(size[0], size[1]) * 1.35)), 10.0)
    height = float(center[2] + max(2.2, size[2] * 0.75))
    frame_count = max(1, int(round(args.fps * args.duration)))

    print(f"[software_render] scene={scene}")
    print(f"[software_render] triangles={len(triangles)} frames={frame_count} output_dir={output_dir}")
    for frame_index in range(frame_count):
        angle = math.radians(args.orbit_degrees) * frame_index / max(frame_count, 1)
        eye = np.array(
            [
                target[0] + radius * math.cos(angle),
                target[1] + radius * math.sin(angle),
                height,
            ],
            dtype=np.float64,
        )
        image = render_frame(triangles, eye, target, args.width, args.height)
        cv2.imwrite(str(output_dir / f"rgb_{frame_index:04d}.png"), image)
        if frame_index == 0 or (frame_index + 1) % max(1, frame_count // 8) == 0:
            print(f"[software_render] captured {frame_index + 1}/{frame_count}", flush=True)

    encode_video(output_dir, video, args.fps, args.crf)
    print(f"[software_render] video={video}")


if __name__ == "__main__":
    main()
