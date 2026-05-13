#!/usr/bin/env python3
"""Create a DKK5 USD layer with camera sensor prims at the robot camera mounts."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom


CAMERA_MOUNTS = {
    "head_camera_sensor": "/World/RJ2506/cam_head",
    "left_hand_camera_sensor": "/World/RJ2506/left_hand_front_cam",
    "right_hand_camera_sensor": "/World/RJ2506/right_hand_front_cam",
}

# USD cameras use the OpenGL convention: forward is -Z and up is +Y. The DKK
# camera mount frames use +X as the optical forward axis, so rotate the camera
# child so its -Z axis points along the mount frame's +X axis.
CAMERA_LOCAL_ORIENT_WXYZ = (0.5, 0.5, -0.5, -0.5)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        default="/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5.usd",
        help="Source DKK5 USD scene.",
    )
    parser.add_argument(
        "--output",
        default="/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5_camera_sensors.usd",
        help="Output USD layer containing a reference to the source scene plus camera sensor prims.",
    )
    parser.add_argument(
        "--manifest",
        default="/root/data1/kdi/workspace/sim1/outputs/dkk_5_camera_sensors_manifest.json",
        help="JSON manifest describing the generated camera prims.",
    )
    parser.add_argument("--focal-length", type=float, default=18.0)
    parser.add_argument("--horizontal-aperture", type=float, default=20.955)
    parser.add_argument("--vertical-aperture", type=float, default=15.2908)
    parser.add_argument("--clipping-range", type=float, nargs=2, default=(0.1, 1000.0))
    parser.add_argument("--focus-distance", type=float, default=400.0)
    parser.add_argument("--f-stop", type=float, default=0.0)
    return parser.parse_args()


def require_mounts(source_stage: Usd.Stage) -> None:
    missing = [path for path in CAMERA_MOUNTS.values() if not source_stage.GetPrimAtPath(path).IsValid()]
    if missing:
        raise RuntimeError(f"Missing camera mount prims in source scene: {missing}")


def configure_camera(
    stage: Usd.Stage,
    mount_path: str,
    camera_name: str,
    args: argparse.Namespace,
) -> str:
    stage.OverridePrim(mount_path)
    camera_path = f"{mount_path}/{camera_name}"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.CreateProjectionAttr("perspective")
    camera.CreateFocalLengthAttr(args.focal_length)
    camera.CreateHorizontalApertureAttr(args.horizontal_aperture)
    camera.CreateVerticalApertureAttr(args.vertical_aperture)
    camera.CreateClippingRangeAttr(Gf.Vec2f(*args.clipping_range))
    camera.CreateFocusDistanceAttr(args.focus_distance)
    camera.CreateFStopAttr(args.f_stop)

    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))
    w, x, y, z = CAMERA_LOCAL_ORIENT_WXYZ
    xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    xform.AddScaleOp().Set(Gf.Vec3d(1.0, 1.0, 1.0))
    return camera_path


def main() -> None:
    args = parse_args()
    source_path = Path(args.source).resolve()
    output_path = Path(args.output).resolve()
    manifest_path = Path(args.manifest).resolve()

    if not source_path.is_file():
        raise FileNotFoundError(f"Source USD not found: {source_path}")

    source_stage = Usd.Stage.Open(str(source_path))
    if source_stage is None:
        raise RuntimeError(f"Failed to open source USD: {source_path}")
    require_mounts(source_stage)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(source_stage))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(source_stage))

    world = stage.DefinePrim("/World", "Xform")
    world.GetReferences().AddReference(str(source_path), "/World")
    stage.SetDefaultPrim(world)

    camera_paths = {}
    for camera_name, mount_path in CAMERA_MOUNTS.items():
        camera_paths[camera_name] = configure_camera(stage, mount_path, camera_name, args)

    stage.GetRootLayer().Save()

    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    manifest = {
        "source": str(source_path),
        "output": str(output_path),
        "camera_paths": camera_paths,
        "camera_local_orient_wxyz": CAMERA_LOCAL_ORIENT_WXYZ,
        "camera_mounts": CAMERA_MOUNTS,
        "focal_length": args.focal_length,
        "horizontal_aperture": args.horizontal_aperture,
        "vertical_aperture": args.vertical_aperture,
        "clipping_range": list(args.clipping_range),
        "focus_distance": args.focus_distance,
        "f_stop": args.f_stop,
        "notes": [
            "Camera prims are authored as children of the existing DKK5 camera mount Xforms.",
            "The local orientation maps the USD/OpenGL camera forward axis (-Z) onto each mount frame's +X axis.",
        ],
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"created_usd={output_path}")
    print(f"manifest={manifest_path}")
    for name, path in camera_paths.items():
        print(f"{name}={path}")


if __name__ == "__main__":
    main()
