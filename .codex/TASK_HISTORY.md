## 2026-05-13T04:52:27Z - Recorded simulation next-step recommendations

- Changed files:
  - `docs/simulation_next_steps.md`: added prioritized simulation next steps and the suggested immediate task.
  - `.codex/TASK_HISTORY.md`: appended this summary.
- Commands run:
  - `sed -n '1,120p' .codex/TASK_HISTORY.md`
  - `sed -n '1,120p' .codex/DECISIONS.md`
  - `rg --files docs .codex`
  - `date -u +%Y-%m-%dT%H:%M:%SZ`
- Results:
  - Captured the recommendations in a durable project document.
  - No simulation code was changed.
- Unresolved issues:
  - The next implementation priority still needs to be selected before coding begins.
- Next steps:
  - Start with DKK5 carry workflow task-success metrics unless project direction changes.

## 2026-05-13T04:50:57Z - Simulation next-work assessment

- Changed files:
  - `.codex/TASK_HISTORY.md`: appended this assessment summary.
- Commands run:
  - `sed -n '1,220p' .codex/TASK_HISTORY.md`
  - `sed -n '1,220p' .codex/DECISIONS.md`
  - `rg --files`
  - `sed -n '1,220p' docs/dkk2_video_work_notes.md`
  - `sed -n '1,220p' docs/dkk2_replicator_rendering_workflow.md`
  - `find scripts -maxdepth 1 -type f -name '*.sh' -o -name '*.py' | sort`
  - `find outputs -maxdepth 2 -name 'manifest.json' -o -name '*manifest*.json' | sort | tail -40`
  - `rg -n "TODO|FIXME|Next steps|Unresolved|unresolved|warning|warn|manifest|collision|physics|drive|carry|camera" docs scripts .codex -S`
  - `sed -n '1,220p' scripts/record_dkk5_diff_motion_video.py`
  - `sed -n '1,220p' scripts/record_dkk5_diff_carry_video.py`
- Results:
  - Reviewed current DKK2/DKK5 simulation, rendering, recording, camera-sensor, and manifest coverage.
  - No simulation code was changed.
- Unresolved issues:
  - Existing source-scene material/conveyor texture warnings remain documented.
  - Need project-level decision on whether next priority is physics fidelity, task success metrics, sensor dataset output, or scenario coverage.
- Next steps:
  - Prioritize physics/contact validation, objective metrics in manifests, scenario matrix expansion, and repeatable regression smoke tests.

## 2026-04-28T03:36:58Z - DKK5 remaining camera validations

- Changed files:
  - `.codex/TASK_HISTORY.md`: appended this validation summary.
- Generated outputs:
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_left_test/dkk5_left_hand_camera_sensor.mp4`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_left_test/manifest.json`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_right_test/dkk5_right_hand_camera_sensor.mp4`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_right_test/manifest.json`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_global_test/dkk5_global_camera_view.mp4`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_global_test/manifest.json`
- Commands run:
  - `/home/kdi/workspace/isaac/isaac-sim/python.sh scripts/record_dkk5_diff_motion_video.py --camera-prim /World/DKK2/RJ2506/left_hand_front_cam/left_hand_camera_sensor ...`
  - `/home/kdi/workspace/isaac/isaac-sim/python.sh scripts/record_dkk5_diff_motion_video.py --camera-prim /World/DKK2/RJ2506/right_hand_front_cam/right_hand_camera_sensor ...`
  - `/home/kdi/workspace/isaac/isaac-sim/python.sh scripts/record_dkk5_diff_motion_video.py --camera-position=-4.0,4.0,2.8 --look-at=-1.6,0.3,1.1 ...`
  - `ffprobe` on all three MP4 files
  - Python/PIL frame-stat checks on first, middle, and last frame sets
- Results:
  - Left-hand camera video: 640x360, 24fps, 3.0s, 72 encoded frames.
  - Right-hand camera video: 640x360, 24fps, 3.0s, 72 encoded frames.
  - Global camera video: 640x360, 24fps, 3.0s, 72 encoded frames.
  - Pixel stats confirmed non-empty PNG frames for all three views.
- Unresolved issues:
  - Existing Isaac/source-scene material and conveyor texture warnings still appear but did not block rendering or encoding.
  - Global output directory contained 73 PNG frames while the encoded video used the requested 72 frames.
- Next steps:
  - Review the three MP4 files visually and tune camera local orientation or focal length only if the view framing is not acceptable.

## 2026-04-28T02:46:57Z - DKK5 camera sensor USD

- Changed files:
  - `scripts/create_dkk5_camera_sensor_usd.py`: added a generator for `/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5_camera_sensors.usd`.
  - `scripts/record_dkk5_diff_motion_video.py`: added `--camera-prim` support so videos can be rendered from an existing USD camera sensor prim.
- Generated assets:
  - `/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5_camera_sensors.usd`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk_5_camera_sensors_manifest.json`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_test/dkk5_head_camera_sensor.mp4`
  - `/root/data1/kdi/workspace/sim1/outputs/dkk5_camera_sensor_test/manifest.json`
- Camera mount inspection:
  - Source `dkk_5.usd` contains camera mount Xforms at `/World/RJ2506/cam_head`, `/World/RJ2506/left_hand_front_cam`, and `/World/RJ2506/right_hand_front_cam`, but no authored `UsdGeom.Camera` prims.
  - Added `head_camera_sensor`, `left_hand_camera_sensor`, and `right_hand_camera_sensor` as child `Camera` prims under those mounts.
- Commands run:
  - `python3 scripts/create_dkk5_camera_sensor_usd.py`
  - `python3 -m py_compile scripts/create_dkk5_camera_sensor_usd.py scripts/record_dkk5_diff_motion_video.py`
  - `python3` USD validation snippets using `pxr.Usd` and `UsdGeom.Camera`
  - `/home/kdi/workspace/isaac/isaac-sim/python.sh scripts/record_dkk5_diff_motion_video.py --scene /root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_5_camera_sensors.usd --camera-prim /World/DKK2/RJ2506/cam_head/head_camera_sensor ...`
  - `ffprobe` and a PIL frame-stat check on the generated MP4/PNG frames
- Results:
  - New USD opens and all three camera prims are valid `UsdGeom.Camera` prims with focal length `18.0` and clipping range `(0.1, 1000.0)`.
  - Test simulation rendered 72 RGB frames at 640x360 and encoded a 3.0s 24fps MP4 with 72 frames.
  - Pixel stats on first/middle/last frames showed non-empty frame content.
- Unresolved issues:
  - Isaac/USD still reports pre-existing missing conveyor/material texture assets from the source scene; these warnings also appear when opening the source scene and did not block camera-sensor rendering.
- Next steps:
  - Use `--camera-prim /World/DKK2/RJ2506/left_hand_front_cam/left_hand_camera_sensor` or `/World/DKK2/RJ2506/right_hand_front_cam/right_hand_camera_sensor` for wrist-camera validation videos if needed.
