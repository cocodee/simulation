## 2026-04-28 - DKK5 camera validation

- No new architecture changes were made for the remaining camera checks. Reused the existing `--camera-prim` path for wrist-camera sensor validation and the default Replicator-created camera path for the global view.

## 2026-04-28 - DKK5 camera sensor layer

- Created `dkk_5_camera_sensors.usd` as a lightweight reference layer over the original `dkk_5.usd` instead of editing the source asset in place.
- Authored camera sensors as `UsdGeom.Camera` children of the existing DKK5 camera mount Xforms:
  - `/World/RJ2506/cam_head/head_camera_sensor`
  - `/World/RJ2506/left_hand_front_cam/left_hand_camera_sensor`
  - `/World/RJ2506/right_hand_front_cam/right_hand_camera_sensor`
- Used USD/OpenGL camera convention from IsaacLab/Isaac Sim camera handling: camera forward is `-Z`, up is `+Y`. The local sensor orientation `(w, x, y, z) = (0.5, 0.5, -0.5, -0.5)` maps camera `-Z` onto each mount frame's `+X`.
- Extended the DKK5 video recorder with an optional `--camera-prim` argument instead of creating a separate recorder, preserving the previous external observer camera behavior by default.
