# dkk_2 Replicator 仿真视频生成流程

日期：2026-04-20

## 背景

目标是根据 `/home/kdi/workspace/IsaacSim_Rendering_Guide.md` 的指导，重新实现 `dkk_2.usd` 的 Isaac Sim 渲染脚本，并生成一段质量可用的新仿真视频。

本次工作不使用 Docker，使用本机 Isaac Sim：

```text
/root/data1/kdi/workspace/isaac/isaac-sim/python.sh
```

输入场景：

```text
/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_2.usd
```

最终采用 Isaac Sim Replicator + `BasicWriter` 直接输出 RGB 帧，再用 `ffmpeg` 编码 MP4。该路径替代了之前效果不理想的软件渲染视频。

## 工作成果

新增脚本：

```text
/root/data1/kdi/workspace/sim1/scripts/record_dkk2_replicator.py
/root/data1/kdi/workspace/sim1/scripts/run_dkk2_replicator_video.sh
```

最终视频：

```text
/root/data1/kdi/workspace/sim1/outputs/dkk_2_replicator_video.mp4
```

相关产物：

```text
/root/data1/kdi/workspace/sim1/outputs/dkk_2_replicator_manifest.json
/root/data1/kdi/workspace/sim1/outputs/dkk_2_replicator_frames/rgb_*.png
/root/data1/kdi/workspace/sim1/outputs/dkk_2_replicator_run.log
```

视频验证结果：

```text
codec: h264
resolution: 1280x720
fps: 24
encoded frames: 96
duration: 4.0s
size: 371451 bytes
```

Manifest 关键记录：

```text
reference_prim: /World/DKK2
prim_count: 283
mesh_count: 83
frame_count: 97
encoded_frame_count: 96
requested_frames: 96
warmup_steps: 30
camera_position: [-4.0, 4.0, 2.8]
look_at: [-1.6, 0.3, 1.1]
active_gpu: 0
multi_gpu: false
```

`frame_count` 比 `encoded_frame_count` 多 1 是 Replicator writer attach 时产生的初始化帧。脚本编码时只取最后 `requested_frames` 张帧，因此 MP4 帧数和时长是正确的。

## 复现命令

推荐使用 wrapper：

```bash
cd /root/data1/kdi/workspace/sim1
FRAMES=96 WARMUP_STEPS=30 FPS=24 WIDTH=1280 HEIGHT=720 scripts/run_dkk2_replicator_video.sh
```

可调环境变量：

```text
ISAAC_SIM_DIR      Isaac Sim 安装目录，默认 /root/data1/kdi/workspace/isaac/isaac-sim
SCENE_PATH         输入 USD，默认 supre_robot_assets/scenes/dkk_2.usd
OUTPUT_DIR         Replicator RGB 帧输出目录
VIDEO_PATH         MP4 输出路径
MANIFEST_PATH      manifest 输出路径
FRAMES             编码帧数
WARMUP_STEPS       writer attach 前的仿真 warmup 步数
FPS                输出视频帧率
WIDTH/HEIGHT       输出分辨率
ACTIVE_GPU         Isaac activeGpu，默认 0
MULTI_GPU          默认 false
CAMERA_POSITION    相机位置，默认 -4.0,4.0,2.8
LOOK_AT            相机朝向目标点，默认 -1.6,0.3,1.1
```

## 实现流程

核心脚本是 `scripts/record_dkk2_replicator.py`。流程如下：

1. 启动 `SimulationApp`，headless 模式，指定 `active_gpu=0`，关闭 multi-GPU。
2. 创建 Isaac `World(stage_units_in_meters=1.0)`。
3. 先添加灯光，再加载场景。
4. 使用 `stage.DefinePrim(...).GetReferences().AddReference(...)` 挂载 `dkk_2.usd`。
5. 将场景挂到 `/World/DKK2`，避免直接挂到 `/World` 与场景自身 default prim 冲突。
6. warmup 若干帧，等待 stage composition 和渲染资源准备。
7. 创建 Replicator camera 和 render product。
8. 使用 `BasicWriter` 输出 RGB 帧。
9. 每帧配对执行：

```python
simulation_app.update()
rep.orchestrator.step()
```

10. `rep.orchestrator.wait_until_complete()` 等 writer 完成。
11. 用 `ffmpeg + libx264` 将最后 `FRAMES` 张 `rgb_*.png` 编码成 MP4。
12. 写出 manifest，记录输入、输出、帧数、相机、GPU 等信息。

## 和指南一致的关键点

来自 `/home/kdi/workspace/IsaacSim_Rendering_Guide.md` 的经验被保留下来：

- 使用 `stage.DefinePrim() + AddReference()` 加载 USD。
- 显式添加灯光，避免渲染全黑。
- 使用 Replicator camera + render product + `BasicWriter`。
- 每帧同时推进 Isaac app 和 Replicator：

```python
simulation_app.update()
rep.orchestrator.step()
```

- 不使用 `rep.orchestrator.preview()`。
- 输出后检查真实落盘文件，而不是只相信 API 调用返回。

## 本次适配点

### 1. 场景挂载点使用 `/World/DKK2`

`dkk_2.usd` 自身包含 `/World` 和 default prim。最初直接 reference 到 `/World` 会触发复杂重组，容易卡住或提前关闭。

稳定做法：

```python
prim = stage.DefinePrim("/World/DKK2", "Xform")
prim.GetReferences().AddReference(str(scene_path))
```

这样场景内容最终位于：

```text
/World/DKK2/RJ2506
/World/DKK2/ConveyorBelt_A08
...
```

### 2. 灯光用低层 USD prim 创建

指南里使用：

```python
UsdLux.DomeLight.Define(...)
UsdLux.DirectionalLight.Define(...)
```

但在当前本地 Isaac Sim 环境中，`UsdLux.DomeLight.Define()` 会导致 app 提前关闭，且没有 Python traceback。

本次改用低层 USD prim 写法：

```python
dome_light = stage.DefinePrim("/World/DomeLight", "DomeLight")
dome_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(1800.0)

sun_light = stage.DefinePrim("/World/DirectionalLight", "DistantLight")
sun_light.CreateAttribute("inputs:intensity", Sdf.ValueTypeNames.Float).Set(900.0)
sun_light.CreateAttribute("inputs:angle", Sdf.ValueTypeNames.Float).Set(0.5)
```

这保留了“必须加灯光”的效果，同时避开当前环境里的 `UsdLux` 崩点。

### 3. 先加灯光，再 reference 场景

在场景 reference 后立刻操作 `/World` 添加 light，曾触发 app 提前关闭。稳定顺序是：

```text
create World
add lights
add dkk_2 reference
warmup
create camera/render product/writer
capture frames
```

### 4. 编码只取最后 `FRAMES` 张图

Replicator `BasicWriter` 在 attach 时会额外写一张初始化帧。为保证 MP4 的帧数、时长准确，编码时只取排序后的最后 `FRAMES` 张 `rgb_*.png`。

结果：

```text
frame_count: 97
encoded_frame_count: 96
nb_frames: 96
duration: 4.0s
```

## 已知日志和限制

当前运行仍会出现这些日志，但不影响视频生成：

```text
Multiple Installable Client Drivers (ICDs) are found for the same GPU
Could not open asset ... Conveyors/Textures/*.usd
References an asset that can not be found: Materials/Textures/FOF_Map_Palette_A_D.tga
References an asset that can not be found: Materials/Textures/FOF_Map_Palette_A_N.tga
Stiffness/Damping attribute is unsupported for articulation joints
```

含义：

- ICD 警告来自系统 Vulkan/NVIDIA 驱动安装重复。脚本通过 `--no-multi-gpu` 和 `active_gpu=0` 规避了实际运行风险。
- conveyor 和 FOF 贴图缺失是资产包内容问题，会影响部分材质外观，但不阻止场景加载、仿真推进和 RGB 帧输出。
- articulation joint 的 stiffness/damping 警告来自 PhysX schema 兼容性，不影响本次视频输出。

## 验证方法

检查视频元信息：

```bash
ffprobe -v error \
  -show_entries format=duration,size \
  -show_entries stream=codec_name,width,height,r_frame_rate,nb_frames \
  -of default=noprint_wrappers=1 \
  outputs/dkk_2_replicator_video.mp4
```

期望输出：

```text
codec_name=h264
width=1280
height=720
r_frame_rate=24/1
nb_frames=96
duration=4.000000
```

检查帧目录：

```bash
find outputs/dkk_2_replicator_frames -maxdepth 1 -type f -name 'rgb_*.png' | wc -l
```

当前目录为 97 张，其中 96 张被编码进 MP4。

## 可复用经验

1. 对 Isaac headless 渲染，优先使用 Replicator `BasicWriter`，比 viewport capture 更稳定。
2. 对复杂 USD 场景，不要盲目 reference 到 `/World`；如果原 USD 自身 default prim 也是 `/World`，挂到 `/World/<SceneName>` 更安全。
3. `UsdLux` schema API 在某些 Isaac 环境里可能导致无 traceback 退出；必要时可以直接 `DefinePrim(typeName)` 并写 `inputs:*` 属性。
4. Replicator attach 可能产生额外初始化帧，编码阶段应显式选择需要的帧范围。
5. 每个视频任务都应该写 manifest，记录输入、输出、帧数、相机和 GPU 参数，方便复现和排查。
6. 必须用 `ffprobe` 验证最终 MP4，不只看脚本是否退出成功。
7. 资产缺失警告和渲染失败要区分：材质贴图缺失可能只影响外观，不一定阻止视频生成。

## 当前推荐路径

后续类似 `dkk_2.usd` 的本地 Isaac 视频生成任务，优先使用：

```bash
cd /root/data1/kdi/workspace/sim1
scripts/run_dkk2_replicator_video.sh
```

需要调整镜头时，优先改环境变量：

```bash
CAMERA_POSITION="-5.0,3.5,3.0" LOOK_AT="-1.6,0.3,1.1" scripts/run_dkk2_replicator_video.sh
```

需要更长视频时：

```bash
FRAMES=240 FPS=24 scripts/run_dkk2_replicator_video.sh
```
