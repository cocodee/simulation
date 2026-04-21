# dkk_2 场景生成视频工作总结

日期：2026-04-20

## 背景

目标是使用 `../supre_robot_assets/scenes/dkk_2.usd` 运行或渲染仿真，并生成一段视频。约束是使用 `../supre_robot_assets/scenes` 里的资源，不依赖也不关注 `dkk_simulation`。

最终交付了一个独立于 `dkk_simulation` 的视频生成入口：

```bash
cd /root/data1/kdi/workspace/sim1
scripts/run_dkk2_orbit.sh all
```

默认读取：

```text
/root/data1/kdi/workspace/supre_robot_assets/scenes/dkk_2.usd
```

默认输出：

```text
/root/data1/kdi/workspace/sim1/outputs/dkk_2_orbit.mp4
/root/data1/kdi/workspace/sim1/outputs/dkk_2_orbit_frames/rgb_*.png
```

## 已产出的工具

### `scripts/run_dkk2_orbit.sh`

统一入口脚本。默认使用 `software` 后端，可稳定从 USD mesh 生成 orbit 预览视频。

常用参数：

```bash
DURATION=4 FPS=12 WIDTH=960 HEIGHT=540 MAX_TRIANGLES=80000 scripts/run_dkk2_orbit.sh all
```

关键环境变量：

```text
BACKEND=software      默认，直接读 USD mesh，软件投影渲染并编码视频
BACKEND=isaac         尝试使用 Isaac Sim 打开场景和 capture
DURATION              视频时长，默认 8
FPS                   帧率，默认 24
WIDTH/HEIGHT          输出分辨率，默认 1280x720
MAX_TRIANGLES         软件渲染三角面预算，默认 80000
OUTPUT_DIR            帧输出目录
VIDEO_PATH            MP4 输出路径
```

### `scripts/render_dkk2_software.py`

可复用的软件预览渲染器。它直接用 `pxr.Usd` 读取 USD stage，遍历 `UsdGeom.Mesh`，用 `UsdGeom.XformCache` 得到世界坐标，将三角面按 orbit 相机投影成 PNG，再调用 `ffmpeg` 编码 MP4。

适用场景：

- 需要快速确认一个 USD 场景能否生成视频。
- Isaac Sim headless 渲染输出不稳定。
- 不需要 RTX 级真实材质，只需要结构清晰的 orbit preview。

限制：

- 不是真实 Isaac/RTX 渲染。
- 不解析完整材质网络，只按 prim path 生成近似颜色。
- 不运行物理仿真，只做静态场景预览。

### `scripts/record_dkk2_orbit.py`

Isaac Sim 版本的尝试脚本。它能打开 `dkk_2.usd`，并且保留了自动取景、加灯、相机轨道等逻辑。但在当前 headless/no-window 容器环境里，Replicator、renderer capture 和 viewport capture 都遇到输出层不落盘或阻塞的问题。

后续如果有可用显示环境、EGL 配置或更完整的 Isaac 渲染容器，可以继续从这个脚本恢复 Isaac 后端。

### `scripts/encode_video.py`

独立的 `rgb_*.png` 到 MP4 编码工具，使用 `ffmpeg + libx264`。

### `scripts/run_dkk2_local_isaac.sh`

无 Docker 的本地 Isaac Sim 入口。默认使用：

```text
/root/data1/kdi/workspace/isaac/isaac-sim/python.sh
```

它会调用 `scripts/simulate_dkk2_local.py`，直接打开 `dkk_2.usd`，启动 timeline，推进指定仿真步数，并写出 manifest。

常用命令：

```bash
STEPS=120 WARMUP_STEPS=30 scripts/run_dkk2_local_isaac.sh
```

默认输出：

```text
/root/data1/kdi/workspace/sim1/outputs/dkk_2_local_isaac_manifest.json
```

### `scripts/simulate_dkk2_local.py`

本地 Isaac 仿真脚本。它不依赖 Docker，不使用 `dkk_simulation`，主要流程是：

1. 用本地 Isaac Sim `SimulationApp` 启动 headless runtime。
2. 用 `omni.usd.get_context().open_stage()` 打开 `dkk_2.usd`。
3. warmup 若干帧。
4. `timeline.play()` 后推进 `simulation_app.update()`。
5. 写出 prim、mesh、步数和耗时等 manifest。

## 关键经验教训

### 1. 不要用错误容器假设路径已经挂载

已有容器 `wheel-isaac-sim` 并没有挂载 `/root/data1/kdi/workspace`，它看不到本次需要的 `supre_robot_assets/scenes/dkk_2.usd`。

可复用判断：

```bash
docker ps --format '{{.ID}} {{.Image}} {{.Names}} {{.Status}}'
docker inspect <container> --format '{{json .Mounts}}'
```

如果目标资产不在已有容器挂载里，应新起容器并显式挂载：

```bash
docker run --rm --gpus all \
  -v /root/data1/kdi/workspace:/workspace \
  -w /workspace/sim1 \
  ...
```

### 2. Docker image 的默认 entrypoint 可能吞掉命令

`wheel-loading-sim:latest` 默认 entrypoint 会启动 Isaac streaming/headless app，直接把脚本参数吞掉。解决方式是显式覆盖：

```bash
--entrypoint bash
```

后续类似任务里，只要发现 `docker run image command` 没有执行预期脚本，应第一时间检查 entrypoint。

### 3. 挂载目录权限要提前处理

容器默认用户无法删除或写入宿主输出目录时，会出现权限错误。当前 wrapper 用 root 运行 Isaac 容器，并在结束后 `chown` 回宿主用户。

模式：

```bash
--user root
...
chown -R "$HOST_UID:$HOST_GID" "$CONTAINER_OUTPUT_DIR"
```

软件后端直接在宿主运行，不会制造 root-owned 输出。

### 4. 对 USD stage，`open_stage` 比 `Import` 更适合保留相对引用

最初尝试把 `dkk_2.usd` import 到新 stage，导致相对引用如 `../assets/...` 解析失败。直接打开原 stage 能保留正确的 resolver context：

```python
context = omni.usd.get_context()
context.open_stage(str(scene_path))
stage = context.get_stage()
```

经验：对包含大量相对 payload/reference 的 USD 场景，优先直接 `Open/open_stage`，不要先导入到空 stage。

### 5. Isaac `World.reset()` 不适合随便套到任意资产场景

对这个 scene，`World.reset()` 触发了 physics schema invalid prim 相关错误。该任务只是渲染预览，不需要物理 reset。

经验：静态场景预览优先只打开 USD、推进 app update、摆相机、渲染。不要无必要创建 `World` 或 `world.reset()`。

### 6. Replicator `step()` 返回不代表 writer 已经落盘

本次 BasicWriter 出现了：

```text
captured 1/1
Replicator did not write any rgb_*.png frames
```

即使加了：

```python
rep.orchestrator.wait_until_complete()
```

仍未产出 PNG。

经验：Replicator 任务必须在脚本内显式检查输出文件存在，而不是只相信 API 返回：

```python
frames = sorted(output_dir.rglob("rgb_*.png"))
if not frames:
    raise RuntimeError("no frames written")
```

### 7. headless/no-window 下 viewport/swapchain capture 可能阻塞

尝试过：

- `omni.renderer_capture.capture_next_frame_swapchain`
- `capture_next_frame_rp_resource`
- `omni.kit.capture.viewport.CaptureExtension`
- `viewport.wait_for_rendered_frames()`

在当前容器环境里都无法稳定落盘，部分路径会卡住。

经验：headless 渲染要设置硬超时和输出检查。不要让 capture API 无限等待。

### 8. 某些 Kit 扩展存在于镜像里，但默认没启用

`omni.kit.capture.viewport` 文件存在，但 base python kit 里默认不能 import。需要显式启用：

```python
import omni.kit.app

omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
    "omni.kit.capture.viewport",
    True,
)
```

经验：遇到 `ModuleNotFoundError: No module named 'omni.kit.capture'` 时，不要只查 pip 包，应先查 Kit extension 是否启用。

### 9. USD 资产 warning 不一定是致命错误

`dkk_2.usd` 打开时会报 Conveyor 贴图/材质引用缺失，例如：

```text
Could not open asset ... Conveyors/Textures/...
```

但 mesh 仍能读取，场景结构仍可渲染。

经验：区分材质/贴图 warning 和 stage/mesh 加载失败。可先验证：

```python
from pxr import Usd, UsdGeom

stage = Usd.Stage.Open(scene_path)
meshes = [prim for prim in stage.Traverse() if prim.IsA(UsdGeom.Mesh)]
print(len(meshes))
```

### 10. 软件预览渲染是一个实用 fallback

当 Isaac/RTX/headless capture 层不稳定时，可以先用 USD Python API 做结构预览：

1. `Usd.Stage.Open(scene_path)`
2. 遍历 `UsdGeom.Mesh`
3. `UsdGeom.XformCache().GetLocalToWorldTransform(prim)`
4. 三角化 face
5. orbit camera 投影
6. `cv2.fillConvexPoly` 绘制
7. `ffmpeg` 编码

这条路径不能替代真实渲染，但能快速产出可检查的视频，是很好的工程 fallback。

### 11. 本地 Isaac Sim 路径优先避免 Docker 挂载问题

当 `/root/data1/kdi/workspace/isaac/isaac-sim` 已经存在时，可以直接使用本地 `python.sh`，不再使用 Docker：

```bash
/root/data1/kdi/workspace/isaac/isaac-sim/python.sh scripts/simulate_dkk2_local.py
```

本地运行时仍要注意：

- headless 环境会有 `GLFW initialization failed` warning，这是无显示环境下的预期现象。
- 系统缺 `libGLU.so.1` 时，Iray/MDL 会报 `libneuray.so` 加载错误；最小仿真仍可完成，但真实渲染可能受影响。
- 机器上 Vulkan ICD 重复时，Isaac 会重复枚举 GPU。应禁用 multi-GPU 并固定 `active_gpu=0`。

当前脚本默认采用：

```python
{
    "active_gpu": 0,
    "multi_gpu": False,
    "headless": True,
}
```

## 后续类似任务的推荐流程

### 快速路径

1. 确认资产路径存在：

```bash
ls -lh ../supre_robot_assets/scenes/dkk_2.usd
```

2. 先用 USD API 验证 mesh 可读：

```bash
python - <<'PY'
from pxr import Usd, UsdGeom
stage = Usd.Stage.Open("../supre_robot_assets/scenes/dkk_2.usd")
print("stage", bool(stage))
print("meshes", sum(1 for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)))
PY
```

3. 先用软件后端生成 smoke video：

```bash
DURATION=0.5 FPS=2 WIDTH=320 HEIGHT=180 scripts/run_dkk2_orbit.sh all
```

4. 再提高分辨率和帧数：

```bash
DURATION=4 FPS=12 WIDTH=960 HEIGHT=540 MAX_TRIANGLES=80000 scripts/run_dkk2_orbit.sh all
```

### 需要真实 Isaac 渲染时

1. 先确认容器挂载和 entrypoint。
2. 用 `omni.usd.get_context().open_stage()` 打开 scene。
3. 不要默认 `World.reset()`。
4. 使用最小帧数 smoke test。
5. 每一步都检查输出目录，而不是只看 API log。
6. 如果 capture 阻塞，杀掉容器并改 fallback，不要无限等待。

### 需要本地 Isaac 仿真时

1. 确认本地安装：

```bash
ls -lh /root/data1/kdi/workspace/isaac/isaac-sim/python.sh
ls -lh /root/data1/kdi/workspace/isaac/isaac-sim-assets
```

2. 最小 smoke test：

```bash
STEPS=10 WARMUP_STEPS=5 WIDTH=640 HEIGHT=360 \
  OUTPUT_PATH=/root/data1/kdi/workspace/sim1/outputs/dkk_2_local_isaac_smoke_manifest.json \
  scripts/run_dkk2_local_isaac.sh
```

3. 正式推进仿真：

```bash
STEPS=120 WARMUP_STEPS=30 scripts/run_dkk2_local_isaac.sh
```

4. 检查 manifest：

```bash
cat outputs/dkk_2_local_isaac_manifest.json
```

## 可沉淀为技能的能力点

### USD 场景体检

目标：快速判断一个 USD 是否能打开、相对引用是否能解析、mesh 是否存在、bbox 是否合理。

核心工具：

- `pxr.Usd`
- `pxr.UsdGeom`
- `UsdGeom.BBoxCache`
- `UsdGeom.XformCache`

### Docker + Isaac Sim 运行诊断

目标：确认容器是否看得到资产、entrypoint 是否正确、用户权限是否正确、GPU 是否可用。

核心命令：

```bash
docker ps
docker inspect
docker run --rm --gpus all --entrypoint bash -v ...
```

### Headless 渲染输出验证

目标：避免 “API 说完成但没有文件”。

核心规则：

- capture 后立即 `find output_dir`
- 脚本内检查 PNG 是否存在且 size > 0
- 所有 capture wait 都要有 timeout

### USD 软件预览渲染

目标：在真实渲染不可用时仍能生成结构预览视频。

核心工具：

- `pxr.Usd` 读取 stage
- `cv2` 绘制三角面
- `ffmpeg` 编码 MP4

## 当前遗留问题

1. Isaac Sim 后端目前能打开 `dkk_2.usd`，但当前 headless 容器的 capture 输出层无法稳定落盘。
2. 软件后端生成的是结构预览，不是真实材质和真实光照。
3. `dkk_2.usd` 资产中 Conveyor 的部分材质/贴图引用缺失，真实渲染时仍会出现 warning。
4. `MAX_TRIANGLES` 是性能和视觉细节的折中。值越大越细，但软件渲染越慢。

## 推荐保留的命令

生成默认视频：

```bash
cd /root/data1/kdi/workspace/sim1
scripts/run_dkk2_orbit.sh all
```

快速 smoke test：

```bash
DURATION=0.5 FPS=2 WIDTH=320 HEIGHT=180 scripts/run_dkk2_orbit.sh all
```

较清晰的短视频：

```bash
DURATION=4 FPS=12 WIDTH=960 HEIGHT=540 MAX_TRIANGLES=80000 scripts/run_dkk2_orbit.sh all
```

尝试 Isaac 后端：

```bash
BACKEND=isaac DURATION=0.5 FPS=2 WIDTH=320 HEIGHT=180 scripts/run_dkk2_orbit.sh all
```
