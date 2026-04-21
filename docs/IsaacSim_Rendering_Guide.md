# Isaac Sim 渲染指南

> 整合自 `wheel/docs/TROSSEN_RENDERING_GUIDE.md` 和 `wheel/CLAUDE.md`

---

## 概述

本文档记录在 NVIDIA Isaac Sim (Isaac Lab 2.3.0 + Isaac Sim 5.1.0, RTX H100) 环境中成功渲染 Trossen mobile_ai 轮式机器人的完整流程。

**核心成果**：
- 物理仿真正常（机器人可移动 0.38m）
- Replicator 成功捕获 212 帧图像
- 在 H100 GPU headless 模式下完成

---

## 成功脚本

```python
#!/usr/bin/env python3
"""Trossen mobile_ai - Working render with physics"""
import os
os.environ['PYTHONUNBUFFERED'] = '1'

from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True, "width": 1280, "height": 720})

from isaacsim.core.api import World
import omni.timeline
from pxr import Usd, UsdLux, UsdGeom, UsdPhysics
import numpy as np
import omni.replicator.core as rep

print("=== Trossen Rendering ===")

# 1. 创建世界
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
stage = world.stage

# 2. 加载机器人 USD
os.chdir("/workspace/trossen_ai_isaac/assets/robots/mobile_ai")
robot_prim = stage.DefinePrim("/World/mobile_ai", "Xform")
robot_prim.GetReferences().AddReference("./mobile_ai.usd")

# 3. 修复 broken reference（Trossen USD 的内部引用问题）
broken_visuals = stage.GetPrimAtPath("/World/mobile_ai/base_footprint/visuals")
if broken_visuals and broken_visuals.IsValid():
    refs = broken_visuals.GetReferences()
    refs.SetReferences([])  # 清除损坏的引用

# 4. 添加灯光（必须！否则全黑）
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1500)
Dir_light = UsdLux.DomeLight.Define(stage, "/World/Dir")
Dir_light.CreateIntensityAttr(800)

# 5. 设置 Replicator
camera = rep.create.camera(position=(-3.0, 3.0, 2.5), look_at=(0, 0.5, 0))
rp = rep.create.render_product(camera, (1280, 720))
writer = rep.WriterRegistry.get("BasicWriter")
output_dir = "/tmp/outputs/trossen"
os.makedirs(output_dir, exist_ok=True)
writer.initialize(output_dir=output_dir, rgb=True)
writer.attach([rp])

# 6. 设置轮子速度控制
def set_wheel_velocity(velocity):
    for jpath in ["/World/mobile_ai/joints/left_wheel", "/World/mobile_ai/joints/right_wheel"]:
        j = stage.GetPrimAtPath(jpath)
        if j and j.IsValid():
            drive = UsdPhysics.DriveAPI.Get(j, "angular")
            if drive:
                drive.CreateTypeAttr("velocity")
                drive.CreateDampingAttr(1000.0)
                drive.CreateMaxForceAttr(100000.0)
                drive.CreateTargetVelocityAttr(velocity)

# 7. 获取 chassis 用于位置跟踪
chassis = stage.GetPrimAtPath("/World/mobile_ai/base_footprint")
chassis_xf = UsdGeom.Xformable(chassis) if chassis.IsValid() else None

# 8. 重置世界并开始仿真
world.reset()
omni.timeline.get_timeline_interface().play()
for _ in range(30):
    simulation_app.update()

# 记录起始位置
if chassis_xf:
    m0 = chassis_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos0 = np.array([m0[3][0], m0[3][1], m0[3][2]])
    print(f"Start: ({pos0[0]:.3f}, {pos0[1]:.3f}, {pos0[2]:.3f})")

# 9. 启动轮子
set_wheel_velocity(90.0)
print("Running...")

# 10. 渲染循环
for i in range(180):
    simulation_app.update()
    rep.orchestrator.step()

    if i % 30 == 0 and chassis_xf:
        m = chassis_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        p = np.array([m[3][0], m[3][1], m[3][2]])
        print(f"Step {i}: ({p[0]:.3f}, {p[1]:.3f})")

# 11. 计算移动距离
if chassis_xf and pos0 is not None:
    m1 = chassis_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    pos1 = np.array([m1[3][0], m1[3][1], m1[3][2]])
    dist = np.linalg.norm(pos1[:2] - pos0[:2])
    print(f"Distance: {dist:.4f}m")

print("Done!")
simulation_app.close()
```

---

## 关键要点

### 1. USD 加载方式

**正确方式**：
```python
stage.DefinePrim("/World/mobile_ai", "Xform")
stage.GetPrimAtPath("/World/mobile_ai").GetReferences().AddReference(usd_path)
```

**错误方式**（会导致 Replicator 崩溃）：
```python
from isaacsim.core.utils.stage import add_reference_to_stage
add_reference_to_stage(usd_path=usd_path, prim_path="/World/mobile_ai")
```

### 2. 灯光设置

**必须添加**，否则渲染全黑：
```python
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(1500)
dir_light = UsdLux.DirectionalLight.Define(stage, "/World/DirectionalLight")
dir_light.CreateIntensityAttr(800)
dir_light.CreateAngleAttr(0.5)
```

### 3. Broken Reference 修复

Trossen USD 存在内部引用错误：
```
Unresolved reference prim path @mobile_ai.usd@</visuals/base_footprint>
```

解决方法是清除损坏的引用：
```python
broken_visuals = stage.GetPrimAtPath("/World/mobile_ai/base_footprint/visuals")
if broken_visuals and broken_visuals.IsValid():
    refs = broken_visuals.GetReferences()
    refs.SetReferences([])
```

### 4. 轮子速度控制

使用 `UsdPhysics.DriveAPI`：
```python
drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
drive.CreateTypeAttr("velocity")           # velocity 模式
drive.CreateDampingAttr(1000.0)            # 阻尼
drive.CreateMaxForceAttr(100000.0)         # 最大力
drive.CreateTargetVelocityAttr(velocity)    # 目标速度 (rad/s)
```

### 5. Replicator 配置

```python
# 创建相机
camera = rep.create.camera(position=(-3.0, 3.0, 2.5), look_at=(0, 0.5, 0))

# 创建渲染产品
rp = rep.create.render_product(camera, (1280, 720))

# 设置写入器
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="/tmp/outputs", rgb=True)
writer.attach([rp])

# 每帧渲染
rep.orchestrator.step()  # 不要用 rep.orchestrator.preview()
```

### 6. 渲染 vs 物理更新

```python
for i in range(180):
    simulation_app.update()    # 物理世界进一步
    rep.orchestrator.step()   # Replicator 渲染一帧
```

两个必须配对调用，缺一不可。

---

## 高质量渲染配置

```python
import carb
settings = carb.settings.get_settings()

# 路径追踪采样
settings.set("/rtx/pathtracing/spp", 256)           # 每帧采样数
settings.set("/rtx/pathtracing/totalSpp", 1024)      # 总采样数

# 降噪
settings.set("/rtx/pathtracing/optixDenoiser/enabled", True)

# DLSS + 抗锯齿
settings.set("/rtx/post/dlss/execMode", 1)
settings.set("/rtx/post/aa/op", 2)
```

---

## 问题与解决

| 问题 | 原因 | 解决方案 |
|------|------|---------|
| Replicator 崩溃 | `add_reference_to_stage` API 不兼容 | 改用 `stage.DefinePrim() + AddReference()` |
| 渲染全黑 | 缺少灯光 | 添加 DomeLight + DirectionalLight |
| Broken reference 警告 | USD 内部路径引用错误 | 清除损坏的 `base_footprint/visuals` 引用 |
| 机器人不移动 | DriveAPI 未正确设置 | 必须设置 `TypeAttr("velocity")` + `TargetVelocityAttr()` |
| 帧全黑 | USD 内部引用导致视觉组件未加载 | 使用 `stage.Export()` 或清除 broken refs |

---

## 生成 GIF

使用 PIL 将渲染帧合成为 GIF：

```python
from PIL import Image
import os

frames_dir = '/tmp/outputs/trossen'
frames = sorted([f for f in os.listdir(frames_dir)
                if f.startswith('rgb_') and f.endswith('.png')])

imgs = []
for i in range(0, min(len(frames), 180), 3):  # 每3帧取1张
    img = Image.open(os.path.join(frames_dir, frames[i]))
    img = img.resize((640, 360))
    imgs.append(img)
    if len(imgs) >= 60:
        break

imgs[0].save('/tmp/result.gif', save_all=True, append_images=imgs[1:],
              duration=67, loop=0)
```

---

## 容器环境

```bash
# 进入 Isaac Sim 容器
docker exec -it wheel-isaac-sim bash

# 容器内 Python 路径
/isaac-sim/python.sh

# 运行脚本
cd /workspace
/isaac-sim/python.sh scripts/trossen_render.py
```

---

## 文件结构

```
/workspace/trossen_ai_isaac/assets/robots/mobile_ai/
├── mobile_ai.usd          # 原始 USD（含 broken refs）
├── meshes/                # 网格文件
├── robots.yaml           # 配置文件
└── ...

/tmp/outputs/trossen/     # 渲染输出
├── rgb_0000.png
├── rgb_0001.png
├── ...
└── result.gif
```

---

## 常用脚本

| 脚本 | 功能 |
|------|------|
| `scripts/create_factory_simulation.py` | Phase 1 场景 |
| `scripts/render_moving_final.py` | 移动机器人渲染 |
| `/workspace/scenes/factory_with_rj2506_fixed.usd` | RJ2506 机器人场景 |

---

**每次说成功的时候，都要展示渲染结果图片给用户看！**
