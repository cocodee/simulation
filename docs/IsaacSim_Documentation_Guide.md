# IsaacSim 文档与示例指南

> 生成时间: 2026-04-21
> 文档路径: /root/data1/kdi/workspace/isaac/isaac-sim

---

## 目录

1. [文档 (Documentation)](#文档-documentation)
2. [示例 (Examples)](#示例-examples)
3. [扩展 (Extensions)](#扩展-extensions)

---

## 文档 (Documentation)

### 主要文档目录

```
/root/data1/kdi/workspace/isaac/isaac-sim/docs/py/
├── source/extensions/     # 扩展文档 (95个模块)
├── api/                   # API 参考文档
└── index.html            # 文档首页
```

### 文档覆盖的主要模块

| 模块路径 | 说明 |
|---------|------|
| `isaacsim.core.api` | 核心API |
| `isaacsim.core.nodes` | 节点系统 |
| `isaacsim.sensors.camera` | 相机传感器 |
| `isaacsim.sensors.physics` | 物理传感器 |
| `isaacsim.sensors.rtx` | RTX传感器 |
| `isaacsim.robot.manipulators` | 机械臂控制 |
| `isaacsim.robot.wheeled_robots` | 轮式机器人 |
| `isaacsim.cortex.framework` | Cortex框架 |
| `isaacsim.replicator.*` | Replicator工具 |
| `isaacsim.ros2.bridge` | ROS2桥接 |
| `isaacsim.asset.importer` | 资产导入 (URDF/MJCF) |
| `isaacsim.simulation_app` | 仿真应用 |

### 扩展文档详细列表

位于 `/root/data1/kdi/workspace/isaac/isaac-sim/docs/py/source/extensions/`:

```
isaacsim.app.about
isaacsim.app.selector
isaacsim.app.setup
isaacsim.asset.browser
isaacsim.asset.exporter.urdf
isaacsim.asset.gen.conveyor
isaacsim.asset.gen.omap
isaacsim.asset.importer.heightmap
isaacsim.asset.importer.mjcf
isaacsim.asset.importer.urdf
isaacsim.asset.validation
isaacsim.benchmark.examples
isaacsim.benchmark.services
isaacsim.code_editor.jupyter
isaacsim.code_editor.vscode
isaacsim.core.api
isaacsim.core.cloner
isaacsim.core.deprecation_manager
isaacsim.core.experimental.materials
isaacsim.core.experimental.objects
isaacsim.core.experimental.prims
isaacsim.core.experimental.utils
isaacsim.core.includes
isaacsim.core.nodes
isaacsim.core.prims
isaacsim.core.simulation_manager
isaacsim.core.throttling
isaacsim.core.utils
isaacsim.core.version
isaacsim.cortex.behaviors
isaacsim.cortex.framework
isaacsim.examples.browser
isaacsim.examples.extension
isaacsim.examples.interactive
isaacsim.examples.ui
isaacsim.gui.components
isaacsim.gui.content_browser
isaacsim.replicator.behavior
isaacsim.replicator.examples
isaacsim.replicator.grasping
isaacsim.replicator.writers
isaacsim.robot.schema
isaacsim.robot.surface_gripper
isaacsim.robot_motion.lula
isaacsim.robot_setup.grasp_editor
isaacsim.robot_setup.wizard
isaacsim.sensors.camera
isaacsim.sensors.physics
isaacsim.sensors.physics.examples
isaacsim.sensors.physx
isaacsim.sensors.physx.examples
isaacsim.sensors.rtx
isaacsim.storage.native
isaacsim.test.collection
isaacsim.test.docstring
isaacsim.test.utils
isaacsim.util.camera_inspector
isaacsim.util.merge_mesh
omni.isaac.dynamic_control
omni.kit.loop-isaac
omni.replicator.agent
omni.syntheticdata
```

---

## 示例 (Examples)

### 1. Standalone Examples 目录结构

```
/root/data1/kdi/workspace/isaac/isaac-sim/standalone_examples/
├── api/                    # API示例 (26个模块)
├── replicator/             # Replicator示例
├── testing/                # 测试示例
├── notebooks/              # Jupyter notebooks
├── tutorials/              # 教程
└── benchmarks/             # 基准测试
```

### 2. API 示例模块

| 模块路径 | 说明 |
|---------|------|
| `isaacsim.core.api` | 核心功能示例 |
| `isaacsim.sensors.camera` | 相机示例 |
| `isaacsim.sensors.physics` | 物理传感器示例 |
| `isaacsim.sensors.rtx` | RTX传感器示例 |
| `isaacsim.robot.manipulators` | 机械臂控制 |
| `isaacsim.robot.manipulators.examples.franka` | Franka 机械臂示例 |
| `isaacsim.robot.wheeled_robots.examples` | 轮式机器人 |
| `isaacsim.cortex.framework` | Cortex框架 |
| `isaacsim.replicator.examples` | Replicator示例 |
| `isaacsim.replicator.behavior` | Replicator行为 |
| `isaacsim.replicator.domain_randomization` | 域随机化 |
| `isaacsim.replicator.grasping` | 抓取示例 |
| `isaacsim.ros2.bridge` | ROS2桥接 |
| `isaacsim.asset.importer.urdf` | URDF导入 |
| `isaacsim.asset.importer.mjcf` | MJCF导入 |
| `isaacsim.util.debug_draw` | 调试绘图 |
| `isaacsim.simulation_app` | 仿真应用 |
| `isaacsim.core.cloner` | 克隆器 |
| `isaacsim.core.experimental` | 实验性功能 |
| `isaacsim.xr.openxr` | OpenXR 支持 |

### 3. Replicator 示例

```
standalone_examples/replicator/
├── object_based_sdg/       # 物体级场景生成
├── scene_based_sdg/        # 场景级生成
├── pose_generation/        # 姿态生成
├── mobility_gen/            # 移动性生成
├── online_generation/      # 在线生成
├── augmentation/            # 增强
└── infinigen/              # InfiniGen集成
```

### 4. Testing 模块

```
standalone_examples/testing/
├── isaacsim.core.api
├── isaacsim.simulation_app
├── isaacsim.cortex.framework
├── isaacsim.robot.manipulators.examples.franka
├── isaacsim.ros2.bridge
├── isaacsim.replicator.examples
├── isaacsim.sensors.physics
├── isaacsim.test.docstring
├── isaacsim.benchmark.services
├── omni.isaac.dynamic_control
├── omni.replicator.agent
├── omni.syntheticdata
├── python_sh
├── validation
└── notebooks/
```

### 5. Benchmarks 模块

```
standalone_examples/benchmarks/
└── validation/
```

---

## 扩展 (Extensions)

### 扩展目录

位于 `/root/data1/kdi/workspace/isaac/isaac-sim/exts/` 下，包含 **99个扩展模块**。

### 主要扩展列表

```
isaacsim.app.about
isaacsim.app.selector
isaacsim.app.setup
isaacsim.asset.browser
isaacsim.asset.exporter.urdf
isaacsim.asset.gen.conveyor
isaacsim.asset.gen.conveyor.ui
isaacsim.asset.gen.omap
isaacsim.asset.gen.omap.ui
isaacsim.asset.importer.heightmap
isaacsim.asset.importer.mjcf
isaacsim.asset.importer.urdf
isaacsim.asset.validation
isaacsim.benchmark.examples
isaacsim.benchmark.services
isaacsim.code_editor.jupyter
isaacsim.code_editor.vscode
isaacsim.core.api
isaacsim.core.cloner
isaacsim.core.deprecation_manager
isaacsim.core.experimental.materials
isaacsim.core.experimental.objects
isaacsim.core.experimental.prims
isaacsim.core.experimental.utils
isaacsim.core.includes
isaacsim.core.nodes
isaacsim.core.prims
isaacsim.core.simulation_manager
isaacsim.core.throttling
isaacsim.core.utils
isaacsim.core.version
isaacsim.cortex.behaviors
isaacsim.cortex.framework
isaacsim.examples.browser
isaacsim.examples.extension
isaacsim.examples.interactive
isaacsim.examples.ui
isaacsim.gui.content_browser
isaacsim.robot.schema
isaacsim.robot.surface_gripper
isaacsim.robot.surface_gripper.ui
isaacsim.robot.wheeled_robots
isaacsim.robot.wheeled_robots.ui
isaacsim.robot.manipulators.ui
isaacsim.robot.manipulators.examples.franka
isaacsim.replicator.behavior
isaacsim.replicator.examples
isaacsim.replicator.grasping
isaacsim.replicator.grasping.ui
isaacsim.replicator.writers
isaacsim.robot_setup.grasp_editor
isaacsim.robot_setup.wizard
isaacsim.robot_motion.lula
isaacsim.sensors.camera
isaacsim.sensors.camera.ui
isaacsim.sensors.physics
isaacsim.sensors.physics.ui
isaacsim.sensors.physics.examples
isaacsim.sensors.physx
isaacsim.sensors.physx.ui
isaacsim.sensors.physx.examples
isaacsim.sensors.rtx
isaacsim.sensors.rtx.ui
isaacsim.storage.native
isaacsim.test.collection
isaacsim.test.docstring
isaacsim.test.utils
isaacsim.util.camera_inspector
isaacsim.util.merge_mesh
```

---

## 常用路径速查

| 资源类型 | 路径 |
|---------|------|
| 主目录 | `/root/data1/kdi/workspace/isaac/isaac-sim/` |
| Python API文档 | `/root/data1/kdi/workspace/isaac/isaac-sim/docs/py/` |
| API示例 | `/root/data1/kdi/workspace/isaac/isaac-sim/standalone_examples/api/` |
| Replicator示例 | `/root/data1/kdi/workspace/isaac/isaac-sim/standalone_examples/replicator/` |
| 扩展源码 | `/root/data1/kdi/workspace/isaac/isaac-sim/exts/` |
| 扩展缓存 | `/root/data1/kdi/workspace/isaac/isaac-sim/extscache/` |

---

## 版本信息

- IsaacSim 版本: 5.1.0
- 平台: Linux x86_64
