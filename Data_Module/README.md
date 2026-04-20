# Data_Module

数据感知层。组合 VLM 视觉语义、物体事实、参数计算，为规划器提供结构化输入。

## 核心接口

```python
from Data_Module import build_context, VLMCore, load_object_facts, ParameterCalculator

# 构建完整上下文
snapshot = build_context(user_intent="导航到6,0,0")

# 单独使用 VLM
vlm = VLMCore()
description = vlm.describe_structured("image.jpg")

# 加载物体事实
facts = load_object_facts("config/object_facts.json")

# 参数计算
calculator = ParameterCalculator()
annotated = calculator.annotate_tasks(tasks, object_facts)
```

## 目录结构

```text
Data_Module/
  __init__.py       对外暴露 VLMCore, load_object_facts, ParameterCalculator 等
  vlm.py            VLMCore 视觉感知核心
  image_source.py   图像来源管理
  vlm_utils.py      视觉工具函数
  facts.py          object_facts 加载与标准化
  params.py         ParameterCalculator 参数计算器
  context.py        build_context() + build_planner_context()
  schema.py         RobotStateSnapshot dataclass
  prompts/
    VlmPrompt.yaml  VLM 提示词
  assets/
    0-4.png         VLM 参考图片
```

## 各文件职责

### `vlm.py` — VLMCore

视觉语言模型核心类：

- `describe_structured(image_path)` — 调用 VLM API 获取结构化视觉描述
- `build_scene_facts(visual_context)` — 从视觉描述中提取场景事实
- `merge_scene_facts(scene_facts, object_facts)` — 合并视觉事实和物体事实
- 使用 `Data_Module/prompts/VlmPrompt.yaml` 中的提示词

### `image_source.py` — 图像来源管理

支持三种图像来源（通过环境变量选择）：

- 仿真截图（默认）— 从 `/tmp/envtest_front_camera.png` 读取
- USB 摄像头 — `FINALPROJECT_VLM_IMAGE_PATH` 指定设备路径
- ROS2 topic — Go2 腕部摄像头 `/go2/wrist_camera/image_raw`（需 `FINALPROJECT_ROBOT_TYPE=go2`）

### `facts.py` — 物体事实管理

- `load_object_facts(path)` — 从 JSON 文件加载并标准化物体事实
- `normalize_object_facts(payload)` — 标准化：验证 objects 数组、constraints、navigation_goal、robot_pose
- 内部辅助：`_normalize_vec3()`、`_normalize_bool()`、`_normalize_object()`

### `params.py` — ParameterCalculator

参数计算器，根据 object_facts 为规划任务填充具体参数：

- `annotate_tasks(tasks, object_facts)` — 遍历任务列表，为每个任务计算 `calculated_parameters`
- 自动推断：推箱子目标位置、攀爬对正点、导航速度等
- 依赖 `object_facts` 中的物体几何信息（center, size）和约束（max_climb_height_m）

### `context.py` — 上下文构建

两个主要函数：

- `build_context(user_intent, object_facts_path)` — 构建完整 `RobotStateSnapshot`
  1. 调用 `Hardware_Module.get_state()` 获取硬件状态
  2. 调用 `sync_object_facts_from_live_data()` 同步实时数据
  3. 加载 object_facts
  4. 合并 scene_facts
- `build_planner_context(scene_facts, object_facts, synced_payload)` — 构建传给 Planner 的字典
  - 组装 `robot_state`、`constraints`、`objects`、`scene_facts`

### `schema.py` — RobotStateSnapshot

```python
@dataclass
class RobotStateSnapshot:
    state: dict[str, Any]         # 硬件状态
    visual_context: dict | None   # 视觉描述
    scene_facts: dict | None      # 场景事实
    object_facts: dict | None     # 物体事实
    connected: bool               # 是否已连接
    task_type: str | None         # 后端类型
```

## 与其他模块的关系

- 调用 `Hardware_Module.get_state()` 获取硬件状态
- 调用 `Hardware_Module.registry` 同步实时数据
- 被 `Robot_Module/vision/vlm_observe.py` 调用（VLMCore）
- 被 `Robot_Module/tools.py` 调用（load_object_facts, ParameterCalculator, build_planner_context）
- 被 `Planner_Module/planner.py` 调用（通过依赖注入使用 ParameterCalculator）
- 不导入 `Planner_Module`、`Excu_Module`、`Robot_Module`
