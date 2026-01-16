# VLM_Module - 视觉语言模型模块

## 概述

VLM_Module 提供了**视觉语言模型 (Vision-Language Model)** 集成能力，使机器人能够理解和分析视觉信息。

**当前状态**: 🎯 **Demo阶段** - 颜色识别演示，将扩展为完整的环境感知模块

## Demo功能：颜色识别与动作映射

### 支持的颜色与动作

| 颜色 | 动作 |
|------|------|
| 🔴 红色 | 前进1米 |
| 🟠 橙色 | 前进1米 |
| 🟡 黄色 | 左转90度 |
| 🟢 绿色 | 后退1米 |
| 🔵 蓝色 | 右转90度 |
| 🟣 紫色 | 停止 |
| ⚫ 黑色 | 无动作 |

### 快速开始

```bash
# 启动系统
./start_robot_system.sh

# 输入指令
检测颜色并移动
```

## 文件结构

```
VLM_Module/
├── __init__.py           # 模块导出
├── vlm_core.py           # VLMCore 核心类
├── test_vlm.py           # 基础测试脚本
├── test_vlm_action.py    # 动作测试脚本
├── prompts/              # VLM 提示词模板
│   └── perceive_environment.yaml  # 颜色识别提示词
└── assets/               # 测试图片
    ├── red.png
    ├── green.png
    ├── blue.png
    ├── yellow.png
    └── ...
```

## 核心类: VLMCore

### 初始化

```python
from VLM_Module.vlm_core import VLMCore

# 使用默认配置 (本地 Ollama)
vlm = VLMCore()

# 自定义配置
vlm = VLMCore(
    model='qwen3-vl:4b',
    host='http://localhost:11434'
)
```

### 核心方法

#### `perceive(image_path=None)` - 颜色识别

识别图片中的方块颜色并返回对应动作。

```python
result = vlm.perceive('/path/to/image.png')

# 返回格式:
# {
#     'vlm_input': '检测到red色方块',
#     'action': 'move_forward'
# }

# 无有效颜色时返回 None
result = vlm.perceive('/path/to/black.png')
# 返回: None
```

## 系统集成

### 1. 作为 Robot_Module 工具

VLM已注册为机器人技能，LLM可直接调用：

```python
# 用户指令
"检测颜色并移动"
"根据图片颜色输出动作"

# LLM会自动调用 detect_color_and_act 工具
```

### 2. 支持自定义图片路径

```bash
# 指定图片路径
前进1米，然后根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色并执行动作

# LLM会正确提取路径并传入参数
```

### 3. 工具函数定义

```python
# Robot_Module/module/vision.py
async def detect_color_and_act(image_path: str = None) -> str:
    """检测图片颜色并执行相应动作

    重要：如果用户指令中提到了图片路径，必须将完整路径作为image_path参数传入！

    Args:
        image_path: 图片文件路径（可选）。如果用户提供了路径，必须使用该路径；否则使用默认图片。

    Returns:
        动作指令JSON字符串
    """
    # 识别颜色
    # 执行对应动作
```

## 提示词配置

`prompts/perceive_environment.yaml`:

```yaml
prompt: |
  请识别图片中方块的颜色。

  只返回以下JSON格式，不要其他内容：
  {
    "color": "颜色英文"
  }

  颜色映射：
  - 红色 → "red"
  - 橙色 → "orange"
  - 黄色 → "yellow"
  - 绿色 → "green"
  - 蓝色 → "blue"
  - 紫色 → "purple"
  - 黑色 → "black"
```

## 使用示例

### 基础颜色识别测试

```bash
cd /home/robot/work/FinalProject
python3 VLM_Module/test_vlm.py
```

### 动作执行测试

```bash
python3 VLM_Module/test_vlm_action.py
```

### 交互式测试

```bash
./start_robot_system.sh

# 测试默认图片
检测颜色并移动

# 测试指定图片（红色）
前进1米，然后根据 /home/robot/work/FinalProject/VLM_Module/assets/red.png 检测颜色并执行动作

# 测试指定图片（绿色）
检测 /home/robot/work/FinalProject/VLM_Module/assets/green.png 的颜色并移动
```

## 输入指令格式

### 格式1：使用默认图片
```
检测颜色并移动
根据图片颜色输出动作
```

### 格式2：指定图片路径
```
根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色并执行动作
前进1米，然后根据 /path/to/image.png 检测颜色
```

### 格式3：组合指令
```
前进1米，左转90度，再根据 /home/robot/work/FinalProject/VLM_Module/assets/blue.png 检测颜色并执行相应动作
```

## 依赖

```bash
# 核心依赖
pip install ollama pyyaml

# 本地 VLM 模型 (Ollama)
ollama pull qwen3-vl:4b
```

## 调试输出

VLM包含详细的调试信息：

```
[VLM] 原始响应: {"color": "red"}
[VLM] 识别颜色: red
[vision.detect_color_and_act] 检测到红色/橙色，执行前进
```

## 配置说明

### 修改默认图片

编辑 `vlm_core.py:10`:

```python
self.default_image = '/home/robot/work/FinalProject/VLM_Module/assets/green.png'
```

### 修改颜色映射

编辑 `vlm_core.py:46-54`:

```python
color_action_map = {
    'red': 'move_forward',
    'green': 'move_backward',
    # 添加自定义映射...
}
```

## 未来扩展计划

### Phase 1: 环境感知 (下一步)

- 场景分析 (室内/室外/房间类型)
- 物体检测 (识别关键物体及位置)
- 障碍物识别 (类型、位置、严重程度)
- 可通行区域分析

### Phase 2: 高级功能

- 多物体跟踪
- 3D 空间理解
- 动态场景分析
- 语义分割

### Phase 3: 决策增强

- 与 LLM 深度集成
- 视觉导航规划
- 复杂场景理解
- 多模态推理

## 数据流设计

```
┌──────────────┐
│ 摄像头/图片  │
└──────┬───────┘
       ↓
┌─────────────────────────────────────┐
│ VLM_Module.vlm_core.perceive()      │
│                                     │
│  1. 加载图片                        │
│  2. 构建提示词                      │
│  3. 调用 Ollama VLM API             │
│  4. 解析 JSON 响应                  │
│  5. 映射颜色 → 动作                 │
└──────────┬──────────────────────────┘
           ↓
    {'color': 'red', 'action': 'move_forward'}
           ↓
┌─────────────────────────────────────┐
│ Robot_Module.detect_color_and_act() │
│                                     │
│  1. 接收动作指令                    │
│  2. 发送到仿真器/真实机器人          │
└─────────────────────────────────────┘
```

## 相关文档

- [主项目 README](../README.md)
- [LLM_Module README](../LLM_Module/README.md)
- [Robot_Module README](../Robot_Module/README.md)

---

**视觉感知，智能决策！** 🚀
