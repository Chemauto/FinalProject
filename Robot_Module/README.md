# Robot_Module - MCP 工具注册中心

## 概述

Robot_Module 是机器人技能的**MCP (Model Context Protocol) 工具注册中心**，负责管理所有机器人技能函数，并提供统一的调用接口。

### 核心功能

- **模块化工具注册**: 基于 FastMCP 的工具注册框架
- **装饰器风格**: 使用 `@mcp.tool()` 装饰器标记工具函数
- **自动元数据提取**: 从函数签名和 docstring 自动提取工具定义
- **ROS2 通讯**: 通过 ROS2 话题与仿真器通信
- **易于扩展**: 添加新工具只需编写函数并注册

## 文件结构

```
Robot_Module/
├── skill.py              # FastMCP 服务器入口
├── module/               # 功能模块目录
│   ├── __init__.py
│   ├── base.py           # 底盘控制模块（移动、旋转、停止）
│   ├── vision.py         # 视觉感知模块（VLM颜色识别）
│   └── example.py        # 示例模块（模板）
└── README.md
```

## 已注册的工具

| 模块 | 工具数 | 工具列表 |
|------|--------|----------|
| base.py | 4 | `move_forward`, `move_backward`, `turn`, `stop` |
| vision.py | 1 | `detect_color_and_act` |
| **总计** | **5** |  |

## 核心文件说明

### 1. `skill.py` - MCP 服务器入口

**职责**:
- 初始化 FastMCP 服务器
- 注册所有功能模块的工具函数
- 管理全局工具注册表
- 提供工具定义获取接口

**主要组件**:

```python
# FastMCP 服务器实例
mcp = FastMCP("robot")

# 工具注册表
_tool_registry = {}   # 函数名 -> 函数对象
_tool_metadata = {}   # 函数名 -> 元数据（OpenAI function calling 格式）

# 工具定义获取
get_tool_definitions() -> List[Dict]  # 返回 LLM 可用的工具定义
```

**注册流程**:
```python
def register_all_modules():
    """注册所有功能模块到 MCP 服务器"""
    # 1. 注册底盘控制模块（返回工具字典）
    base_tools = register_base_tools(mcp)
    _tool_registry.update(base_tools)

    # 2. 注册视觉感知模块（返回工具字典）
    vision_tools = register_vision_tools(mcp)
    _tool_registry.update(vision_tools)
```

### 2. `module/base.py` - 底盘控制模块

**已实现的工具**:

| 工具名 | 描述 | 参数 |
|--------|------|------|
| `move_forward` | 向前移动 | `distance: float` (距离), `speed: float` (速度) |
| `move_backward` | 向后移动 | `distance: float`, `speed: float` |
| `turn` | 原地旋转 | `angle: float` (角度), `angular_speed: float` (角速度) |
| `stop` | 紧急停止 | 无参数 |

**代码风格**:
```python
def register_tools(mcp):
    """注册底盘控制工具函数"""

    @mcp.tool()
    async def move_forward(distance: float = 1.0, speed: float = 0.3) -> str:
        """向前移动指定距离

        机器人沿当前朝向向前移动。

        Args:
            distance: 移动距离（米），默认1.0米
            speed: 移动速度（米/秒），默认0.3米/秒

        Returns:
            动作指令JSON字符串

        Examples:
            move_forward(distance=2.0, speed=0.5)  # 前进2米，速度0.5m/s
        """
        # 发送到仿真器
        _get_action_queue().put(action)
        return json.dumps(action, ensure_ascii=False)

    # 返回工具函数字典
    return {
        'move_forward': move_forward,
        'move_backward': move_backward,
        'turn': turn,
        'stop': stop
    }
```

### 3. `module/vision.py` - 视觉感知模块

**工具功能**:

| 工具名 | 描述 | 功能 |
|--------|------|------|
| `detect_color_and_act` | 检测颜色并执行动作 | 识别图片中方块颜色，执行对应动作 |

**颜色-动作映射**:

| 颜色 | 动作 |
|------|------|
| 🔴 红色 | 前进1米 |
| 🟠 橙色 | 前进1米 |
| 🟡 黄色 | 左转90度 |
| 🟢 绿色 | 后退1米 |
| 🔵 蓝色 | 右转90度 |
| 🟣 紫色 | 停止 |
| ⚫ 黑色 | 无动作 |

**代码示例**:
```python
def register_tools(mcp):
    """注册视觉感知工具函数"""

    @mcp.tool()
    async def detect_color_and_act(image_path: str = None) -> str:
        """检测图片颜色并执行相应动作

        重要：如果用户指令中提到了图片路径，必须将完整路径作为image_path参数传入！

        Args:
            image_path: 图片文件路径（可选）。如果用户提供了路径，必须使用该路径；否则使用默认图片。

        Returns:
            动作指令JSON字符串
        """
        # VLM识别颜色
        # 执行对应动作
        # 发送到仿真器

    return {
        'detect_color_and_act': detect_color_and_act
    }
```

### 4. `module/example.py` - 示例模块

**用途**: 添加新功能的参考模板

**模板结构**:
```python
def register_tools(mcp):
    """注册示例模块的工具函数"""

    @mcp.tool()
    async def example_tool(param1: str, param2: float = 10.0) -> str:
        """示例工具函数

        这是一个模板函数，展示如何定义新的工具函数。

        Args:
            param1: 第一个参数（字符串）
            param2: 第二个参数（数字，可选）

        Returns:
            操作结果JSON字符串
        """
        # 实现工具逻辑
        pass

    return {
        'example_tool': example_tool
    }
```

## 数据流

```
用户输入
    ↓
┌─────────────────────────────────────────┐
│ LLM_Module (下层LLM)                    │
│ 工具调用请求                             │
│ (function_name="move_forward",          │
│  arguments={"distance": 1.0})            │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│ Robot_Module.skill.py                   │
│ get_skill_function(function_name)       │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│ module/base.py.move_forward()           │
│ 构造动作指令                            │
│ {"action": "move_forward", ...}         │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│ _action_queue.put(action)               │
│ ROS2 Topic (/robot/command)             │
└──────────────┬──────────────────────────┘
               ↓
┌─────────────────────────────────────────┐
│ Sim_Module (仿真器)                      │
│ 订阅消息并执行动作                       │
└─────────────────────────────────────────┘
```

## 添加新工具模块

### 步骤 1: 创建模块文件

```bash
cd Robot_Module/module
cp example.py your_module.py
```

### 步骤 2: 编辑工具函数

```python
"""你的模块 (Your Module Name)

负责某个具体功能。

Functions:
    - your_tool: 你的工具函数
"""

import sys
import json

# 全局动作队列
_action_queue = None


def _get_action_queue():
    """获取动作队列"""
    global _action_queue
    if _action_queue is None:
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_shared_queue
        _action_queue = get_shared_queue()
    return _action_queue


def register_tools(mcp):
    """注册你的模块的工具函数

    Args:
        mcp: FastMCP 服务器实例

    Returns:
        工具函数字典 {name: function}
    """

    @mcp.tool()
    async def your_tool(param1: str, param2: float = 10.0) -> str:
        """你的工具函数

        详细说明工具的功能和使用场景。

        Args:
            param1: 参数1描述
            param2: 参数2描述（可选）

        Returns:
            操作结果JSON字符串

        Examples:
            your_tool(param1="test", param2=5.0)
        """
        action = {
            'action': 'your_action',
            'parameters': {'param1': param1, 'param2': param2}
        }

        _get_action_queue().put(action)
        return json.dumps(action, ensure_ascii=False)

    return {
        'your_tool': your_tool
    }
```

### 步骤 3: 在 skill.py 中注册

```python
# 1. 在导入部分添加
from module.your_module import register_tools as register_your_tools

# 2. 在 register_all_modules() 函数中添加
def register_all_modules():
    register_base_tools(mcp)
    register_your_tools(mcp)  # ← 添加这行
```

## 通信机制

### ROS2 话题通讯

- **话题名称**: `/robot/command`
- **消息类型**: `std_msgs/String`
- **消息格式**: JSON 字符串

```python
# 消息示例
{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
{"action": "turn", "parameters": {"angle": 90.0, "angular_speed": 0.5}}
```

### 调试命令

```bash
# 查看话题列表
ros2 topic list

# 查看话题消息
ros2 topic echo /robot/command

# 查看话题信息
ros2 topic info /robot/command
```

## 设计特点

1. **装饰器风格**: 使用 `@mcp.tool()` 直接标记工具函数
2. **函数内部定义**: 工具函数定义在 `register_tools(mcp)` 内部
3. **返回工具字典**: `register_tools` 返回 `{name: function}` 字典
4. **自动注册**: skill.py 自动合并所有工具到注册表
5. **统一代码风格**: 所有模块保持相同的代码结构

## 相关文档

- [主项目 README](../README.md)
- [Interactive_Module README](../Interactive_Module/README.md)
- [LLM_Module README](../LLM_Module/README.md)
- [Sim_Module README](../Sim_Module/README.md)
- [VLM_Module README](../VLM_Module/README.md)

---

**模块化，易扩展！** 🚀
