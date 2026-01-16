# Robot_Module - MCP 工具注册中心

## 概述

Robot_Module 是机器人技能的**MCP (Model Context Protocol) 工具注册中心**，负责管理所有机器人技能函数，并提供统一的调用接口。

### 核心功能

- **模块化工具注册**: 基于 FastMCP 的工具注册框架
- **自动元数据提取**: 从函数签名和 docstring 自动提取工具定义
- **进程间通信**: 通过 multiprocessing.Queue 与仿真器通信
- **易于扩展**: 添加新工具只需编写函数并注册

## 文件结构

```
Robot_Module/
├── skill.py              # FastMCP 服务器入口
├── module/               # 功能模块目录
│   ├── __init__.py
│   ├── base.py           # 底盘控制模块（移动、旋转、停止）
│   └── example.py        # 示例模块（模板）
└── README.md
```

## 核心文件说明

### 1. `skill.py` - MCP 服务器入口

**职责**:
- 初始化 FastMCP 服务器
- 注册所有功能模块的工具函数
- 管理全局工具注册表（`_tool_registry` 和 `_tool_metadata`）
- 提供动作队列管理

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
    # 1. 注册底盘控制模块
    register_base_tools(mcp, _tool_registry, _tool_metadata)

    # 2. 注册示例模块
    register_example_tools(mcp, _tool_registry, _tool_metadata)
```

### 2. `module/base.py` - 底盘控制模块

**已实现的工具**:

| 工具名 | 描述 | 参数 |
|--------|------|------|
| `move_forward` | 向前移动 | `distance: float` (距离), `speed: float` (速度) |
| `move_backward` | 向后移动 | `distance: float`, `speed: float` |
| `turn` | 原地旋转 | `angle: float` (角度), `angular_speed: float` (角速度) |
| `stop` | 紧急停止 | 无参数 |

**工具函数格式**:
```python
async def move_forward(distance: float = 1.0, speed: float = 0.3) -> str:
    """向前移动指定距离

    Args:
        distance: 移动距离（米），默认1.0米
        speed: 移动速度（米/秒），默认0.3米/秒

    Returns:
        动作指令JSON字符串
    """
    action = {
        'action': 'move_forward',
        'parameters': {'distance': distance, 'speed': speed}
    }

    # 发送到仿真器
    if _action_queue:
        _action_queue.put(action)

    return json.dumps(action, ensure_ascii=False)
```

**注册函数**:
```python
def register_tools(mcp, tool_registry=None, tool_metadata=None):
    """注册底盘控制模块的工具函数"""
    tools = [move_forward, move_backward, turn, stop]

    for func in tools:
        # 注册到 FastMCP
        mcp.tool()(func)

        # 提取并存储元数据（用于 LLM function calling）
        if tool_registry is not None and tool_metadata is not None:
            name, metadata = _extract_tool_metadata(func)
            tool_registry[name] = func
            tool_metadata[name] = metadata
```

### 3. `module/example.py` - 示例模块

**用途**: 添加新功能的参考模板

```python
async def example_tool(param1: str, param2: float = 10.0) -> str:
    """示例工具函数

    这是一个模板函数，展示如何定义新的工具函数。

    Args:
        param1: 第一个参数（字符串）
        param2: 第二个参数（数字，可选）

    Returns:
        操作结果JSON字符串
    """
    result = {
        'status': 'success',
        'message': f'执行完成: {param1}, {param2}'
    }
    return json.dumps(result, ensure_ascii=False)
```

## 数据流

```
LLM_Module (下层LLM)
    ↓ 工具调用请求
    (function_name="move_forward", arguments={"distance": 1.0})
    ↓
Robot_Module.skill.py
    ↓ 查找 _tool_registry
    ↓ 调用 move_forward(**arguments)
    ↓
module/base.py.move_forward()
    ↓ 构造动作指令
    {"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
    ↓
_action_queue.put(action)
    ↓
multiprocessing.Queue
    ↓
Sim_Module (仿真器)
    ↓ 执行动作并可视化
```

## 添加新工具模块

### 步骤 1: 创建模块文件

```bash
cd Robot_Module/module
cp example.py your_module.py
```

### 步骤 2: 编辑工具函数

```python
"""
你的模块 (Your Module Name)

负责某个具体功能。

Functions:
    - your_tool: 你的工具函数
"""

import sys
import json
import inspect

# 全局动作队列（用于与仿真器通信）
_action_queue = None


def set_action_queue(queue=None):
    """设置全局动作队列"""
    global _action_queue
    # ... 队列设置逻辑


# =============================================================================
# 工具函数实现
# =============================================================================

async def your_tool(param1: str, param2: float = 10.0) -> str:
    """你的工具函数描述

    详细说明工具的功能和使用场景。

    Args:
        param1: 参数1描述
        param2: 参数2描述（可选）

    Returns:
        动作指令JSON字符串
    """
    print(f"[your_module.your_tool] 执行: param1={param1}, param2={param2}", file=sys.stderr)

    action = {
        'action': 'your_action',
        'parameters': {'param1': param1, 'param2': param2}
    }

    # 发送到仿真器（如果需要）
    if _action_queue:
        _action_queue.put(action)

    return json.dumps(action, ensure_ascii=False)


# =============================================================================
# MCP 注册函数
# ==============================================================================

def _extract_tool_metadata(func):
    """从函数提取工具元数据（OpenAI function calling 格式）"""
    # ... 元数据提取逻辑


def register_tools(mcp, tool_registry=None, tool_metadata=None):
    """注册你的模块的工具函数到 MCP 服务器

    Args:
        mcp: FastMCP 服务器实例
        tool_registry: 工具函数注册表（可选）
        tool_metadata: 工具元数据注册表（可选）
    """
    # 要注册的工具函数列表
    tools = [your_tool]

    for func in tools:
        # 注册到 FastMCP
        mcp.tool()(func)

        # 提取并存储元数据（用于 LLM function calling）
        if tool_registry is not None and tool_metadata is not None:
            name, metadata = _extract_tool_metadata(func)
            tool_registry[name] = func
            tool_metadata[name] = metadata

    print(f"[your_module.py] 你的模块已注册 ({len(tools)} 个工具)", file=sys.stderr)
```

### 步骤 3: 在 skill.py 中注册

编辑 `skill.py`:

```python
# 1. 在导入部分添加
from module.your_module import register_tools as register_your_tools

# 2. 在 register_all_modules() 函数中添加
def register_all_modules():
    register_base_tools(mcp, _tool_registry, _tool_metadata)
    register_your_tools(mcp, _tool_registry, _tool_metadata)  # ← 添加这行
```

### 步骤 4: 测试新工具

```bash
# 重启交互界面
python3 Interactive_Module/interactive.py

# 查看是否显示新工具
# 可用工具: N 个
#   • your_tool(...)
```

## 元数据自动提取

`_extract_tool_metadata()` 函数自动从函数签名和 docstring 提取工具定义：

**输入**:
```python
async def move_forward(distance: float = 1.0, speed: float = 0.3) -> str:
    """向前移动指定距离

    Args:
        distance: 移动距离（米），默认1.0米
        speed: 移动速度（米/秒），默认0.3米/秒
    """
    pass
```

**输出** (OpenAI function calling 格式):
```json
{
  "type": "function",
  "function": {
    "name": "move_forward",
    "description": "向前移动指定距离",
    "parameters": {
      "type": "object",
      "properties": {
        "distance": {
          "type": "number",
          "description": "移动距离（米），默认1.0米"
        },
        "speed": {
          "type": "number",
          "description": "移动速度（米/秒），默认0.3米/秒"
        }
      },
      "required": []
    }
  }
}
```

## 通信机制

### 与仿真器通信

Robot_Module 通过 `multiprocessing.Queue` 与 Sim_Module 通信：

```python
# 1. 初始化共享队列
from shared_queue import get_shared_queue
_action_queue = get_shared_queue()

# 2. 发送动作指令
action = {'action': 'move_forward', 'parameters': {...}}
_action_queue.put(action)

# 3. 仿真器接收指令
action = action_queue.get()
# 执行动作...
```

### 文件队列实现

使用 `shared_queue.py` 实现跨进程通信：

```python
# 文件: /tmp/robot_finalproject/commands.jsonl
{"action": "move_forward", "parameters": {"distance": 1.0, "speed": 0.3}}
{"action": "turn", "parameters": {"angle": 90.0, "angular_speed": 0.5}}
```

## 依赖

```
fastmcp>=0.1.0    # MCP 服务器框架
```

## 设计特点

1. **模块化**: 每个功能模块独立文件，职责单一
2. **自动化**: 元数据自动提取，无需手动定义
3. **标准化**: 基于 FastMCP 的工具注册标准
4. **易扩展**: 添加新工具只需 3 步
5. **类型安全**: 函数签名和 docstring 提供完整类型信息

## 相关文档

- [主项目 README](../README.md)
- [Interactive_Module README](../Interactive_Module/README.md)
- [LLM_Module README](../LLM_Module/README.md)
- [Sim_Module README](../Sim_Module/README.md)

## 示例：完整的工具添加流程

假设我们要添加一个"播放声音"的工具：

1. **创建 `module/sound.py`**:
```python
async def play_sound(sound_name: str, volume: float = 0.8) -> str:
    """播放指定声音

    Args:
        sound_name: 声音文件名
        volume: 音量（0-1），默认0.8

    Returns:
        播放结果JSON字符串
    """
    action = {'action': 'play_sound', 'parameters': {'sound_name': sound_name, 'volume': volume}}
    return json.dumps(action, ensure_ascii=False)

def register_tools(mcp, tool_registry=None, tool_metadata=None):
    tools = [play_sound]
    # ... 注册逻辑
```

2. **在 `skill.py` 中注册**:
```python
from module.sound import register_tools as register_sound_tools

def register_all_modules():
    register_base_tools(mcp, _tool_registry, _tool_metadata)
    register_sound_tools(mcp, _tool_registry, _tool_metadata)
```

3. **测试**:
```bash
# 重启系统
./start_robot_system.sh

# 输入指令
💬 请输入指令: 播放提示音
# LLM 会自动调用 play_sound 工具
```

---

**模块化，易扩展！** 🚀
