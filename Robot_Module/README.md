# Interactive_Module - 交互界面模块

用户交互界面的核心模块，提供 CLI 命令行交互功能，协调 LLM 和 Robot_Module。

## 📁 模块结构

```
Interactive_Module/
├── README.md              # 本文档
└── interactive.py         # CLI 交互主程序
```

## 🎯 功能特性

- **自然语言交互**: 支持中文指令输入

====================================
Robot_Module 模块详细文档
====================================

# Robot_Module - MCP 工具注册中心

基于 FastMCP 的机器人工具注册中心，提供底盘控制、追击功能、视觉检测等模块化工具。

## 📁 模块结构

```
Robot_Module/
├── README.md              # 本文档
├── skill.py               # FastMCP 服务器入口
└── module/                # 功能模块
    ├── __init__.py
    ├── base.py            # 底盘控制 (move/turn/stop)
    ├── chase.py           # 追击功能 (chase_enemy)
    └── vision.py          # 视觉检测 (detect_color)
```

## 🎯 功能特性

- **MCP 标准化**: 基于 FastMCP 的工具注册
- **模块化设计**: 每个功能独立模块
- **懒加载**: ROS2/VLM 延迟初始化
- **自动文档**: 从 docstring 生成工具描述
- **类型安全**: 支持类型提示和参数验证
- **错误处理**: 完善的异常处理机制

## 🔧 核心组件

### skill.py - FastMCP 服务器入口

**主要功能：**

```python
from fastmcp import FastMCP

# 创建 MCP 服务器
mcp = FastMCP("robot-skills")

# 注册所有模块
def register_all_modules():
    from module.base import register_tools as register_base
    from module.chase import register_tools as register_chase
    from module.vision import register_tools as register_vision

    register_base(mcp)
    register_chase(mcp)
    register_vision(mcp)

    print(f"[Robot_Module] 已注册 {len(_tool_registry)} 个工具")
```

**工具注册流程：**

1. 创建 FastMCP 服务器实例
2. 导入各模块的 `register_tools()` 函数
3. 调用 `register_tools(mcp)` 注册工具
4. 使用 `@mcp.tool()` 装饰器标记工具函数
5. 从 docstring 自动生成工具描述

### module/base.py - 底盘控制

**可用工具：**

| 工具名称 | 功能 | 参数 |
|---------|------|------|
| `move_forward` | 前进 | distance (米), speed (m/s) |
| `move_backward` | 后退 | distance (米), speed (m/s) |
| `turn` | 旋转 | angle (度), angular_speed (rad/s) |
| `stop` | 停止 | 无 |

**使用示例：**

```python
# 前进 1.5 米，速度 0.3 m/s
await move_forward(distance=1.5, speed=0.3)

# 后退 0.5 米，速度 0.2 m/s
await move_backward(distance=0.5, speed=0.2)

# 左转 90 度，角速度 0.5 rad/s
await turn(angle=-90.0, angular_speed=0.5)

# 停止运动
await stop()
```

**内部实现：**

```python
@mcp.tool()
async def move_forward(distance: float = 1.0, speed: float = 0.3) -> str:
    """前进指定距离

    Args:
        distance: 前进距离（米），默认 1.0
        speed: 前进速度（m/s），默认 0.3

    Returns:
        执行结果 JSON 字符串
    """
    action = {
        'action': 'move_forward',
        'parameters': {'distance': distance, 'speed': speed}
    }
    _get_action_queue().put(action)
    return json.dumps({"success": True, "action": action})
```

### module/chase.py - 追击功能

**可用工具：**

| 工具名称 | 功能 | 参数 |
|---------|------|------|
| `chase_enemy` | 追击最近的敌人 | 无 |
| `get_enemy_positions` | 获取所有敌人位置 | 无 |
| `chase_target` | 追击指定坐标 | target_x, target_y, threshold, step_distance |
| `chase_nearest_enemy` | 从列表追击最近 | enemy_positions (JSON) |
| `calculate_chase_angle` | 计算追击角度 | robot_x, robot_y, robot_angle, target_x, target_y |

**使用示例：**

```python
# 追击最近的敌人
await chase_enemy()

# 追击指定坐标
await chase_target(target_x=700, target_y=300)

# 获取敌人位置
positions = await get_enemy_positions()
# [{"id": "1", "x": 600, "y": 200}, ...]

# 从列表追击最近
await chase_nearest_enemy('[{"id": "1", "x": 600, "y": 200}]')

# 计算追击角度
result = await calculate_chase_angle(
    robot_x=400, robot_y=300, robot_angle=0,
    target_x=600, target_y=200
)
```

**追击参数配置：**

```python
# 全局配置（chase.py 顶部）
MAX_STEP_DISTANCE = 0.5  # 最大步长（米）
ARRIVAL_THRESHOLD = 5.0  # 到达阈值（像素）
```

**追击流程：**

```
1. 获取敌人位置（重试 10 次，每次 300ms）
2. 获取机器人位置
3. 找到最近的敌人
4. 计算目标角度和角度差
5. 旋转到目标方向（角度差 > 5° 时）
6. 循环前进直到到达（PID 控制步长）
7. 清除敌人
```

### module/vision.py - 视觉检测

**可用工具：**

| 工具名称 | 功能 | 参数 |
|---------|------|------|
| `detect_color_and_act` | 检测颜色并执行动作 | image_path |

**使用示例：**

```python
# 检测图片中的颜色并执行相应动作
result = await detect_color_and_act(image_path="/home/robot/image.png")

# 返回示例
{
  "success": True,
  "detected_color": "红色",
  "action": "turn",
  "action_parameters": {"angle": -90.0}
}
```

**颜色动作映射：**

```python
COLOR_ACTION_MAP = {
    "红色": {"action": "turn", "parameters": {"angle": -90.0}},  # 左转
    "蓝色": {"action": "turn", "parameters": {"angle": 90.0}},   # 右转
    "绿色": {"action": "move_forward", "parameters": {"distance": 0.5}},
    "黄色": {"action": "stop", "parameters": {}},
}
```

## 📝 添加新工具

### 步骤 1: 创建模块文件

在 `Robot_Module/module/` 下创建新文件：

```bash
cd Robot_Module/module
cp base.py your_module.py
```

### 步骤 2: 编写工具函数

```python
# your_module.py
import json
import sys

# 全局变量（懒加载）
_action_queue = None

def _get_action_queue():
    """获取动作队列（懒加载）"""
    global _action_queue
    if _action_queue is None:
        from pathlib import Path
        project_root = Path(__file__).parent.parent.parent
        sys.path.insert(0, str(project_root))
        from ros_topic_comm import get_shared_queue
        _action_queue = get_shared_queue()
    return _action_queue

def register_tools(mcp):
    """注册工具到 MCP 服务器"""

    @mcp.tool()
    async def your_tool(param1: float, param2: str = "default") -> str:
        """工具描述（会自动生成文档）

        Args:
            param1: 参数1描述
            param2: 参数2描述，默认值 "default"

        Returns:
            执行结果 JSON 字符串

        Examples:
            your_tool(param1=1.0, param2="test")
        """
        # 实现逻辑
        action = {
            'action': 'your_action',
            'parameters': {'param1': param1, 'param2': param2}
        }
        _get_action_queue().put(action)

        return json.dumps({
            "success": True,
            "action": action
        }, ensure_ascii=False)

    print(f"[your_module] 已注册 1 个工具")

    return {
        'your_tool': your_tool
    }
```

### 步骤 3: 注册到 skill.py

```python
# skill.py
def register_all_modules():
    # ... 现有模块 ...

    # 注册新模块
    from module.your_module import register_tools as register_your_module
    register_your_module(mcp)
```

### 步骤 4: 测试工具

```bash
# 启动交互程序
python3 Interactive_Module/interactive.py

# 输入命令测试
your_tool(param1=1.0, param2="test")
```

## 🔌 MCP 工具规范

### 工具函数签名

```python
@mcp.tool()
async def tool_name(param1: type, param2: type = default) -> str:
    """工具描述（一行）

    详细描述（多行可选）

    Args:
        param1: 参数1描述
        param2: 参数2描述

    Returns:
        返回值描述

    Examples:
        tool_name(param1=1.0, param2="test")
    """
    # 实现
    pass
```

### 返回值格式

```json
{
  "success": true/false,
  "message": "执行结果描述",
  "data": {}
}
```

### 错误处理

```python
try:
    # 执行逻辑
    result = do_something()
    return json.dumps({"success": True, "data": result})
except Exception as e:
    return json.dumps({
        "success": False,
        "error": str(e)
    }, ensure_ascii=False)
```

## 🔌 ROS2 通讯

### 话题列表

| 话题名称 | 消息类型 | 用途 |
|---------|---------|------|
| `/robot/command` | String | 动作命令 |

### 消息格式

```json
{
  "action": "move_forward",
  "parameters": {
    "distance": 1.0,
    "speed": 0.3
  }
}
```

### 发送命令

```python
from ros_topic_comm import get_shared_queue

queue = get_shared_queue()
queue.put({
    'action': 'move_forward',
    'parameters': {'distance': 1.0, 'speed': 0.3}
})
```

## 🐛 调试技巧

### 1. 查看已注册工具

```python
# 在 skill.py 中
print(f"已注册工具: {list(_tool_registry.keys())}")
```

### 2. 查看工具元数据

```python
for name, metadata in _tool_metadata.items():
    print(f"{name}: {metadata['description']}")
```

### 3. 测试单个工具

```python
# 直接调用
from module.base import move_forward
result = await move_forward(distance=1.0, speed=0.3)
print(result)
```

### 4. 查看 ROS2 消息

```bash
ros2 topic echo /robot/command
```

## 🔗 相关模块

- `ros_topic_comm.py` - ROS2 通讯模块
- `Test_Module/chase_core.py` - 追击算法
- `VLM_Module/vlm_core.py` - 视觉语言模型

## 📝 依赖

```
fastmcp>=0.1.0  # MCP 服务器框架
rclpy           # ROS2 Python 客户端库
```

---

**模块化，易扩展！** 🛠️








