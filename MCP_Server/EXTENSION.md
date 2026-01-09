# MCP Robot Control Server - 扩展开发指南

本文档详细说明如何扩展和修改MCP Robot Control Server，包括添加新技能、新适配器等。

## 📋 目录

- [扩展流程概览](#扩展流程概览)
- [场景1: 添加新的Robot Skill](#场景1-添加新的robot-skill)
- [场景2: 修改现有Skill](#场景2-修改现有skill)
- [场景3: 添加新适配器](#场景3-添加新适配器)
- [场景4: 添加新的通信架构](#场景4-添加新的通信架构)
- [场景5: 自定义任务规划逻辑](#场景5-自定义任务规划逻辑)
- [场景6: 调整执行参数](#场景6-调整执行参数)
- [检查清单](#检查清单)
- [常见问题](#常见问题)

---

## 🎯 扩展流程概览

```
确定扩展类型
    ↓
┌─────────────────────────────────────┐
│ 需要修改哪些文件？                    │
├─────────────────────────────────────┤
│                                     │
│ ┌─────────────┐  ┌──────────────┐  │
│ │Robot Skills │  │  Adapters    │  │
│ │             │  │              │  │
│ │ - MCP_TOOLS │  │ - 新适配器   │  │
│ │ - 转换函数   │  │ - __init__.py│  │
│ │ - 延迟计算   │  │              │  │
│ └─────────────┘  └──────────────┘  │
│                                     │
│ ┌─────────────┐  ┌──────────────┐  │
│ │ 配置文件     │  │  文档更新     │  │
│ │             │  │              │  │
│ │ - config    │  │ - README.md  │  │
│ │ - yaml      │  │              │  │
│ └─────────────┘  └──────────────┘  │
│                                     │
└─────────────────────────────────────┘
    ↓
修改文件
    ↓
测试验证
    ↓
更新文档
```

---

## 场景1: 添加新的Robot Skill

### 示例: 添加 `jump_forward` (向前跳跃) 功能

### 📁 需要修改的文件

| 文件 | 作用 | 优先级 |
|------|------|--------|
| `llm_agent_with_mcp.py` | 定义MCP工具 | ⭐⭐⭐ 必须 |
| `llm_agent_with_mcp.py` | 实现命令转换 | ⭐⭐⭐ 必须 |
| `llm_agent_with_mcp.py` | 计算执行延迟 | ⭐⭐⭐ 必须 |
| `robot_skills.py` | 添加skill方法 | ⭐⭐ 可选 |
| `config.yaml` | 配置参数 | ⭐ 可选 |

### 📝 详细步骤

#### 步骤1: 在 `llm_agent_with_mcp.py` 中定义MCP工具

**位置**: 第30-245行，`MCP_TOOLS` 列表

**添加**:
```python
{
    "type": "function",
    "function": {
        "name": "jump_forward",
        "description": "向前跳跃指定距离",
        "inputSchema": {
            "type": "object",
            "properties": {
                "distance": {
                    "type": "number",
                    "description": "跳跃距离数值"
                },
                "height": {
                    "type": "number",
                    "description": "跳跃高度（米）",
                    "default": 0.5
                },
                "unit": {
                    "type": "string",
                    "description": "距离单位（m/cm/mm）",
                    "enum": ["m", "cm", "mm"],
                    "default": "m"
                }
            },
            "required": ["distance"]
        }
    }
}
```

#### 步骤2: 在 `build_command_from_tool()` 中添加转换逻辑

**位置**: 第553-629行

**添加**:
```python
elif function_name == "jump_forward":
    distance = function_args["distance"]
    height = function_args.get("height", 0.5)
    unit = function_args.get("unit", "m")
    command = {
        "action": "jump",
        "parameters": {
            "direction": "forward",
            "distance": f"{distance}{unit}",
            "height": f"{height}m"
        }
    }
```

#### 步骤3: 在 `get_action_delay_from_command()` 中添加延迟计算

**位置**: 第632-673行

**添加**:
```python
elif action == "jump":
    # 跳跃时间 = 跳跃高度相关的滞空时间 + 距离/速度
    height = float(params.get("height", "0.5m").replace("m", ""))
    distance = parse_distance(params.get("distance", "1m"))

    # 物理估算: t = 2*sqrt(2*h/g)
    import math
    air_time = 2 * math.sqrt(2 * height / 9.8)
    ground_time = distance / 1.0  # 假设1m/s

    return max(1.0, air_time + ground_time)
```

#### 步骤4 (可选): 在 `robot_skills.py` 中添加skill方法

**位置**: 任意位置，建议在其他move方法附近

**添加**:
```python
def jump_forward(self, distance: float, height: float = 0.5, unit: str = "m") -> Dict[str, Any]:
    """
    向前跳跃

    Args:
        distance: 跳跃距离
        height: 跳跃高度（米）
        unit: 距离单位

    Returns:
        执行结果
    """
    print(f"[Robot Skill] jump_forward: {distance}{unit}, height: {height}m")
    command = {
        "action": "jump",
        "parameters": {
            "direction": "forward",
            "distance": f"{distance}{unit}",
            "height": f"{height}m"
        }
    }
    result = self.adapter.send_command(command)
    return {
        "success": True,
        "action": "jump_forward",
        "distance": distance,
        "height": height,
        "unit": unit,
        "result": result
    }
```

#### 步骤5 (可选): 更新配置文件

**位置**: `config.yaml` 第42-58行

**添加到 `skills.enabled` 列表**:
```yaml
skills:
  enabled:
    - "jump_forward"  # 新增
    # ... 其他skills
```

### ✅ 测试

```python
# 测试新skill
from adapters import DoraAdapter
from robot_skills import RobotSkills

adapter = DoraAdapter()
robot = RobotSkills(adapter)

# 测试
result = robot.jump_forward(1.0, 0.5, "m")
print(result)
```

---

## 场景2: 修改现有Skill

### 示例: 修改 `move_forward` 支持加速度参数

### 📁 需要修改的文件

| 文件 | 需要修改的内容 |
|------|--------------|
| `llm_agent_with_mcp.py` | MCP_TOOLS定义、build_command_from_tool、get_action_delay_from_command |
| `robot_skills.py` | move_forward方法 |
| 适配器 | 可能需要支持新参数 |

### 📝 详细步骤

#### 步骤1: 修改MCP工具定义

**位置**: `llm_agent_with_mcp.py` 第68-83行

**修改前**:
```python
{
    "type": "function",
    "function": {
        "name": "move_forward",
        "description": "向前移动指定距离",
        "inputSchema": {
            "type": "object",
            "properties": {
                "distance": {"type": "number"},
                "unit": {"type": "string", "enum": ["m", "cm", "mm"], "default": "m"}
            },
            "required": ["distance"]
        }
    }
}
```

**修改后**:
```python
{
    "type": "function",
    "function": {
        "name": "move_forward",
        "description": "向前移动指定距离",
        "inputSchema": {
            "type": "object",
            "properties": {
                "distance": {"type": "number"},
                "unit": {"type": "string", "enum": ["m", "cm", "mm"], "default": "m"},
                "acceleration": {
                    "type": "number",
                    "description": "加速度（m/s²），默认0.5",
                    "default": 0.5
                }
            },
            "required": ["distance"]
        }
    }
}
```

#### 步骤2: 修改命令转换

**位置**: `llm_agent_with_mcp.py` 第578-585行

**修改前**:
```python
elif function_name == "move_forward":
    distance = function_args["distance"]
    unit = function_args.get("unit", "m")
    command = {
        "action": "navigate",
        "parameters": {"direction": "front", "distance": f"{distance}{unit}"}
    }
```

**修改后**:
```python
elif function_name == "move_forward":
    distance = function_args["distance"]
    unit = function_args.get("unit", "m")
    acceleration = function_args.get("acceleration", 0.5)
    command = {
        "action": "navigate",
        "parameters": {
            "direction": "front",
            "distance": f"{distance}{unit}",
            "acceleration": acceleration
        }
    }
```

#### 步骤3: 修改延迟计算

**位置**: `llm_agent_with_mcp.py` 第632-673行

**修改**:
```python
elif action == "navigate":
    if "distance" in params:
        distance_str = params["distance"]
        if distance_str.endswith("cm"):
            distance = float(distance_str.replace("cm", "")) / 100
        elif distance_str.endswith("mm"):
            distance = float(distance_str.replace("mm", "")) / 1000
        else:  # m
            distance = float(distance_str.replace("m", ""))

        # 使用加速度计算时间
        acceleration = params.get("acceleration", 0.5)
        # t = sqrt(2*d/a)
        import math
        duration = math.sqrt(2 * distance / acceleration)

        return max(1.0, duration)
```

#### 步骤4: 修改robot_skills.py

**位置**: `robot_skills.py` 第56-73行

**修改**:
```python
def move_forward(self, distance: float, unit: str = "m", acceleration: float = 0.5) -> Dict[str, Any]:
    """
    向前移动

    Args:
        distance: 距离数值
        unit: 单位（m/cm/mm），默认m
        acceleration: 加速度（m/s²），默认0.5

    Returns:
        执行结果
    """
    print(f"[Robot Skill] move_forward: {distance}{unit}, acceleration: {acceleration}")
    command = {
        "action": "navigate",
        "parameters": {
            "direction": "front",
            "distance": f"{distance}{unit}",
            "acceleration": acceleration
        }
    }
    result = self.adapter.send_command(command)
    return {
        "success": True,
        "action": "move_forward",
        "distance": distance,
        "unit": unit,
        "acceleration": acceleration,
        "result": result
    }
```

---

## 场景3: 添加新适配器

### 示例: 添加 `MQTT` 适配器

### 📁 需要修改的文件

| 文件 | 作用 | 优先级 |
|------|------|--------|
| `adapters/mqtt_adapter.py` | 新建适配器文件 | ⭐⭐⭐ 必须 |
| `adapters/__init__.py` | 导出新适配器 | ⭐⭐⭐ 必须 |
| `mcp_robot_server.py` | 支持新适配器 | ⭐⭐ 必须 |
| `config.yaml` | 添加MQTT配置 | ⭐ 必须 |
| `README.md` | 文档说明 | ⭐ 可选 |

### 📝 详细步骤

#### 步骤1: 创建适配器文件

**新建**: `adapters/mqtt_adapter.py`

```python
# -*- coding: utf-8 -*-
"""
MQTT Adapter
通过MQTT协议发送机器人控制命令
"""
import sys
import json
from typing import Dict, Any

if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False
    print("INFO: paho-mqtt not installed. MQTT adapter will be disabled.")
    print("      To enable MQTT support, install: pip install paho-mqtt")

from .base_adapter import BaseAdapter


class MQTTAdapter(BaseAdapter):
    """MQTT协议适配器"""

    def __init__(self, config: Dict[str, Any] = None):
        """
        初始化MQTT适配器

        Args:
            config: 配置参数
                   - broker: MQTT broker地址，默认"localhost"
                   - port: MQTT端口，默认1883
                   - topic: 发布话题，默认"robot/command"
                   - client_id: 客户端ID
                   - qos: QoS等级（0/1/2），默认0
        """
        super().__init__(config)
        self.broker = self.config.get("broker", "localhost")
        self.port = self.config.get("port", 1883)
        self.topic = self.config.get("topic", "robot/command")
        self.client_id = self.config.get("client_id", "mcp_robot_control")
        self.qos = self.config.get("qos", 0)

        self.client = None
        self.is_connected = False

        if not MQTT_AVAILABLE:
            print("[MQTTAdapter] paho-mqtt not available")
            return

        try:
            # 创建MQTT客户端
            self.client = mqtt.Client(client_id=self.client_id)

            # 连接到broker
            self.client.connect(self.broker, self.port, 60)
            self.client.loop_start()

            # 等待连接
            import time
            time.sleep(1)

            if self.client.is_connected():
                self.is_connected = True
                print(f"[MQTTAdapter] Connected to {self.broker}:{self.port}")
            else:
                print(f"[MQTTAdapter] Failed to connect to {self.broker}:{self.port}")

        except Exception as e:
            print(f"[MQTTAdapter] Failed to initialize: {e}")

    def connect(self) -> bool:
        """建立连接"""
        if not MQTT_AVAILABLE:
            return False

        if self.is_connected:
            return True

        return self.client.is_connected if self.client else False

    def disconnect(self) -> bool:
        """断开连接"""
        if self.client:
            self.client.loop_stop()
            self.client.disconnect()
            self.client = None

        self.is_connected = False
        print("[MQTTAdapter] Disconnected")
        return True

    def send_command(self, command: Dict[str, Any]) -> Dict[str, Any]:
        """
        发送命令到MQTT话题

        Args:
            command: 命令字典

        Returns:
            执行结果
        """
        if not self.is_available():
            return {
                "success": False,
                "error": "MQTT adapter not available"
            }

        try:
            # 转换为JSON
            payload = json.dumps(command, ensure_ascii=False)

            # 发布到MQTT
            result = self.client.publish(
                self.topic,
                payload,
                qos=self.qos
            )

            if result.rc == mqtt.MQTT_ERR_SUCCESS:
                print(f"[MQTTAdapter] Published to '{self.topic}': {command['action']}")
                return {
                    "success": True,
                    "adapter": "mqtt",
                    "topic": self.topic,
                    "command": command
                }
            else:
                return {
                    "success": False,
                    "error": f"MQTT publish failed: {result.rc}",
                    "command": command
                }

        except Exception as e:
            print(f"[MQTTAdapter] Failed to send command: {e}")
            return {
                "success": False,
                "error": str(e),
                "command": command
            }

    def is_available(self) -> bool:
        """检查是否可用"""
        return MQTT_AVAILABLE and self.is_connected and self.client is not None

    def get_info(self) -> Dict[str, Any]:
        """获取适配器信息"""
        info = super().get_info()
        info.update({
            "broker": self.broker,
            "port": self.port,
            "topic": self.topic,
            "client_id": self.client_id,
            "qos": self.qos,
            "mqtt_available": MQTT_AVAILABLE
        })
        return info
```

#### 步骤2: 更新 `adapters/__init__.py`

**位置**: 第1-11行

**修改**:
```python
# -*- coding: utf-8 -*-
"""
Adapters for Robot Control
支持不同的通信框架：Dora, ROS1, ROS2, MQTT等
"""
from .base_adapter import BaseAdapter
from .dora_adapter import DoraAdapter
from .ros1_adapter import ROS1Adapter
from .ros2_adapter import ROS2Adapter
from .mqtt_adapter import MQTTAdapter  # 新增

__all__ = ["BaseAdapter", "DoraAdapter", "ROS1Adapter", "ROS2Adapter", "MQTTAdapter"]
```

#### 步骤3: 更新 `mcp_robot_server.py`

**位置**: 第33-34行

**修改**:
```python
from robot_skills import RobotSkills
from adapters import DoraAdapter, ROS1Adapter, ROS2Adapter, MQTTAdapter  # 添加MQTTAdapter
```

**位置**: 第66-82行，`_init_adapter()` 方法

**添加**:
```python
def _init_adapter(self):
    """初始化通信适配器"""
    if self.adapter_type == "dora":
        self.adapter = DoraAdapter()
        ...
    elif self.adapter_type == "ros1":
        self.adapter = ROS1Adapter()
        ...
    elif self.adapter_type == "ros2":
        self.adapter = ROS2Adapter()
        ...
    elif self.adapter_type == "mqtt":  # 新增
        self.adapter = MQTTAdapter()
        if not self.adapter.is_available():
            print("[MCP Server] WARNING: MQTT adapter not available, running in standalone mode")
    else:
        print(f"[MCP Server] Unknown adapter type: {self.adapter_type}")
        return
```

**位置**: 第426-432行，命令行参数

**修改**:
```python
parser.add_argument(
    "--adapter",
    type=str,
    default="dora",
    choices=["dora", "ros1", "ros2", "mqtt"],  # 添加mqtt
    help="Communication adapter (default: dora)"
)
```

#### 步骤4: 更新 `config.yaml`

**位置**: 第9-34行

**添加**:
```yaml
adapter:
  type: "dora"  # 选择 "dora"、"ros1"、"ros2" 或 "mqtt"

  # ... 其他适配器配置

  # MQTT特定配置
  mqtt:
    broker: "localhost"  # MQTT broker地址
    port: 1883          # MQTT端口
    topic: "robot/command"  # 发布话题
    client_id: "mcp_robot_control"  # 客户端ID
    qos: 0              # QoS等级
```

#### 步骤5: 安装依赖

**添加到 `requirements.txt`**:
```
paho-mqtt>=1.6.1
```

---

## 场景4: 添加新的通信架构

### 示例: 添加 `HTTP REST API` 支持

这需要创建一个独立的HTTP服务器，不依赖Dora节点。

### 📁 需要创建的文件

| 文件 | 作用 |
|------|------|
| `http_server.py` | HTTP REST API服务器 |
| `examples/http_client.py` | HTTP客户端示例 |

### 📝 详细实现

#### 步骤1: 创建HTTP服务器

**新建**: `http_server.py`

```python
# -*- coding: utf-8 -*-
"""
HTTP REST API Server
提供HTTP接口控制机器人
"""
from flask import Flask, request, jsonify
import json
from robot_skills import RobotSkills
from adapters import DoraAdapter, ROS1Adapter, ROS2Adapter

app = Flask(__name__)

# 初始化robot skills
adapter = DoraAdapter()  # 或其他适配器
robot = RobotSkills(adapter)

@app.route('/api/command', methods=['POST'])
def execute_command():
    """执行机器人命令"""
    try:
        data = request.json
        skill_name = data.get('skill')
        params = data.get('params', {})

        # 调用对应的skill
        skill_func = getattr(robot, skill_name, None)
        if not skill_func:
            return jsonify({
                "success": False,
                "error": f"Unknown skill: {skill_name}"
            }), 400

        result = skill_func(**params)
        return jsonify(result)

    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@app.route('/api/skills', methods=['GET'])
def list_skills():
    """列出所有可用的skills"""
    skills = [method for method in dir(robot) if not method.startswith('_')]
    return jsonify({
        "skills": skills
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    """获取机器人状态"""
    return jsonify({
        "adapter": adapter.get_info(),
        "robot": robot.get_status()
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
```

#### 步骤2: 使用HTTP API

```python
import requests

# 执行命令
response = requests.post('http://localhost:5000/api/command', json={
    'skill': 'move_forward',
    'params': {
        'distance': 1.0,
        'unit': 'm'
    }
})

print(response.json())
```

---

## 场景5: 自定义任务规划逻辑

### 示例: 添加任务优先级和依赖关系

### 📁 需要修改的文件

| 文件 | 作用 |
|------|------|
| `llm_agent_with_mcp.py` | 修改plan_tasks()函数 |

### 📝 详细步骤

#### 步骤1: 修改prompt模板

**位置**: 第374-415行

**修改**:
```python
planning_prompt = """你是一个机器人任务规划助手。

# 新增规则
1. 为每个任务分配优先级（1-高，2-中，3-低）
2. 标注任务依赖关系（如果有）
3. 使用智能排序优化执行顺序

输出格式（JSON）：
{{
  "tasks": [
    {{
      "step": 1,
      "task": "子任务描述1",
      "type": "动作类型",
      "priority": 1,
      "dependencies": []
    }},
    {{
      "step": 2,
      "task": "子任务描述2",
      "type": "动作类型",
      "priority": 2,
      "dependencies": [1]
    }}
  ],
  "summary": "整体任务概述"
}}

示例：
输入："先检查周围环境，如果安全则前进1米"
输出：
{{
  "tasks": [
    {{
      "step": 1,
      "task": "检查周围环境",
      "type": "感知",
      "priority": 1,
      "dependencies": []
    }},
    {{
      "step": 2,
      "task": "前进1米",
      "type": "移动",
      "priority": 2,
      "dependencies": [1]
    }}
  ],
  "summary": "环境检查后前进"
}}

用户输入：""" + user_input
```

#### 步骤2: 添加任务排序逻辑

**位置**: 第447-461行之后添加

```python
def sort_tasks_by_priority(tasks: list) -> list:
    """
    根据优先级和依赖关系排序任务

    Args:
        tasks: 原始任务列表

    Returns:
        排序后的任务列表
    """
    # 简单的拓扑排序
    sorted_tasks = []
    task_dict = {t['step']: t for t in tasks}
    completed = set()

    while len(sorted_tasks) < len(tasks):
        for task in tasks:
            if task['step'] in completed:
                continue

            # 检查依赖是否满足
            deps = task.get('dependencies', [])
            if all(dep in completed for dep in deps):
                sorted_tasks.append(task)
                completed.add(task['step'])
                break

    return sorted_tasks
```

#### 步骤3: 集成到主流程

**位置**: 第696行附近

**修改**:
```python
# 阶段1: 上层LLM - 任务规划
tasks = plan_tasks(user_input)

# 新增: 根据优先级排序
tasks = sort_tasks_by_priority(tasks)

# 阶段2: 顺序执行每个子任务
...
```

---

## 场景6: 调整执行参数

### 示例: 修改机器人的移动速度、转向速度等

### 📁 需要修改的文件

| 文件 | 位置 | 内容 |
|------|------|------|
| `llm_agent_with_mcp.py` | 第632-673行 | 延迟计算函数 |
| `config.yaml` | 新增params节 | 配置参数 |

### 📝 详细步骤

#### 方式1: 硬编码修改（快速）

**位置**: `llm_agent_with_mcp.py` 第632-673行

```python
def get_action_delay_from_command(command: dict) -> float:
    """根据命令估算执行时间"""
    action = command.get("action")
    params = command.get("parameters", {})

    if action == "navigate":
        if "angle" in params:
            angle_str = params["angle"]
            angle = float(angle_str.replace("deg", "").replace("-", ""))

            # 修改这里: 调整转向速度 (当前45度/秒)
            TURN_SPEED = 60  # 改为60度/秒（更快）
            return max(1.0, angle / TURN_SPEED)

        elif "distance" in params:
            distance_str = params["distance"]
            distance = parse_distance(distance_str)

            # 修改这里: 调整移动速度 (当前0.5m/s)
            MOVE_SPEED = 1.0  # 改为1m/s（更快）
            return max(0.5, distance / MOVE_SPEED)
```

#### 方式2: 配置文件修改（推荐）

**步骤1**: 更新 `config.yaml`

```yaml
# 新增执行参数配置
execution:
  turn:
    speed: 60  # 度/秒
  move:
    speed: 1.0  # 米/秒
  pick:
    duration: 2.0  # 秒
  place:
    duration: 2.0  # 秒
```

**步骤2**: 读取配置

**位置**: `llm_agent_with_mcp.py` 开头添加

```python
import yaml

# 加载配置
with open(os.path.join(BASE_DIR, 'config.yaml'), 'r', encoding='utf-8') as f:
    CONFIG = yaml.safe_load(f)

EXECUTION_CONFIG = CONFIG.get('execution', {})
```

**步骤3**: 使用配置

```python
def get_action_delay_from_command(command: dict) -> float:
    """根据命令估算执行时间"""
    action = command.get("action")
    params = command.get("parameters", {})

    if action == "navigate":
        if "angle" in params:
            angle = float(params["angle"].replace("deg", "").replace("-", ""))
            turn_speed = EXECUTION_CONFIG.get('turn', {}).get('speed', 45)
            return max(1.0, angle / turn_speed)

        elif "distance" in params:
            distance = parse_distance(params["distance"])
            move_speed = EXECUTION_CONFIG.get('move', {}).get('speed', 0.5)
            return max(0.5, distance / move_speed)
```

---

## ✅ 扩展检查清单

完成扩展后，请使用此清单验证：

### 代码检查

- [ ] 所有新增文件已创建
- [ ] 所有必需的导入语句已添加
- [ ] 函数签名保持一致
- [ ] 错误处理已实现
- [ ] 日志输出清晰

### 功能测试

- [ ] 单元测试通过（如果有）
- [ ] 集成测试通过
- [ ] 在Dora环境中测试成功
- [ ] 在真实机器人上测试成功（如果适用）

### 兼容性检查

- [ ] 不影响现有的skills
- [ ] 不影响现有的适配器
- [ ] 向后兼容旧版本

### 文档更新

- [ ] README.md已更新
- [ ] 代码注释完整
- [ ] 使用示例已添加
- [ ] 变更日志已更新

### 性能检查

- [ ] 没有明显的性能下降
- [ ] 内存使用正常
- [ ] 响应时间可接受

---

## 🐛 常见问题

### Q1: 添加新skill后LLM不调用

**可能原因**:
- MCP工具定义中的参数类型错误
- required参数列表缺少必要参数
- 描述不够清晰

**解决方案**:
```python
# 检查工具定义
{
    "type": "function",
    "function": {
        "name": "your_skill",
        "description": "清晰描述这个技能做什么，何时使用",
        "inputSchema": {
            "type": "object",
            "properties": {
                "param1": {
                    "type": "number",  # 确保类型正确
                    "description": "详细说明这个参数"
                }
            },
            "required": ["param1"]  # 确保必需参数
        }
    }
}
```

### Q2: 延迟计算不准确

**解决方案**:
1. 测量实际执行时间
2. 调整 `get_action_delay_from_command()` 中的参数
3. 或使用配置文件动态调整

### Q3: 新适配器无法初始化

**检查项**:
```python
# 1. 检查依赖是否安装
try:
    import required_library
except ImportError:
    print("Library not installed")

# 2. 检查配置是否正确
print(f"Config: {self.config}")

# 3. 检查连接逻辑
print(f"Connected: {self.is_connected}")
```

### Q4: 命令转换失败

**调试步骤**:
```python
# 在 build_command_from_tool() 中添加
print(f"Debug: function_name={function_name}")
print(f"Debug: function_args={function_args}")

# 确保所有分支都返回command
if not command:
    print(f"Warning: No command generated for {function_name}")
```

### Q5: 多步骤执行顺序错误

**检查**:
1. `plan_tasks()` 返回的任务是否正确排序
2. for循环是否正确遍历
3. 是否有异步执行导致并发

---

## 📚 参考资源

### 代码结构理解

```
用户输入
    ↓
plan_tasks() ──────────────→ 子任务列表
    ↓                            ↓
execute_single_task()    每个子任务
    ↓
build_command_from_tool()  MCP工具 → Dora命令
    ↓
get_action_delay_from_command()  计算延迟
    ↓
node.send_output()  发送
    ↓
time.sleep(delay)  等待
```

### 文件依赖关系

```
llm_agent_with_mcp.py (核心)
    ├─ 使用 MCP_TOOLS (工具定义)
    ├─ 调用 build_command_from_tool() (转换)
    └─ 调用 get_action_delay_from_command() (延迟)

robot_skills.py (可选，用于独立服务器)
    └─ 使用 adapters (通信层)

adapters/ (适配器层)
    ├─ base_adapter.py (接口)
    ├─ dora_adapter.py (Dora)
    ├─ ros1_adapter.py (ROS1)
    ├─ ros2_adapter.py (ROS2)
    └─ xxx_adapter.py (新适配器)
```

---

## 🎓 扩展建议

### 渐进式扩展

1. **阶段1**: 修改现有skill（最简单）
2. **阶段2**: 添加新skill（需要修改3-4个地方）
3. **阶段3**: 添加新适配器（需要理解架构）
4. **阶段4**: 修改核心逻辑（需要充分测试）

### 最佳实践

1. **先在Dora中测试** - 仿真环境安全
2. **逐步添加功能** - 每次只改一个功能
3. **保持向后兼容** - 不要破坏现有功能
4. **添加日志** - 方便调试
5. **编写测试** - 确保稳定性
6. **更新文档** - 方便后续维护

### 版本控制建议

```bash
# 每次扩展创建分支
git checkout -b feature/add-jump-skill

# 完成后合并
git checkout main
git merge feature/add-jump-skill
```

---

**最后更新**: 2026-01-09
**版本**: 1.0.0
**维护者**: MCP Robot Control Team
