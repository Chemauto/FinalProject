# MCP_Server - MCP 服务器和技能库

核心模块：MCP 服务器 + 机器人技能库 + 通信适配器。

## 🎯 模块概述

本模块提供：
- **MCP Robot Server** - 基于 Model Context Protocol 的机器人控制服务器
- **Robot Skills** - 丰富的机器人操作技能库
- **通信适配器** - 支持 ROS1、ROS2、Dora 等多种通信方式
- **双层 LLM 架构** - 任务规划 + 任务执行

## 📁 文件结构

```
MCP_Server/
├── mcp_robot_server.py           # MCP 服务器主文件
├── robot_skills.py               # 机器人技能库
├── config.yaml                   # 配置文件
├── requirements.txt              # Python 依赖
├── mcp_client_test.py            # MCP 客户端测试工具
└── adapters/                     # 通信适配器
    ├── __init__.py
    ├── base_adapter.py           # 适配器基类
    ├── dora_adapter.py           # Dora 适配器
    ├── ros1_adapter.py           # ROS1 适配器
    └── ros2_adapter.py           # ROS2 适配器
```

## 🚀 快速开始

### 1. 安装依赖

```bash
pip install mcp openai python-dotenv pyyaml
```

### 2. 配置 API Key

在项目根目录创建 `.env` 文件：

```bash
Test_API_KEY=sk-your-api-key-here
```

获取 API Key: https://dashscope.aliyun.com

### 3. 使用方式

MCP Server 通常被其他模块调用：

- **Dora 版本**: 参考 `Dora_Module/README.md`
- **ROS2 版本**: 参考 `ROS_Module/README.md`

## 🔧 核心组件

### 1. MCP Robot Server (`mcp_robot_server.py`)

基于 MCP 协议的机器人控制服务器，提供标准化的机器人操作接口。

**核心功能：**
- 注册 MCP 工具（Robot Skills）
- 处理 LLM 的工具调用请求
- 通过适配器发送命令到通信层

**使用示例：**
```python
from mcp_robot_server import RobotControlServer

# 创建服务器
server = RobotControlServer(adapter_type="ros2")

# 运行服务器
await server.run()
```

### 2. Robot Skills (`robot_skills.py`)

机器人操作技能库，定义所有可用的机器人操作。

**技能分类：**

#### 导航类
- `turn_left(angle)` - 向左转指定角度
- `turn_right(angle)` - 向右转指定角度
- `move_forward(distance, unit)` - 向前移动
- `move_backward(distance, unit)` - 向后移动
- `move_left(distance, unit)` - 向左移动
- `move_right(distance, unit)` - 向右移动
- `navigate_to(location, direction, distance)` - 导航到指定位置

#### 操作类
- `pick_up(object_name)` - 抓取物体
- `place(object_name, location)` - 放置物体

#### 工具类
- `stop()` - 停止机器人
- `get_status()` - 获取机器人状态
- `get_battery()` - 获取电池状态
- `get_location()` - 获取当前位置
- `wait(seconds)` - 等待指定时间

**添加新技能：**
```python
def my_new_skill(self, param1: str, param2: int = 10):
    """
    我的新技能

    Args:
        param1: 参数1说明
        param2: 参数2说明，默认10

    Returns:
        执行结果
    """
    # 实现逻辑
    result = self.adapter.send_command(...)

    # 返回结果
    return {
        "success": True,
        "message": "执行成功"
    }
```

### 3. 通信适配器 (`adapters/`)

适配器模式，支持多种通信方式。

**适配器列表：**
- `DoraAdapter` - Dora 数据流管道
- `ROS1Adapter` - ROS1 通信
- `ROS2Adapter` - ROS2 通信

**使用适配器：**
```python
from adapters import ROS2Adapter

# 创建适配器
adapter = ROS2Adapter()

# 检查可用性
if adapter.is_available():
    # 初始化
    adapter.init()

    # 发送命令
    adapter.send_command({
        "action": "navigate",
        "parameters": {"direction": "front", "distance": "1m"}
    })
```

## 🏗️ 系统架构

```
┌─────────────────────────────────────────┐
│  LLM 客户端                              │
│  (Claude, GPT-4, etc.)                  │
└──────────┬──────────────────────────────┘
           ↓
┌──────────────────────────────────────────┐
│  MCP Robot Server                        │
│  - 注册工具 (Robot Skills)               │
│  - 处理工具调用                          │
│  - 返回执行结果                          │
└──────────┬──────────────────────────────┘
           ↓
┌──────────────────────────────────────────┐
│  适配器层                                │
│  - DoraAdapter                          │
│  - ROS1Adapter                          │
│  - ROS2Adapter                          │
└──────────┬──────────────────────────────┘
           ↓
┌──────────────────────────────────────────┐
│  通信层                                  │
│  - Dora 数据流                          │
│  - ROS1/ROS2 话题                       │
└──────────┬──────────────────────────────┘
           ↓
┌──────────────────────────────────────────┐
│  机器人                                  │
│  - 仿真器                               │
│  - 真实机器人                           │
└──────────────────────────────────────────┘
```

## 📝 配置文件

`config.yaml` 配置示例：

```yaml
adapter:
  type: "dora"  # 或 "ros1", "ros2"

  ros1:
    node_name: "mcp_robot_control"
    topic_name: "/robot_command"

  ros2:
    node_name: "mcp_robot_control"
    topic_name: "/robot_command"
```

## 🧪 测试工具

### MCP 客户端测试

```bash
python3 mcp_client_test.py
```

可以：
- 列出所有可用工具
- 调用单个工具
- 查看执行结果

## 🔧 扩展开发

### 添加新适配器

1. 继承 `BaseAdapter`
2. 实现 `send_command()` 方法
3. 在 `__init__.py` 中注册

示例：
```python
from adapters.base_adapter import BaseAdapter

class MyAdapter(BaseAdapter):
    def __init__(self):
        super().__init__("my_adapter")

    def send_command(self, command: dict):
        # 实现发送逻辑
        pass

    def is_available(self) -> bool:
        # 检查是否可用
        return True
```

### 添加新技能

在 `robot_skills.py` 的 `RobotSkills` 类中添加新方法：

```python
def my_skill(self, param1: str, param2: int = 10):
    """
    技能描述

    Args:
        param1: 参数说明
        param2: 参数说明，默认10

    Returns:
        执行结果
    """
    # 1. 验证参数
    if not param1:
        return {
            "success": False,
            "error": "param1 不能为空"
        }

    # 2. 构建命令
    command = {
        "action": "my_action",
        "parameters": {
            "param1": param1,
            "param2": param2
        }
    }

    # 3. 发送命令
    result = self.adapter.send_command(command)

    # 4. 返回结果
    return {
        "success": True,
        "message": f"执行成功: {param1}",
        "result": result
    }
```

然后在 MCP Server 中注册：

```python
@self.server.list_tools()
async def handle_list_tools():
    return [
        Tool(
            name="my_skill",
            description="我的新技能",
            inputSchema={
                "type": "object",
                "properties": {
                    "param1": {"type": "string"},
                    "param2": {"type": "integer", "default": 10}
                },
                "required": ["param1"]
            }
        )
    ]

@self.server.call_tool()
async def handle_call_tool(name, arguments):
    if name == "my_skill":
        return self.robot_skills.my_skill(**arguments)
```

## 📚 相关资源

- [项目根目录](../README.md)
- [ROS_Module](../ROS_Module/README.md)
- [Dora_Module](../Dora_Module/README.md)
- [MCP 协议规范](https://modelcontextprotocol.io/)

---

**提示**: 本模块为核心组件，通常不需要单独运行，而是被 ROS_Module 或 Dora_Module 调用。
