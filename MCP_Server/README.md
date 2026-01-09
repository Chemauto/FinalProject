# MCP Robot Control Server

基于MCP (Model Context Protocol) 的机器人控制服务器，采用**双层LLM架构**实现智能任务分解和顺序执行。支持Dora仿真和ROS1真实机器人控制。

## 📋 目录

- [核心特性](#核心特性)
- [系统架构](#系统架构)
- [代码实现思路](#代码实现思路)
- [目录结构](#目录结构)
- [快速开始](#快速开始)
- [详细使用说明](#详细使用说明)
- [扩展指南](#扩展指南)
- [故障排除](#故障排除)

---

## 🎯 核心特性

### 1. **双层LLM架构** ⭐
- **上层LLM（任务规划器）**: 将复杂指令智能分解为子任务序列
- **下层LLM（任务执行器）**: 将每个子任务转换为具体的robot skills调用
- **严格顺序执行**: 确保每个动作完成后再执行下一个

### 2. **多框架支持**
- ✅ **Dora**: 完整支持，用于仿真和快速原型
- ✅ **ROS1**: 完整支持，用于实际机器人控制
- 🔧 **扩展性**: 通过适配器模式轻松添加新框架（如ROS2）

### 3. **智能任务处理**
- 自动识别复杂指令（如"先左转90度，再往前走1米"）
- 智能分解为2-5个原子子任务
- 自动计算每个动作的执行时间
- 实时显示执行进度

### 4. **丰富的Robot Skills**

#### 导航类
- `turn_left(angle)` - 左转指定角度
- `turn_right(angle)` - 右转指定角度
- `move_forward(distance, unit)` - 前进
- `move_backward(distance, unit)` - 后退
- `move_left(distance, unit)` - 左移
- `move_right(distance, unit)` - 右移
- `navigate_to(location, direction, distance)` - 导航到指定位置

#### 操作类
- `pick_up(object_name)` - 抓取物体
- `place(object_name, location)` - 放置物体

#### 工具类
- `stop()` - 停止机器人
- `get_status()` - 获取状态
- `wait(seconds)` - 等待

---

## 🏗️ 系统架构

### 整体架构图

```
用户输入自然语言
    ↓
┌─────────────────────────────────────────┐
│  llm_agent_with_mcp.py (Dora节点)       │
│                                         │
│  ┌───────────────────────────────────┐  │
│  │ 上层LLM - 任务规划器              │  │
│  │ plan_tasks()                      │  │
│  │                                    │  │
│  │ 输入: "先左转90度，再往前走1米"    │  │
│  │ 输出: [                            │  │
│  │   {step:1, task:"向左转90度"},     │  │
│  │   {step:2, task:"向前走1米"}      │  │
│  │ ]                                  │  │
│  └───────────────────────────────────┘  │
│                 ↓                        │
│  ┌───────────────────────────────────┐  │
│  │ 顺序执行循环                       │  │
│  │ for task in tasks:                │  │
│  └───────────────────────────────────┘  │
│                 ↓                        │
│  ┌───────────────────────────────────┐  │
│  │ 下层LLM - 任务执行器              │  │
│  │ execute_single_task()            │  │
│  │                                    │  │
│  │ 输入: "向左转90度"                │  │
│  │ 输出: turn_left(90)               │  │
│  └───────────────────────────────────┘  │
│                 ↓                        │
│  build_command_from_tool()             │
│  (MCP工具 → Dora命令)                  │
│                 ↓                        │
│  node.send_output("command", ...)      │
│  (发送到Dora数据流)                     │
│                 ↓                        │
│  智能延迟 (等待动作完成)                │
└─────────────────────────────────────────┘
    ↓
┌─────────────────────────────────────────┐
│  Dora Simulator                         │
│  (执行机器人动作并渲染)                  │
└─────────────────────────────────────────┘
```

### 双层LLM工作流程

#### 阶段1: 任务规划（上层LLM）

**输入**: 用户自然语言指令
```python
"先左转90度，再往前走1米"
```

**处理**: `plan_tasks()` 函数
- 使用专门的prompt模板引导LLM分解任务
- 要求输出JSON格式的任务列表
- 每个任务包含：步骤号、任务描述、动作类型

**输出**: 子任务序列
```json
{
  "tasks": [
    {"step": 1, "task": "向左转90度", "type": "转向"},
    {"step": 2, "task": "向前走1米", "type": "移动"}
  ],
  "summary": "左转后前进"
}
```

#### 阶段2: 顺序执行（下层LLM）

对每个子任务：

**输入**: 单个子任务描述
```python
"向左转90度"
```

**处理**: `execute_single_task()` 函数
- 调用下层LLM，使用MCP工具定义
- LLM自动选择合适的工具（如`turn_left`）
- 提取工具名称和参数

**输出**: 工具调用
```python
{
  "name": "turn_left",
  "arguments": {"angle": 90}
}
```

#### 阶段3: 命令转换与发送

**输入**: 工具调用
```python
turn_left(angle=90)
```

**处理**: `build_command_from_tool()` 函数
- 将MCP工具调用转换为Dora命令格式
- 添加动作类型和参数

**输出**: Dora命令
```python
{
  "action": "navigate",
  "parameters": {"angle": "-90deg"}
}
```

#### 阶段4: 执行与延迟

**发送命令**:
```python
node.send_output("command", arrow_data)
```

**智能延迟**:
```python
delay = get_action_delay_from_command(command)
# 例如: 90度转向 = 2.0秒
time.sleep(delay)
```

---

## 💡 代码实现思路

### 1. 核心设计原则

#### 分层架构
```
规划层（上层LLM）
    ↓ 分解任务
执行层（下层LLM）
    ↓ 转换工具
通信层（Dora/ROS1）
    ↓ 发送命令
执行层（Simulator/Robot）
```

**优势**:
- 职责清晰：规划 vs 执行
- 易于调试：每层可独立测试
- 易于扩展：修改某一层不影响其他层

#### 顺序执行保证
```python
for idx, task in enumerate(tasks, 1):
    print(f"【步骤 {idx}/{len(tasks)}】")
    result = execute_single_task(task["task"], node)
    # 等待当前任务完成
    # 然后才开始下一个任务
```

**关键点**:
- 使用同步循环（不是异步并行）
- 每个任务完成后才开始下一个
- 通过`time.sleep()`确保动作完成

### 2. 关键函数说明

#### `plan_tasks(user_input: str) -> list`

**功能**: 任务规划器

**实现思路**:
1. 设计专门的prompt模板，包含：
   - 角色定义：任务规划助手
   - 输出格式：JSON schema
   - 示例：展示正确的输入输出

2. 调用LLM时使用低temperature（0.3）：
   ```python
   completion = client.chat.completions.create(
       model="qwen-plus",
       temperature=0.3,  # 降低随机性
       messages=[...]
   )
   ```

3. 解析JSON响应：
   ```python
   # 处理可能的markdown代码块
   if response_text.startswith("```"):
       response_text = response_text.split("```")[1]

   plan = json.loads(response_text)
   ```

4. 错误处理：如果解析失败，将整个输入作为单个任务

**扩展建议**:
- 可以添加任务验证逻辑
- 可以优化prompt以适应特定场景
- 可以添加任务优先级

#### `execute_single_task(task_description: str, dora_node) -> dict`

**功能**: 任务执行器

**实现思路**:
1. 调用下层LLM，传入MCP工具定义
2. LLM自动选择合适的工具（function calling）
3. 提取工具名称和参数
4. 转换为Dora命令并发送
5. 等待动作完成

**关键代码**:
```python
# 调用下层LLM
completion = client.chat.completions.create(
    model="qwen-plus",
    messages=[...],
    tools=MCP_TOOLS,  # 传入工具定义
    tool_choice="auto"
)

# 提取工具调用
tool_call = response_message.tool_calls[0]
function_name = tool_call.function.name
function_args = json.loads(tool_call.function.arguments)

# 转换为Dora命令
command = build_command_from_tool(function_name, function_args)
```

**扩展建议**:
- 可以添加执行结果反馈
- 可以支持多个工具调用（一个任务多个动作）
- 可以添加错误重试机制

#### `build_command_from_tool(function_name: str, function_args: dict) -> dict`

**功能**: MCP工具 → Dora命令转换

**实现思路**:
1. 根据工具名称匹配对应的Dora动作
2. 格式化参数（如单位转换）
3. 构建标准的Dora命令格式

**示例**:
```python
if function_name == "turn_left":
    angle = function_args.get("angle", 90)
    command = {
        "action": "navigate",
        "parameters": {"angle": f"-{angle}deg"}  # Dora格式
    }
```

**扩展建议**:
- 添加参数验证
- 添加参数范围检查
- 支持更多工具类型

#### `get_action_delay_from_command(command: dict) -> float`

**功能**: 估算动作执行时间

**实现思路**:
1. 根据动作类型计算时间
2. 转向：与角度成正比
3. 移动：距离 / 速度（假设0.5m/s）

**示例**:
```python
if action == "navigate":
    if "angle" in params:
        angle = float(params["angle"].replace("deg", ""))
        return max(1.5, (angle / 90) * 2.0)  # 90度=2秒
    elif "distance" in params:
        distance = parse_distance(params["distance"])
        return max(1.0, distance / 0.5)  # 1米=2秒
```

**扩展建议**:
- 可以从配置文件读取速度参数
- 可以添加动态调整机制
- 可以从simulator获取实际执行时间

### 3. 数据流设计

#### 输入流
```
用户输入 → PyArrow数组 → 提取字符串 → 传递给plan_tasks()
```

#### 输出流
```
MCP工具 → Dora命令 → PyArrow数组 → node.send_output() → Simulator
```

#### 状态管理
```python
results = []  # 存储每个任务的执行结果
for task in tasks:
    result = execute_single_task(task["task"], node)
    results.append(result)
    # 可以在这里添加状态检查
    if not result.get("success"):
        # 决定是否继续
        pass
```

### 4. 错误处理策略

#### 分层错误处理
```python
# 上层：任务规划
try:
    tasks = plan_tasks(user_input)
except Exception as e:
    # 回退到单个任务模式
    tasks = [{"step": 1, "task": user_input, "type": "综合"}]

# 下层：任务执行
for task in tasks:
    try:
        result = execute_single_task(task["task"], node)
    except Exception as e:
        # 记录错误，继续执行下一个
        result = {"success": False, "error": str(e)}
```

#### 用户友好的错误信息
```python
print(f"[错误] 执行失败: {e}")
print(f"[警告] 步骤 {idx} 失败，但继续执行后续任务")
```

---

## 📁 目录结构

```
MCP_Server/
├── llm_agent_with_mcp.py      # ⭐ 核心文件：双层LLM Dora节点
│   ├── plan_tasks()           #   上层LLM：任务规划
│   ├── execute_single_task()  #   下层LLM：任务执行
│   ├── build_command_from_tool()  #   工具→命令转换
│   └── get_action_delay_from_command()  #   延迟计算
│
├── robot_skills.py            # Robot Skills定义（用于独立MCP服务器）
│   └── class RobotSkills      #   包含所有robot控制方法
│
├── adapters/                  # 通信适配器层
│   ├── __init__.py
│   ├── base_adapter.py        #   基础适配器接口
│   ├── dora_adapter.py        #   Dora框架适配器
│   └── ros1_adapter.py        #   ROS1框架适配器
│
├── mcp_robot_server.py        # 独立MCP服务器（可选）
├── config.yaml               # 配置文件
├── requirements.txt          # 依赖列表
├── QUICKSTART.md             # 快速入门指南
└── README.md                 # 本文档
```

**重点文件说明**:

| 文件 | 用途 | 是否核心 |
|------|------|---------|
| `llm_agent_with_mcp.py` | Dora环境中的LLM节点 | ⭐⭐⭐ |
| `robot_skills.py` | 独立MCP服务器时使用 | ⭐⭐ |
| `adapters/` | 框架适配层 | ⭐⭐ |
| `mcp_robot_server.py` | 独立MCP服务器 | ⭐ |

---

## 🚀 快速开始

### 1. 安装依赖

```bash
cd MCP_Server

# 基础依赖（必需）
pip install -r requirements.txt

# 如果使用Dora框架
pip install dora-rs

# 如果使用ROS1框架
# 请先安装ROS1: http://wiki.ros.org/ROS/Installation
# 然后安装rospy（通常随ROS1一起安装）
```

### 2. 配置

编辑 `config.yaml`:

```yaml
adapter:
  type: "dora"  # 选择 "dora" 或 "ros1"

  dora:
    output_id: "command"

  ros1:
    node_name: "mcp_robot_control"
    topic_name: "/robot_command"
    queue_size: 10
```

### 3. 运行

#### 方式A: 使用Dora（推荐）

```bash
cd ../Dora_Module
dora start dora-interactive-mcp.yaml --attach
```

#### 方式B: 作为独立MCP服务器

```bash
# 使用Dora适配器
python mcp_robot_server.py --adapter dora

# 使用ROS1适配器
python mcp_robot_server.py --adapter ros1
```

---

## 📖 详细使用说明

### 输入示例

#### 单步指令
```
前进1米
```
→ 分解为1个任务

#### 顺序指令
```
先左转90度，再往前走1米
```
→ 分解为2个任务：
  1. 向左转90度
  2. 向前走1米

#### 复杂指令
```
先向左转90度，前进1米，然后向右转45度，再后退50厘米
```
→ 分解为4个任务，严格按顺序执行

### 输出示例

```
============================================================
[用户输入] 先左转90度，再往前走1米
============================================================

============================================================
[上层LLM] 任务规划中...
============================================================
[规划完成] 共分解为 2 个子任务
[任务概述] 左转后前进

子任务序列：
  步骤 1: 向左转90度 (转向)
  步骤 2: 向前走1米 (移动)

============================================================
[开始执行] 按顺序执行子任务
============================================================

【步骤 1/2】
──────────────────────────────────────────────────────────
[执行中] 向左转90度
──────────────────────────────────────────────────────────
[工具调用] turn_left({'angle': 90})
[Dora] 发送命令: {'action': 'navigate', 'parameters': {'angle': '-90deg'}}
[等待] 执行时间: 2.0秒.. ✓ 完成!

【步骤 2/2】
──────────────────────────────────────────────────────────
[执行中] 向前走1米
──────────────────────────────────────────────────────────
[工具调用] move_forward({'distance': 1, 'unit': 'm'})
[Dora] 发送命令: {'action': 'navigate', 'parameters': {'direction': 'front', 'distance': '1m'}}
[等待] 执行时间: 2.0秒.. ✓ 完成!

============================================================
[执行完成] 任务总结
============================================================
  1. 向左转90度 - ✓ 成功
  2. 向前走1米 - ✓ 成功
```

---

## 🔧 扩展指南

### 添加新的Robot Skill

#### 1. 定义MCP工具

在 `llm_agent_with_mcp.py` 的 `MCP_TOOLS` 中添加：

```python
{
    "type": "function",
    "function": {
        "name": "your_new_skill",
        "description": "你的新skill描述",
        "parameters": {
            "type": "object",
            "properties": {
                "param1": {
                    "type": "string",
                    "description": "参数1描述"
                }
            },
            "required": ["param1"]
        }
    }
}
```

#### 2. 实现命令转换

在 `build_command_from_tool()` 中添加：

```python
elif function_name == "your_new_skill":
    param1 = function_args["param1"]
    command = {
        "action": "your_action",
        "parameters": {
            "param1": param1
        }
    }
```

#### 3. 更新延迟计算

在 `get_action_delay_from_command()` 中添加：

```python
elif action == "your_action":
    return 2.0  # 估算执行时间
```

### 添加新的适配器

1. 创建 `adapters/your_adapter.py`:

```python
from .base_adapter import BaseAdapter

class YourAdapter(BaseAdapter):
    def __init__(self, config):
        super().__init__(config)
        # 初始化你的框架

    def connect(self):
        # 实现连接逻辑
        pass

    def send_command(self, command):
        # 实现发送命令逻辑
        pass
```

2. 在 `adapters/__init__.py` 中导出：

```python
from .your_adapter import YourAdapter
__all__ = ["BaseAdapter", "DoraAdapter", "ROS1Adapter", "YourAdapter"]
```

### 调整任务规划逻辑

编辑 `plan_tasks()` 中的prompt：

```python
planning_prompt = """你是一个机器人任务规划助手。

# 自定义规则
1. 根据你的场景添加特定规则
2. 定义任务类型
3. 添加约束条件

...（其余prompt）
"""
```

### 调整执行延迟

编辑 `get_action_delay_from_command()` 中的参数：

```python
# 调整转向速度
TURN_SPEED = 45  # 度/秒（当前45度/秒）
delay = angle / TURN_SPEED

# 调整移动速度
MOVE_SPEED = 0.5  # 米/秒（当前0.5m/s）
delay = distance / MOVE_SPEED
```

---

## 🤖 ROS1 完整示例

### 场景：控制真实机器人移动

1. **启动ROS1核心**：

```bash
roscore
```

2. **创建机器人控制器** (`ros1_robot_controller.py`)：

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class RobotController:
    def __init__(self):
        rospy.init_node('mcp_robot_controller')

        # 订阅MCP命令
        rospy.Subscriber('/robot_command', String, self.command_callback, queue_size=10)

        # 发布到机器人速度控制话题
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        print("[ROS1] Robot Controller Ready")

    def command_callback(self, msg):
        """处理来自MCP的命令"""
        try:
            command = json.loads(msg.data)
            action = command['action']
            params = command.get('parameters', {})

            print(f"[ROS1] Received: {action} - {params}")

            # 执行对应的动作
            if action == "navigate":
                self.handle_navigate(params)
            elif action == "pick":
                self.handle_pick(params)
            elif action == "place":
                self.handle_place(params)
            elif action == "stop":
                self.handle_stop()

        except Exception as e:
            print(f"[ROS1] Error processing command: {e}")

    def handle_navigate(self, params):
        """处理导航命令"""
        twist = Twist()

        # 处理移动
        direction = params.get('direction', 'front')
        if 'distance' in params:
            distance_str = params['distance']
            if distance_str.endswith('m'):
                distance = float(distance_str.replace('m', ''))
                speed = 0.5  # m/s
                duration = distance / speed

                if direction == 'front':
                    twist.linear.x = speed
                elif direction == 'back':
                    twist.linear.x = -speed
                elif direction == 'left':
                    twist.linear.y = speed
                elif direction == 'right':
                    twist.linear.y = -speed

                # 发送速度命令
                self.cmd_vel_pub.publish(twist)
                rospy.sleep(duration)
                self.stop_robot()

    def handle_pick(self, params):
        """处理抓取命令"""
        object_name = params.get('object', 'unknown')
        print(f"[ROS1] Picking up {object_name}")
        # 调用机械臂控制逻辑
        # ...

    def handle_place(self, params):
        """处理放置命令"""
        object_name = params.get('object', 'unknown')
        location = params.get('location', 'unknown')
        print(f"[ROS1] Placing {object_name} at {location}")
        # 调用机械臂控制逻辑
        # ...

    def handle_stop(self):
        """停止机器人"""
        self.stop_robot()

    def stop_robot(self):
        """停止机器人运动"""
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    controller = RobotController()
    controller.run()
```

3. **启动控制器**：

```bash
python ros1_robot_controller.py
```

---

## 🐛 故障排除

### 问题1: "Dora adapter not available"

**原因**: dora-rs未安装

**解决方案**:
```bash
pip install dora-rs
```

### 问题2: 任务规划失败

**症状**: `[规划失败] ...`

**原因**: LLM返回的不是有效JSON

**解决方案**:
- 系统会自动回退到单个任务模式
- 检查LLM API密钥是否有效
- 尝试降低任务复杂度

### 问题3: 动作执行太快/太慢

**原因**: 延迟计算不准确

**解决方案**: 编辑 `get_action_delay_from_command()` 调整参数

### 问题4: ROS1连接失败

**原因**: roscore未运行或节点名冲突

**解决方案**:
```bash
# 确保roscore运行
roscore

# 检查节点列表
rosnode list

# 修改节点名（在config.yaml中）
ros1:
  node_name: "my_robot_control"  # 改成独特的名字
```

### 问题5: 多步骤指令只执行第一步

**原因**: 使用的是旧版本（非MCP版本）

**解决方案**:
```bash
# 确保使用MCP版本
dora start dora-interactive-mcp.yaml --attach
```

---

## 📊 架构对比

| 特性 | 旧版本 (JSON生成) | MCP版本 (工具调用) | 双层LLM版本 (当前) |
|------|------------------|-------------------|-------------------|
| 复杂度 | 需要解析JSON | 自动函数调用 | 自动任务分解+执行 |
| 多步骤 | ❌ 需要特殊处理 | ⚠️ 可能同时执行 | ✅ 严格顺序执行 |
| 扩展性 | 修改prompt | 添加函数 | 添加函数即可 |
| 类型安全 | ❌ 无 | ✅ 有 | ✅ 有 |
| 调试难度 | 高 | 中 | 低（分层清晰） |
| 仿真支持 | ✅ Dora | ✅ Dora | ✅ Dora |
| 真实机器人 | ❌ 无 | ✅ ROS1 | ✅ ROS1 |
| 推荐使用 | ❌ | ⚠️ 可用 | ✅ 强烈推荐 |

---

## 📚 相关文档

- [快速入门指南](QUICKSTART.md)
- [Dora官方文档](https://dora.carsmos.ai/)
- [MCP协议规范](https://modelcontextprotocol.io/)
- [Qwen API文档](https://help.aliyun.com/zh/dashscope/developer-reference/api-details)
- [ROS1教程](http://wiki.ros.org/ROS/Tutorials)

---

## 🤝 贡献指南

欢迎提交Issue和Pull Request！

### 开发环境设置

```bash
# 克隆项目
git clone <your-repo-url>
cd Project/MCP_Server

# 创建虚拟环境
conda create -n robot-control python=3.11 -y
conda activate robot-control

# 安装依赖
pip install -r requirements.txt
pip install dora-rs  # 如果使用Dora
```

### 代码风格

- 使用Python类型提示
- 添加docstring注释
- 遵循PEP 8规范

### 测试

```bash
# 单元测试（待添加）
pytest tests/

# 集成测试
cd ../Dora_Module
dora start dora-interactive-mcp.yaml --attach
```

---

## 📄 许可证

MIT License

---

## 🙏 致谢

- [Dora](https://github.com/dora-rs/dora) - 优秀的机器人数据流框架
- [Model Context Protocol](https://modelcontextprotocol.io/) - 标准化的AI工具调用协议
- [Qwen](https://tongyi.aliyun.com/) - 强大的大语言模型
- [ROS](https://www.ros.org/) - 机器人操作系统标准

---

## 📞 联系方式

如有问题或建议，请提交Issue或Pull Request。

---

**最后更新**: 2026-01-09
**版本**: 2.0.0 (双层LLM架构)
