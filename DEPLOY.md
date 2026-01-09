# 🚀 机器人控制部署指南

本指南说明如何为你的机器人（如四足机器人Go2）集成新的RL技能，通过LLM控制。

## 📋 部署场景示例

**场景**: Go2四足机器人有一个通过RL训练的"翻墙"技能

**目标**: 通过LLM输入"翻过前面的墙"，系统自动：
1. LLM分解任务
2. 调用`climb_wall`技能
3. 通过ROS2发送到Go2
4. Go2执行翻墙动作

---

## 🎯 需要修改的文件（3步）

### 第1步：在技能库中定义新技能

**文件**: `MCP_Server/robot_skills.py`

**位置**: `RobotSkills`类中添加新方法

**示例代码**:

```python
def climb_wall(self, height: str = "1m") -> dict:
    """
    翻墙技能（Go2四足机器人）

    Args:
        height: 墙的高度，支持 "1m", "50cm", "100cm" 等

    Returns:
        执行结果
    """
    try:
        # 1. 验证参数
        if not height:
            return {
                "success": False,
                "error": "缺少墙的高度参数"
            }

        # 2. 构建ROS2命令
        command = {
            "action": "climb_wall",  # 新的动作类型
            "parameters": {
                "height": height,
                "mode": "autonomous"  # 自主模式
            }
        }

        # 3. 发送到机器人
        print(f"[RobotSkills] 执行翻墙: {height}")
        result = self.adapter.send_command(command)

        # 4. 返回结果
        return {
            "success": True,
            "message": f"成功翻过{height}高的墙",
            "result": result
        }

    except Exception as e:
        return {
            "success": False,
            "error": f"翻墙失败: {str(e)}"
        }
```

### 第2步：注册到MCP工具列表

**文件**: `MCP_Server/mcp_robot_server.py`

**位置**: `_register_tools()` 方法中添加工具定义

**示例代码**:

```python
@self.server.list_tools()
async def handle_list_tools() -> list[Tool]:
    """列出所有可用的tools"""
    return [
        # ... 现有工具 ...

        # 新增：翻墙技能
        Tool(
            name="climb_wall",
            description="控制四足机器人翻过障碍物/墙",
            inputSchema={
                "type": "object",
                "properties": {
                    "height": {
                        "type": "string",
                        "description": "墙的高度（如 1m, 50cm, 100cm）",
                        "default": "1m"
                    },
                    "mode": {
                        "type": "string",
                        "description": "执行模式",
                        "enum": ["autonomous", "manual"],
                        "default": "autonomous"
                    }
                },
                "required": []
            }
        ),
    ]
```

### 第3步：在ROS2端实现实际控制

**文件**: `ROS_Module/ros2/ros2_robot_controller.py`

**位置**: `RobotController`类中添加处理方法

**示例代码**:

```python
def handle_climb_wall(self, params):
    """
    处理翻墙命令（Go2四足机器人）

    Args:
        params: 翻墙参数
            - height: 墙高
            - mode: 模式（autonomous/manual）
    """
    height = params.get('height', '1m')
    mode = params.get('mode', 'autonomous')

    print(f"[ROS2] 开始翻墙: {height}, 模式: {mode}")

    # 方案A: 如果Go2有现成的SDK
    try:
        import go2_sdk  # 假设Go2提供Python SDK

        # 连接到Go2
        robot = go2_sdk.Robot()

        # 执行翻墙动作序列
        if mode == "autonomous":
            # 自主模式：使用RL训练的策略
            robot.execute_skill("climb_wall", height=height)
        else:
            # 手动模式：逐步控制
            robot.stand_up()
            robot.step_forward(0.5)
            robot.climb(height)
            robot.step_forward(0.5)

        print(f"[ROS2] 翻墙完成!")

    except ImportError:
        # 方案B: 如果没有SDK，通过ROS话题控制
        print(f"[ROS2] 使用通用ROS2话题控制")

        # 发布到特定话题（假设Go2监听这些话题）
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String

        # 发送翻墙命令
        cmd_msg = String()
        cmd_msg.data = json.dumps({
            "skill": "climb_wall",
            "height": height,
            "mode": mode
        })

        # Go2的监听节点
        climb_pub = self.create_publisher(String, '/go2/skills', 10)
        climb_pub.publish(cmd_msg)

        print(f"[ROS2] 已发送翻墙命令到 /go2/skills")
```

---

## 📝 完整集成流程

### 步骤1: 定义技能

编辑 `MCP_Server/robot_skills.py`，在 `RobotSkills` 类中添加：

```python
def climb_wall(self, height: str = "1m") -> dict:
    """翻墙技能"""
    command = {
        "action": "climb_wall",
        "parameters": {"height": height}
    }
    return self.adapter.send_command(command)
```

### 步骤2: 注册工具

编辑 `MCP_Server/mcp_robot_server.py`，在工具列表中添加：

```python
Tool(
    name="climb_wall",
    description="翻墙技能（四足机器人）",
    inputSchema={
        "type": "object",
        "properties": {
            "height": {"type": "string", "default": "1m"}
        },
        "required": []
    }
)
```

### 步骤3: 实现ROS2控制

编辑 `ROS_Module/ros2/ros2_robot_controller.py`，添加：

```python
def handle_climb_wall(self, params):
    """处理翻墙命令"""
    height = params.get('height', '1m')

    # 发送命令到Go2
    # ... 实现细节见上面 ...
```

### 步骤4: 更新命令处理

在 `ros2_robot_controller.py` 的 `command_callback()` 中添加：

```python
elif action == "climb_wall":
    self.handle_climb_wall(params)
```

---

## 🤖 Go2四足机器人完整示例

### 假设Go2提供以下接口

**方式1: 使用Go2 SDK**

```python
# 在 ros2_robot_controller.py 中

import go2_msgs.msg

def handle_climb_wall(self, params):
    """Go2翻墙技能"""
    height = params.get('height', '1m')

    # 发布到Go2的技能话题
    from unitree_go2_msgs.msg import SkillTrigger

    skill_pub = self.create_publisher(SkillTrigger, '/go2/skill_trigger', 10)

    msg = SkillTrigger()
    msg.skill_name = "climb_wall"
    msg.parameter = json.dumps({"height": height})

    skill_pub.publish(msg)

    print(f"[ROS2] 已触发Go2翻墙技能: {height}")
```

**方式2: 通过动作序列控制**

```python
def handle_climb_wall(self, params):
    """通过动作序列控制翻墙"""
    from geometry_msgs.msg import Twist
    import time

    height = params.get('height', '1m')

    # 翻墙动作序列
    sequence = [
        {"action": "stand", "duration": 1.0},
        {"action": "adjust posture", "duration": 0.5},
        {"action": "approach wall", "duration": 2.0},
        {"action": "climb", "height": height, "duration": 3.0},
        {"action": " descend", "duration": 1.0},
    ]

    for step in sequence:
        print(f"[ROS2] 执行: {step['action']}")
        # 发送命令到Go2
        # ... 具体实现 ...
        time.sleep(step.get('duration', 1.0))

    print("[ROS2] 翻墙完成!")
```

---

## 🧪 测试新技能

### 测试步骤

**终端1**: 启动ROS2系统
```bash
cd ROS_Module/ros2
./start_ros2_mcp.sh
```

**终端2**: 测试新技能
```bash
# 方式1: 通过交互式MCP测试
输入: "翻过前面1米高的墙"

# 方式2: 手动发送ROS2命令测试
ros2 topic pub /robot_command std_msgs/String \
  "{data: '{\"action\": \"climb_wall\", \"parameters\": {\"height\": \"1m\"}}'}"
```

**预期输出**:
```
🧠 [上层LLM] 任务规划中...
✅ [规划完成] 共分解为 1 个子任务
📋 [任务概述] 翻墙

子任务序列：
  步骤 1: 翻墙 (操作)

⚙️ [执行中] 翻墙
🔧 [工具调用] climb_wall({'height': '1m'})
📤 [ROS2] 发送命令: {'action': 'climb_wall', 'parameters': {'height': '1m'}}
⏳ [等待] 执行时间: 5.0秒.. ✅ 完成!

✅ [执行完成] 任务总结
  1. 翻墙 - ✅ 成功
```

---

## 🔧 通用技能模板

### 运动类技能

```python
def jump(self, distance: str = "1m") -> dict:
    """跳跃技能"""
    command = {
        "action": "jump",
        "parameters": {"distance": distance}
    }
    return self.adapter.send_command(command)
```

### 操作类技能

```python
def dance(self, dance_type: str = "wave") -> dict:
    """舞蹈技能"""
    command = {
        "action": "dance",
        "parameters": {"type": dance_type}
    }
    return self.adapter.send_command(command)
```

### 导航类技能

```python
def navigate_to_room(self, room_name: str) -> dict:
    """导航到指定房间"""
    command = {
        "action": "navigate_to",
        "parameters": {"target": room_name}
    }
    return self.adapter.send_command(command)
```

---

## 📋 技能开发清单

### ✅ 必须实现的3个地方

- [ ] **技能定义** (`MCP_Server/robot_skills.py`)
  - 添加方法到 `RobotSkills` 类
  - 定义输入参数和返回值

- [ ] **工具注册** (`MCP_Server/mcp_robot_server.py`)
  - 在 `_register_tools()` 中添加 `Tool` 定义
  - 定义 `inputSchema`

- [ ] **ROS2实现** (`ROS_Module/ros2/ros2_robot_controller.py`)
  - 添加 `handle_xxx()` 方法
  - 实现实际的机器人控制
  - 在 `command_callback()` 中添加 `elif` 分支

### 🔧 可选实现

- [ ] **Simulator可视化** (`ROS_Module/ros2/ros2_simulator.py`)
  - 在仿真器中显示新技能的动画

- [ ] **参数验证**
  - 添加输入参数的验证逻辑

- [ ] **错误处理**
  - 添加异常捕获和错误恢复

---

## 🎓 实战案例：Go2翻墙技能完整实现

### 1. 技能定义 (`MCP_Server/robot_skills.py`)

```python
def climb_wall(self, height: str = "1m", approach: str = "front") -> dict:
    """
    翻墙技能（Go2四足机器人）

    Args:
        height: 墙的高度 (如 "1m", "50cm", "100cm")
        approach: 接近方式 ("front", "left", "right")

    Returns:
        执行结果字典
    """
    # 构建命令
    command = {
        "action": "climb_wall",
        "parameters": {
            "height": height,
            "approach": approach
        }
    }

    # 发送并返回结果
    result = self.adapter.send_command(command)

    return {
        "success": True,
        "message": f"成功翻过{height}高的墙",
        "result": result
    }
```

### 2. 工具注册 (`MCP_Server/mcp_robot_server.py`)

```python
Tool(
    name="climb_wall",
    description="控制四足机器人翻过障碍物",
    inputSchema={
        "type": "object",
        "properties": {
            "height": {
                "type": "string",
                "description": "墙的高度",
                "default": "1m"
            },
            "approach": {
                "type": "string",
                "description": "接近方向",
                "enum": ["front", "left", "right"],
                "default": "front"
            }
        },
        "required": []
    }
)
```

### 3. ROS2实现 (`ROS_Module/ros2/ros2_robot_controller.py`)

```python
def handle_climb_wall(self, params):
    """处理翻墙命令"""
    height = params.get('height', '1m')
    approach = params.get('approach', 'front')

    print(f"[ROS2] 翻墙技能 - 高度: {height}, 方向: {approach}")

    # 将高度转换为米
    if height.endswith('cm'):
        height_m = float(height.replace('cm', '')) / 100
    elif height.endswith('m'):
        height_m = float(height.replace('m', ''))
    else:
        height_m = 1.0

    # 翻墙动作序列（根据实际Go2调整）
    climbing_actions = [
        # 1. 接近墙
        {"name": "approach", "duration": 2.0},
        # 2. 调整姿态
        {"name": "adjust", "duration": 1.0},
        # 3. 攀爬
        {"name": "climb", "duration": height_m * 3},
        # 4. 跨越
        {"name": "cross", "duration": 1.0},
        # 5. 落地
        {"name": "land", "duration": 1.0},
    ]

    total_duration = 0
    for action in climbing_actions:
        print(f"  [ROS2] 执行: {action['name']}")
        # 这里调用实际的Go2控制接口
        # self.go2.execute(action['name'])
        import time
        time.sleep(0.5)  # 模拟执行时间
        total_duration += action['duration']

    print(f"[ROS2] 翻墙完成! 总耗时: {total_duration:.1f}秒")
```

### 4. 命令路由 (`ros2_robot_controller.py`)

```python
def command_callback(self, msg):
    """处理所有命令"""
    try:
        command = json.loads(msg.data)
        action = command['action']
        params = command.get('parameters', {})

        print(f"\n[ROS2] Received command:")
        print(f"  Action: {action}")
        print(f"  Parameters: {params}")

        # 路由到具体处理函数
        if action == "navigate":
            self.handle_navigate(params)
        elif action == "turn_left":
            self.handle_turn(params, direction="left")
        elif action == "turn_right":
            self.handle_turn(params, direction="right")
        elif action == "pick":
            self.handle_pick(params)
        elif action == "place":
            self.handle_place(params)
        elif action == "stop":
            self.handle_stop()
        elif action == "climb_wall":  # 新增
            self.handle_climb_wall(params)
        else:
            print(f"[ROS2] Unknown action: {action}")

    except json.JSONDecodeError as e:
        self.get_logger().error(f'Failed to parse command: {e}')
```

---

## 🧪 测试新技能

### 交互式测试

```bash
# 启动系统
cd ROS_Module/ros2
./start_ros2_mcp.sh

# 输入测试命令
用户> 翻过前面1米高的墙
用户> 翻过左边50厘米高的墙
用户> 先调整姿态，然后翻墙
```

### 命令行测试

```bash
# 简单翻墙
ros2 topic pub /robot_command std_msgs/String \
  "{data: '{\"action\": \"climb_wall\", \"parameters\": {\"height\": \"1m\"}}'}"

# 指定方向
ros2 topic pub /robot_command std_msgs/String \
  "{data: '{\"action\": \"climb_wall\", \"parameters\": {\"height\": \"50cm\", \"approach\": \"left\"}}'}"
```

---

## 📊 技能开发工作流程

```
1. 需求分析
   ↓
2. 定义技能接口 (robot_skills.py)
   ↓
3. 注册到MCP (mcp_robot_server.py)
   ↓
4. 实现ROS2控制 (ros2_robot_controller.py)
   ↓
5. 添加命令路由 (command_callback)
   ↓
6. 测试验证
   ↓
7. 部署到真实机器人
```

---

## 💡 最佳实践

### 参数设计
- 使用字符串参数（如 "1m"）而非纯数字（更直观）
- 提供合理的默认值
- 支持多种单位（m, cm, mm）

### 错误处理
- 验证输入参数
- 提供清晰的错误信息
- 记录详细的日志

### 文档编写
- 为每个技能编写清晰的docstring
- 说明参数含义和取值范围
- 提供使用示例

---

## 🎯 总结：添加新技能只需3步

1. **在 `robot_skills.py` 中定义方法** (10行)
2. **在 `mcp_robot_server.py` 中注册工具** (15行)
3. **在 `ros2_robot_controller.py` 中实现控制** (20行)

**总计约 45 行代码，即可为任何机器人添加新技能！** 🚀

---

**下一步**: 查看具体机器人的SDK文档，了解如何控制机器人，然后按照上述模板集成。
