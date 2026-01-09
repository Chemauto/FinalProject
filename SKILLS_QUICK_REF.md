# 🚀 技能集成速查卡

## ⚡ 快速接入新技能（3个文件）

### 📝 文件1: `MCP_Server/robot_skills.py`

**位置**: `RobotSkills` 类中

```python
def your_new_skill(self, param1: str, param2: int = 10) -> dict:
    """你的新技能"""
    command = {
        "action": "your_action_name",
        "parameters": {
            "param1": param1,
            "param2": param2
        }
    }
    return self.adapter.send_command(command)
```

### 📝 文件2: `MCP_Server/mcp_robot_server.py`

**位置**: `_register_tools()` 方法中

```python
Tool(
    name="your_new_skill",
    description="技能描述",
    inputSchema={
        "type": "object",
        "properties": {
            "param1": {"type": "string"},
            "param2": {"type": "integer", "default": 10}
        },
        "required": ["param1"]
    }
)
```

### 📝 文件3: `ROS_Module/ros2/ros2_robot_controller.py`

**步骤1**: 添加处理方法

```python
def handle_your_new_skill(self, params):
    """处理新技能"""
    param1 = params.get('param1', 'default')
    param2 = params.get('param2', 10)

    # 调用机器人接口
    print(f"[ROS2] 执行新技能: {param1}, {param2}")
    # ... 实际控制代码 ...
```

**步骤2**: 在 `command_callback()` 中添加路由

```python
elif action == "your_action_name":
    self.handle_your_new_skill(params)
```

---

## 🤖 Go2翻墙技能 - 完整代码

### robot_skills.py
```python
def climb_wall(self, height: str = "1m", approach: str = "front") -> dict:
    """翻墙技能"""
    command = {
        "action": "climb_wall",
        "parameters": {"height": height, "approach": approach}
    }
    return self.adapter.send_command(command)
```

### mcp_robot_server.py
```python
Tool(
    name="climb_wall",
    description="控制四足机器人翻过障碍物",
    inputSchema={
        "type": "object",
        "properties": {
            "height": {"type": "string", "default": "1m"},
            "approach": {"type": "string", "enum": ["front", "left", "right"], "default": "front"}
        }
    }
)
```

### ros2_robot_controller.py
```python
def handle_climb_wall(self, params):
    """处理翻墙命令"""
    height = params.get('height', '1m')
    approach = params.get('approach', 'front')

    # 翻墙动作序列
    actions = [
        {"name": "approach", "duration": 2.0},
        {"name": "climb", "duration": 3.0},
        {"name": "cross", "duration": 1.0},
    ]

    for action in actions:
        print(f"[ROS2] 执行: {action['name']}")
        # 调用Go2接口
        time.sleep(0.5)
```

**在 `command_callback()` 中添加:**
```python
elif action == "climb_wall":
    self.handle_climb_wall(params)
```

---

## 🧪 测试命令

### 交互式测试
```bash
./start_ros2_mcp.sh
用户> 翻过前面1米高的墙
```

### 命令行测试
```bash
ros2 topic pub /robot_command std_msgs/String \
  "{data: '{\"action\": \"climb_wall\", \"parameters\": {\"height\": \"1m\"}}'}"
```

---

## 📋 开发清单

- [ ] 定义技能方法 (`robot_skills.py`)
- [ ] 注册MCP工具 (`mcp_robot_server.py`)
- [ ] 实现ROS2控制 (`ros2_robot_controller.py`)
- [ ] 添加命令路由 (`command_callback`)
- [ ] 测试功能
- [ ] 更新文档

---

## 💡 常用技能模板

### 导航类
```python
def navigate_to(self, location: str) -> dict:
    """导航到指定位置"""
    command = {"action": "navigate_to", "parameters": {"target": location}}
    return self.adapter.send_command(command)
```

### 动作类
```python
def jump(self, distance: str = "1m") -> dict:
    """跳跃"""
    command = {"action": "jump", "parameters": {"distance": distance}}
    return self.adapter.send_command(command)
```

### 技能类
```python
def dance(self, dance_type: str = "wave") -> dict:
    """舞蹈"""
    command = {"action": "dance", "parameters": {"type": dance_type}}
    return self.adapter.send_command(command)
```

---

**提示**: 详细的步骤说明请参考 `DEPLOY.md`
