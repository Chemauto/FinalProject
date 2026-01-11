# Robot_Module - 机器人模块

## 功能

机器人的配置、技能和通信映射：
- 定义机器人配置
- 实现机器人技能
- 管理 ROS2/Dora 话题映射

## 目录结构

```
Robot_Module/
├── __init__.py
├── Go2_Quadruped/        # Go2 四足机器人
│   ├── __init__.py
│   ├── go2_robot.py
│   ├── robot_config.yaml      # 机器人配置
│   └── skills/
│       ├── __init__.py
│       └── go2_skills.py      # 技能实现
└── Sim_2D/              # 2D 仿真机器人
    ├── __init__.py
    ├── sim_2d_robot.py
    ├── robot_config.yaml
    └── skills/
        ├── __init__.py
        └── sim_2d_skills.py
```

## 添加新机器人

### 步骤 1: 创建目录结构

```bash
mkdir -p Robot_Module/MyNewRobot/skills
```

### 步骤 2: 创建配置文件

`Robot_Module/MyNewRobot/robot_config.yaml`:

```yaml
robot:
  name: "MyRobot"
  type: "custom"

# 支持的通信方式
communication:
  - ROS2

# ROS2 话题映射配置
ros2:
  subscribe:
    command_topic: "/robot_command"
  publish:
    cmd_vel: "/cmd_vel"

# 支持的技能
skills:
  - move_forward
  - rotate
  - stop
```

### 步骤 3: 实现技能

`Robot_Module/MyNewRobot/skills/myrobot_skills.py`:

```python
def skill_move_forward(distance: float = 1.0, speed: float = 0.3):
    """向前移动"""
    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'front',
            'distance': f'{distance}m'
        }
    }

def skill_rotate(angle: float, angular_speed: float = 0.5):
    """旋转"""
    return {
        'action': 'turn_left',
        'parameters': {
            'angle': f'{angle}deg'
        }
    }
```

### 步骤 4: 创建技能模块导出

`Robot_Module/MyNewRobot/skills/__init__.py`:

```python
from .myrobot_skills import *
```

就这么简单！系统会自动发现和加载新机器人。

## 技能命名规范

所有技能函数必须以 `skill_` 开头，例如：
- `skill_move_forward`
- `skill_rotate`
- `skill_stop`

MCP_Module 会自动发现并注册这些技能。
