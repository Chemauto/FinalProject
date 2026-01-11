# Robot_Module - 机器人模块

## 概述

Robot_Module 是项目的**核心模块之一**，负责定义每个机器人的配置、技能实现和通信映射。每个机器人对应一个独立的子目录，包含配置文件和技能实现。

### 核心功能

- **机器人配置**: 定义机器人属性、通信方式、话题映射
- **技能实现**: 实现机器人特定的行为技能
- **通信映射**: 配置 ROS2/Dora 话题与机器人通信的映射关系
- **完全模块化**: 添加新机器人无需修改核心代码

### 设计原则

1. **配置驱动**: 所有机器人属性通过 YAML 配置文件定义
2. **技能分离**: 技能实现与机器人配置分离
3. **自动发现**: MCP_Module 自动发现和加载新机器人
4. **易于扩展**: 添加新机器人只需 4 步

## 目录结构

```
Robot_Module/
├── __init__.py
│
├── Sim_2D/                    # 2D 仿真机器人 (差速驱动)
│   ├── __init__.py
│   ├── robot_config.yaml      # 机器人配置
│   └── skills/                # 技能实现
│       ├── __init__.py
│       └── sim_2d_skills.py
│
├── Go2_Quadruped/             # Unitree Go2 四足机器人
│   ├── __init__.py
│   ├── robot_config.yaml
│   ├── go2_description/       # URDF 模型文件
│   └── skills/
│       ├── __init__.py
│       └── go2_skills.py
│
└── 4Lun/                      # 4Lun 机器人配置
    ├── __init__.py
    ├── robot_config.yaml
    └── skills/
        ├── __init__.py
        └── 4lun_skills.py
```

## 配置文件格式

每个机器人必须有一个 `robot_config.yaml` 配置文件。

### 完整配置示例

```yaml
# robot_config.yaml

# 机器人基本信息
robot:
  name: "Sim2D"                      # 机器人名称
  type: "differential_drive"         # 机器人类型
  description: "2D Differential Drive Robot"

# 支持的通信方式
communication:
  - ROS2                             # 支持 ROS2
  # - Dora                           # 可选: 支持 Dora

# ROS2 话题映射配置
ros2:
  # 订阅的话题（接收命令）
  subscribe:
    command_topic: "/robot_command"

  # 发布的话题（发送控制命令）
  publish:
    cmd_vel: "/cmd_vel"              # 速度控制 (geometry_msgs/Twist)
    # gripper: "/gripper/command"   # 可选: 夹爪控制
    # joint_states: "/joint_states" # 可选: 关节状态

# 支持的技能列表
skills:
  - move_forward                     # 向前移动
  - move_backward                    # 向后移动
  - rotate                           # 旋转
  - stop                             # 停止
```

### 配置项说明

#### robot (机器人信息)

| 字段 | 类型 | 必需 | 说明 |
|-----|------|------|------|
| name | string | 是 | 机器人名称 (唯一标识) |
| type | string | 是 | 机器人类型 |
| description | string | 否 | 机器人描述 |

**常用机器人类型**:
- `differential_drive`: 差速驱动机器人
- `quadruped`: 四足机器人
- `manipulator`: 机械臂
- `omnidirectional`: 全向移动机器人

#### communication (通信方式)

支持的通信协议列表:
- `ROS2`
- `Dora`

#### ros2 (ROS2 配置)

| 配置项 | 说明 |
|-------|------|
| subscribe.command_topic | 接收命令的 ROS2 话题 |
| publish.cmd_vel | 速度控制话题 |
| publish.gripper | 夹爪控制话题 (可选) |
| publish.joint_states | 关节状态话题 (可选) |

## 技能实现

### 技能命名规范

所有技能函数**必须**以 `skill_` 开头，MCP_Module 会自动发现并注册这些函数。

```python
# 正确
def skill_move_forward(distance: float = 1.0, speed: float = 0.3):
    pass

# 错误 (不会被自动发现)
def move_forward(distance: float = 1.0, speed: float = 0.3):
    pass
```

### 技能函数签名

```python
def skill_<name>(param1: type, param2: type, ...) -> Dict[str, Any]:
    """
    技能描述（会显示给 LLM）

    Args:
        param1: 参数1说明
        param2: 参数2说明

    Returns:
        执行结果字典
    """
    return {
        'action': 'action_name',
        'parameters': {
            'param1': 'value1',
            'param2': 'value2'
        }
    }
```

### 返回值格式

所有技能应返回统一格式的字典:

```python
{
    'action': 'action_name',      # 动作名称 (如 navigate, turn, stop)
    'parameters': {                # 动作参数
        'direction': 'front',
        'distance': '1.0m'
    },
    'delay': 2.0                   # 可选: 预计执行时间（秒）
}
```

### 内置技能示例

#### Sim_2D 机器人

```python
# Robot_Module/Sim_2D/skills/sim_2d_skills.py

def skill_move_forward(distance: float = 1.0, speed: float = 0.2) -> Dict[str, Any]:
    """
    向前移动

    Args:
        distance: 移动距离(米)
        speed: 移动速度(m/s)

    Returns:
        执行结果
    """
    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'front',
            'distance': f'{distance}m'
        }
    }

def skill_turn(angle: float, angular_speed: float = 0.5) -> Dict[str, Any]:
    """
    原地旋转
    - 正角度(>0): 向左转(逆时针)
    - 负角度(<0): 向右转(顺时针)

    Args:
        angle: 旋转角度(度)
        angular_speed: 角速度(rad/s)

    Returns:
        执行结果
    """
    return {
        'action': 'turn',
        'parameters': {
            'angle': f'{angle}deg'
        }
    }

def skill_stop() -> Dict[str, Any]:
    """立即停止"""
    return {
        'action': 'stop',
        'parameters': {}
    }
```

## 添加新机器人

只需 4 步，即可添加一个新机器人到系统：

### 步骤 1: 创建目录结构

```bash
mkdir -p Robot_Module/MyNewRobot/skills
touch Robot_Module/MyNewRobot/__init__.py
touch Robot_Module/MyNewRobot/skills/__init__.py
```

### 步骤 2: 创建配置文件

创建 `Robot_Module/MyNewRobot/robot_config.yaml`:

```yaml
robot:
  name: "MyRobot"
  type: "custom"
  description: "My custom robot"

communication:
  - ROS2

ros2:
  subscribe:
    command_topic: "/robot_command"
  publish:
    cmd_vel: "/cmd_vel"

skills:
  - move_forward
  - move_backward
  - rotate
  - stop
```

### 步骤 3: 实现技能

创建 `Robot_Module/MyNewRobot/skills/myrobot_skills.py`:

```python
from typing import Dict, Any

def skill_move_forward(distance: float = 1.0, speed: float = 0.3) -> Dict[str, Any]:
    """向前移动指定距离"""
    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'front',
            'distance': f'{distance}m',
            'speed': speed
        },
        'delay': distance / speed  # 计算执行时间
    }

def skill_move_backward(distance: float = 1.0, speed: float = 0.3) -> Dict[str, Any]:
    """向后移动指定距离"""
    return {
        'action': 'navigate',
        'parameters': {
            'direction': 'back',
            'distance': f'{distance}m',
            'speed': speed
        },
        'delay': distance / speed
    }

def skill_rotate(angle: float, angular_speed: float = 0.5) -> Dict[str, Any]:
    """旋转指定角度（正值为左转，负值为右转）"""
    return {
        'action': 'turn',
        'parameters': {
            'angle': f'{angle}deg',
            'angular_speed': angular_speed
        },
        'delay': abs(angle) / 90.0 * 2.0  # 估算旋转时间
    }

def skill_stop() -> Dict[str, Any]:
    """立即停止机器人运动"""
    return {
        'action': 'stop',
        'parameters': {}
    }
```

### 步骤 4: 创建技能模块导出

编辑 `Robot_Module/MyNewRobot/skills/__init__.py`:

```python
from .myrobot_skills import *
```

**完成！** 系统会自动发现和加载新机器人。

## 使用示例

### 加载机器人

```python
from MCP_Module import create_mcp_bridge

# 加载单个机器人
bridge = create_mcp_bridge(['Sim_2D'])

# 加载多个机器人
bridge = create_mcp_bridge(['Sim_2D', 'Go2_Quadruped', '4Lun'])

# 加载所有可用机器人
bridge = create_mcp_bridge()
```

### 执行机器人技能

```python
# 查看可用技能
skills = bridge.get_available_skills()
print(f"可用技能: {skills}")
# ['move_forward', 'move_backward', 'rotate', 'stop', ...]

# 执行技能
result = bridge.execute_skill('move_forward', distance=2.0, speed=0.3)

if result['success']:
    print(f"✅ 技能执行成功: {result['result']}")
else:
    print(f"❌ 技能执行失败: {result['error']}")
```

### 查询技能信息

```python
# 获取技能详细信息
info = bridge.skill_registry.get_skill_info('move_forward')
print(f"技能: move_forward")
print(f"  描述: {info['description']}")
print(f"  参数: {info['parameters']}")
print(f"  来自机器人: {info['robot']}")
```

## 现有机器人

### Sim_2D (2D 仿真机器人)

- **类型**: 差速驱动
- **用途**: 快速逻辑验证、算法测试
- **技能**: move_forward, move_backward, rotate, stop
- **配置文件**: `Robot_Module/Sim_2D/robot_config.yaml`

### Go2_Quadruped (Unitree Go2 四足机器人)

- **类型**: 四足机器人
- **用途**: 高动态运动、复杂地形
- **技能**: stand_up, lie_down, move_forward, etc.
- **配置文件**: `Robot_Module/Go2_Quadruped/robot_config.yaml`
- **URDF 模型**: `Robot_Module/Go2_Quadruped/go2_description/`

### 4Lun

- **类型**: 自定义机器人
- **配置文件**: `Robot_Module/4Lun/robot_config.yaml`

## 高级配置

### 多机器人协同

可以同时加载多个机器人，实现多机器人协同:

```python
bridge = create_mcp_bridge(['Sim_2D', 'Go2_Quadruped'])

# 按机器人查看技能
from collections import defaultdict
skills_by_robot = defaultdict(list)

for skill_name, info in bridge.skill_registry.get_all_skills_info().items():
    skills_by_robot[info['robot']].append(skill_name)

for robot, skills in skills_by_robot.items():
    print(f"{robot}: {', '.join(skills)}")
```

### 自定义话题映射

在 `robot_config.yaml` 中自定义话题映射:

```yaml
ros2:
  subscribe:
    command_topic: "/my_custom/command"

  publish:
    cmd_vel: "/my_custom/cmd_vel"
    custom_topic: "/my_custom/output"
```

### 条件技能

某些技能可能只在特定条件下可用:

```python
def skill_high_speed_mode(enabled: bool = True):
    """高速模式 (仅在特定机器人上可用)"""
    if not check_robot_capability():
        return {
            'action': 'error',
            'parameters': {'message': '此技能不支持当前机器人'}
        }
    # 实现逻辑
    pass
```

## 依赖

Robot_Module 本身不依赖额外包，但技能实现可能需要:

```
typing     # 类型注解
```

## 相关文档

- [MCP_Module README](../MCP_Module/README.md) - MCP 中间件
- [Middle_Module README](../Middle_Module/README.md) - 通信层
- [Sim_Module README](../Sim_Module/README.md) - 仿真模块
- [Real_Module README](../Real_Module/README.md) - 真实机器人模块
- [主项目 README](../README.md) - 项目总览
