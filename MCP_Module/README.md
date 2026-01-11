# MCP_Module - MCP 中间件模块

## 概述

MCP_Module 实现了 **Model Context Protocol (MCP) 中间件**，作为连接 LLM/VLM 与机器人底层技能的桥梁。该模块负责技能的动态加载、注册和执行管理。

### 核心功能

- **技能注册**: 从 `Robot_Module` 动态发现和加载机器人技能
- **工具定义**: 生成标准化的 MCP 工具定义供 LLM 调用
- **技能执行**: 统一的技能调用接口和结果管理
- **多机器人支持**: 同时管理多个机器人的技能集
- **配置管理**: 加载和管理机器人配置文件

## 文件结构

```
MCP_Module/
├── __init__.py           # 模块导出
└── mcp_bridge.py         # MCPBridge 和 SkillRegistry 实现
```

## 核心类

### 1. SkillRegistry - 技能注册表

管理所有可用机器人技能的核心类。

#### 初始化

```python
from MCP_Module import SkillRegistry

registry = SkillRegistry()
```

#### 方法说明

##### `register_from_robot_module(robot_name)` - 注册机器人技能

从指定机器人模块加载所有 `skill_*` 函数。

```python
registry.register_from_robot_module('Sim_2D')
registry.register_from_robot_module('Go2_Quadruped')

# 输出:
# ✅ [注册] Sim_2D/move_forward
# ✅ [注册] Sim_2D/move_backward
# ✅ [注册] Sim_2D/rotate
# ✅ [注册] Go2_Quadruped/stand_up
# ...
```

##### `get_skill(skill_name)` - 获取技能函数

```python
skill_func = registry.get_skill('move_forward')
result = skill_func(distance=1.0, speed=0.3)
```

##### `list_skills()` - 列出所有技能

```python
skills = registry.list_skills()
print(skills)
# ['move_forward', 'move_backward', 'rotate', 'stop', ...]
```

##### `get_skill_info(skill_name)` - 获取技能详细信息

```python
info = registry.get_skill_info('move_forward')
print(info)
# {
#     'function': <function move_forward at 0x...>,
#     'description': '向前移动指定距离',
#     'parameters': ['distance', 'speed'],
#     'robot': 'Sim_2D'
# }
```

##### `get_all_skills_info()` - 获取所有技能信息

```python
all_skills = registry.get_all_skills_info()
for name, info in all_skills.items():
    print(f"{name}: {info['description']}")
```

### 2. MCPBridge - MCP 桥接层

连接 LLM/VLM 和底层技能的主要接口类。

#### 初始化

```python
from MCP_Module import MCPBridge

bridge = MCPBridge()
```

#### 方法说明

##### `load_robot(robot_name)` - 加载机器人

加载指定机器人的配置和技能。

```python
bridge.load_robot('Sim_2D')
# 输出:
# ✅ [注册] Sim_2D/move_forward
# ✅ [注册] Sim_2D/move_backward
# ✅ [注册] Sim_2D/rotate
# ✅ [加载] Sim_2D 完成
```

##### `load_all_robots()` - 加载所有机器人

自动发现并加载所有可用机器人。

```python
bridge.load_all_robots()
# 自动扫描 Robot_Module/ 下所有包含 robot_config.yaml 的目录
```

##### `get_available_skills()` - 获取可用技能列表

```python
skills = bridge.get_available_skills()
print(f"可用技能: {skills}")
```

##### `get_mcp_tools_definition()` - 获取 MCP 工具定义

生成符合 OpenAI Function Calling 格式的工具定义。

```python
tools = bridge.get_mcp_tools_definition()

# 返回格式:
# [
#     {
#         'type': 'function',
#         'function': {
#             'name': 'move_forward',
#             'description': '向前移动指定距离',
#             'parameters': {
#                 'type': 'object',
#                 'properties': {
#                     'distance': {'type': 'string', 'description': 'distance parameter'},
#                     'speed': {'type': 'string', 'description': 'speed parameter'}
#                 }
#             }
#         }
#     },
#     ...
# ]
```

##### `execute_skill(skill_name, **kwargs)` - 执行技能

```python
result = bridge.execute_skill('move_forward', distance=1.0, speed=0.3)

# 返回格式:
# {
#     'success': True,
#     'skill': 'move_forward',
#     'result': {
#         'action': 'navigate',
#         'parameters': {'direction': 'front', 'distance': '1.0m'}
#     }
# }
```

##### `process_llm_input(llm_output)` - 处理 LLM 输出

处理 LLM 规划的任务列表并执行。

```python
llm_output = {
    'tasks': [
        {'task': '向前移动1米', 'type': '移动'},
        {'task': '左转90度', 'type': '旋转'}
    ]
}

results = bridge.process_llm_input(llm_output)
```

##### `process_vlm_input(vlm_output)` - 处理 VLM 输出

根据 VLM 感知结果生成控制指令。

```python
vlm_output = {
    'scene_type': 'corridor',
    'obstacles': [
        {'type': 'chair', 'position': 'left', 'distance': '1.5m'}
    ]
}

action = bridge.process_vlm_input(vlm_output)
# {'action': 'stop', 'reason': '检测到障碍物', 'obstacles': [...]}
```

## 便捷函数

### `create_mcp_bridge(robots=None)` - 创建并初始化桥接层

```python
from MCP_Module import create_mcp_bridge

# 加载所有机器人
bridge = create_mcp_bridge()

# 加载指定机器人
bridge = create_mcp_bridge(['Sim_2D', 'Go2_Quadruped'])
```

## 使用示例

### 基础使用

```python
from MCP_Module import create_mcp_bridge

# 创建桥接层
bridge = create_mcp_bridge(['Sim_2D'])

# 查看可用技能
skills = bridge.get_available_skills()
print(f"可用技能: {skills}")

# 执行技能
result = bridge.execute_skill('move_forward', distance=1.0, speed=0.2)

if result['success']:
    print(f"✅ 技能执行成功: {result['result']}")
else:
    print(f"❌ 技能执行失败: {result['error']}")
```

### 与 LLM 集成

```python
from MCP_Module import create_mcp_bridge
from LLM_Module import LLMAgent

# 初始化
bridge = create_mcp_bridge(['Sim_2D'])
llm = LLMAgent(api_key=os.getenv('Test_API_KEY'))

# 获取工具定义
tools = bridge.get_mcp_tools_definition()

# 定义执行函数
def execute_tool(function_name: str, function_args: dict) -> dict:
    return bridge.execute_skill(function_name, **function_args)

# 运行 LLM 流程
results = llm.run_pipeline(
    "向前走2米然后左转90度",
    tools=tools,
    execute_tool_fn=execute_tool
)
```

### 查询技能信息

```python
from MCP_Module import create_mcp_bridge

bridge = create_mcp_bridge(['Sim_2D', 'Go2_Quadruped'])

# 列出所有技能
skills = bridge.get_available_skills()
print(f"总共 {len(skills)} 个技能")

# 查看特定技能详情
for skill in skills[:3]:  # 显示前3个技能
    info = bridge.skill_registry.get_skill_info(skill)
    print(f"\n技能: {skill}")
    print(f"  描述: {info['description']}")
    print(f"  参数: {info['parameters']}")
    print(f"  来自机器人: {info['robot']}")
```

### 多机器人管理

```python
from MCP_Module import MCPBridge

bridge = MCPBridge()

# 加载多个机器人
robots = ['Sim_2D', 'Go2_Quadruped', '4Lun']
for robot in robots:
    bridge.load_robot(robot)

# 按机器人分组查看技能
from collections import defaultdict
skills_by_robot = defaultdict(list)

for skill_name, info in bridge.skill_registry.get_all_skills_info().items():
    skills_by_robot[info['robot']].append(skill_name)

for robot, skills in skills_by_robot.items():
    print(f"\n{robot}: {', '.join(skills)}")
```

## 技能命名规范

所有机器人技能函数必须遵循以下规范：

1. **函数命名**: 必须以 `skill_` 开头
   ```python
   def skill_move_forward(distance: float = 1.0, speed: float = 0.3):
       """向前移动"""
       pass
   ```

2. **类型注解**: 建议添加参数类型注解
   ```python
   def skill_rotate(angle: float, angular_speed: float = 0.5):
       pass
   ```

3. **文档字符串**: 提供清晰的技能描述
   ```python
   def skill_stop():
       """停止机器人运动"""
       return {'action': 'stop'}
   ```

4. **模块导出**: 在 `skills/__init__.py` 中导出
   ```python
   from .robot_skills import *
   ```

## 设计原则

1. **纯中间件**: 不包含业务逻辑，只负责技能管理
2. **动态加载**: 运行时动态发现和加载技能
3. **通用性**: 支持任意数量和类型的机器人
4. **标准化**: 提供统一的接口和数据格式
5. **可扩展**: 易于添加新的技能类型和功能

## 依赖

```
importlib    # 动态模块导入
inspect      # 函数签名和文档提取
yaml         # 配置文件解析
pathlib      # 路径处理
typing        # 类型注解
```

## 错误处理

```python
# 技能不存在
result = bridge.execute_skill('nonexistent_skill')
# {'success': False, 'skill': 'nonexistent_skill', 'error': "技能 'nonexistent_skill' 未找到"}

# 机器人加载失败
bridge.load_robot('InvalidRobot')
# ⚠️  [警告] 无法加载 InvalidRobot 的技能: No module named 'Robot_Module.InvalidRobot'
```

## 相关文档

- [LLM_Module README](../LLM_Module/README.md) - 大语言模型模块
- [VLM_Module README](../VLM_Module/README.md) - 视觉语言模型模块
- [Robot_Module README](../Robot_Module/README.md) - 机器人模块
- [主项目 README](../README.md) - 项目总览
