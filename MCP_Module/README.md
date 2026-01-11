# MCP_Module - MCP 中间件模块

## 功能

Model Context Protocol 中间件，连接 LLM/VLM 和机器人技能：
- 从 `Robot_Module` 加载和注册技能
- 提供 MCP 工具定义给 LLM
- 执行技能并返回结果

## 核心类

### MCPBridge

桥接层，连接 LLM/VLM 和底层技能：

```python
from MCP_Module import create_mcp_bridge

# 创建桥接并加载机器人
bridge = create_mcp_bridge(['Sim_2D', 'Go2_Quadruped'])

# 获取可用技能
skills = bridge.get_available_skills()
print(f"可用技能: {skills}")

# 获取 MCP 工具定义（用于 LLM）
tools = bridge.get_mcp_tools_definition()

# 执行技能
result = bridge.execute_skill('move_forward', distance=1.0, speed=0.3)
```

### SkillRegistry

技能注册表，管理所有可用技能：

```python
from MCP_Module import SkillRegistry

registry = SkillRegistry()

# 从机器人模块加载技能
registry.register_from_robot_module('Sim_2D')

# 获取技能
skill_func = registry.get_skill('move_forward')

# 获取技能信息
info = registry.get_skill_info('move_forward')
```

## 设计原则

- **纯中间件**: 不包含业务逻辑，只负责技能管理
- **动态加载**: 从 `Robot_Module` 动态发现和加载技能
- **通用性**: 支持任意数量的机器人

## 文件结构

```
MCP_Module/
├── __init__.py
└── mcp_bridge.py        # MCPBridge 和 SkillRegistry
```

## 依赖

- yaml
- importlib
- inspect
