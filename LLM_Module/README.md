# LLM_Module - 双层 LLM 核心模块

实现双层 LLM 架构，支持任务规划、执行控制和自适应重规划。

## 📁 模块结构

```
LLM_Module/
├── __init__.py                 # 模块导出
├── llm_core.py                 # LLMAgent 兼容层
├── high_level_llm.py           # 高层LLM - 任务规划器
├── low_level_llm.py            # 低层LLM - 执行控制器
├── task_queue.py               # 任务队列管理
├── execution_monitor.py        # 执行监控器
├── adaptive_controller.py      # 自适应控制器
├── prompts/
│   └── planning_prompt_2d.yaml # 规划提示词模板
├── README.md                   # 本文档（简洁版）
└── README4AI.md                # 详细技术文档
```

## 🎯 核心功能

| 功能 | 模块 | 状态 |
|------|------|------|
| **任务分解** | `high_level_llm.py` | ✅ 完成 |
| **任务执行** | `low_level_llm.py` | ✅ 完成 |
| **任务队列管理** | `task_queue.py` | ✅ 完成 |
| **执行监控** | `execution_monitor.py` | ⚠️ 框架完成，检测逻辑待添加 |
| **自适应重规划** | `adaptive_controller.py` | ⚠️ 框架完成，触发逻辑待添加 |

## 🔄 完整流程

```
用户输入指令
    ↓
┌─────────────────────────────────────┐
│ high_level_llm.py (高层LLM)          │
│ - 理解用户意图                       │
│ - 结合可用技能列表                   │
│ - 分解为任务序列                     │
└──────────────┬──────────────────────┘
               ↓
    [任务1, 任务2, 任务3, ...]
               ↓
┌─────────────────────────────────────┐
│ task_queue.py (任务队列)             │
│ - 管理任务状态                       │
│ - 支持重试机制                       │
│ - 动态插入新任务                     │
└──────────────┬──────────────────────┘
               ↓
    逐个取出任务执行
               ↓
┌─────────────────────────────────────┐
│ execution_monitor.py (监控器)        │
│ - 检测异常（框架）                   │
│ - 支持超时/卡住/振荡检测（待添加）    │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│ low_level_llm.py (低层LLM)           │
│ - 选择合适工具                       │
│ - 生成工具参数                       │
│ - 调用 Robot_Module 执行             │
└──────────────┬──────────────────────┘
               ↓
        Robot_Module (技能调用)
```

## 📖 快速开始

### 基础使用（非自适应模式）

```python
from LLM_Module import LLMAgent

agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)

results = agent.run_pipeline(
    user_input="前进1米然后左转90度",
    tools=available_tools,
    execute_tool_fn=execute_function
)
```

### 启用自适应控制

```python
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml",
    enable_adaptive=True  # 启用自适应
)
```

## 📝 各文件说明

| 文件 | 行数 | 主要功能 |
|------|------|----------|
| `llm_core.py` | 298 | LLMAgent兼容层，向后支持旧代码 |
| `high_level_llm.py` | 286 | 高层LLM，理解意图并分解任务 |
| `low_level_llm.py` | 282 | 低层LLM，选择工具并执行 |
| `task_queue.py` | 254 | 任务队列，状态管理和重试 |
| `execution_monitor.py` | 148 | 监控器，检测执行异常 |
| `adaptive_controller.py` | 383 | 自适应控制器，协调规划和执行 |

## 🔧 待完善功能

### 1. 执行监控 (`execution_monitor.py`)

需要在 `detect_anomaly()` 方法中添加：
- ✅ 超时检测
- ✅ 卡住检测（位置不变）
- ✅ 振荡检测（来回移动）
- ✅ 传感器失效检测
- ✅ 环境变化检测

### 2. 自适应控制 (`adaptive_controller.py`)

需要完善：
- ✅ 后台监控任务启动
- ✅ 异常检测触发重新规划
- ✅ 重新规划级别选择逻辑
- ✅ 失败后自动重试

## 💡 设计特点

1. **职责分离** - 规划和执行完全解耦
2. **向后兼容** - 旧代码无需修改
3. **可扩展** - 预留监控和重规划接口
4. **模块化** - 每个模块独立可测试

## 📊 数据流

```python
# 用户输入
user_input = "追击敌人"

# 1. 高层LLM分解任务
tasks = high_level_llm.plan_tasks(
    user_input=user_input,
    available_skills=["get_enemy_positions", "chase_enemy"]
)
# 返回: [
#   {"step": 1, "task": "获取敌人位置", "type": "感知"},
#   {"step": 2, "task": "追击最近的敌人", "type": "追击"}
# ]

# 2. 低层LLM执行任务
for task in tasks:
    result = low_level_llm.execute_task(
        task_description=task["task"],
        tools=available_tools,
        execute_tool_fn=execute_tool
    )
    # 返回: {"status": "success", "action": "get_enemy_positions", ...}
```

## 🔗 相关模块

- `Interactive_Module/interactive.py` - 交互界面（已启用自适应）
- `Robot_Module/skill.py` - MCP 工具注册
- `VLM_Module/vlm_core_remote.py` - 视觉语言模型

## 📚 详细文档

查看 **[README4AI.md](README4AI.md)** 了解：
- 详细的架构设计
- 完整的 API 参考
- 使用示例和最佳实践
- 扩展指南

## 📈 版本历史

### v2.1.0 (当前)
- ✅ 简化监控器，移除具体检测逻辑（待后续添加）
- ✅ 简化自适应控制器，保留框架
- ✅ 修复统计逻辑兼容性问题
- ✅ 代码注释标记 TODO 扩展点

### v2.0.0
- ✅ 模块化架构重构
- ✅ 双层 LLM 分离
- ✅ 任务队列管理
- ✅ 执行监控框架
- ✅ 自适应控制框架

---

**简洁文档，快速上手！** 🚀
