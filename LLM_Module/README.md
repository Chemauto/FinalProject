# LLM_Module - 双层 LLM 核心模块

实现双层 LLM 架构，包含任务规划、任务执行、自适应重规划等核心功能。

## 📁 模块结构

```
LLM_Module/
├── __init__.py                 # 模块导出
├── llm_core.py                 # 兼容层 - LLMAgent
├── high_level_llm.py           # 高层LLM - 任务规划器
├── low_level_llm.py            # 低层LLM - 执行控制器
├── task_queue.py               # 任务队列管理
├── execution_monitor.py        # 执行监控器
├── adaptive_controller.py      # 自适应控制器
├── prompts/
│   └── planning_prompt_2d.yaml # 2D 机器人规划提示词
└── README.md                   # 本文档
```

## 🎯 功能特性

### 核心功能
- **双层 LLM 架构**: 规划层 + 执行层完全解耦
- **任务队列管理**: 状态跟踪、重试机制、动态插入
- **执行监控**: 超时/卡住/振荡/传感器失效检测
- **环境变化检测**: 自动检测并触发重新规划
- **多级重新规划**: 4级策略（参数/技能/任务/完全）
- **异步执行支持**: AdaptiveController 异步架构
- **向后兼容**: 旧代码无需修改

### 新版本特性 (v2.1)
- ✅ 模块化架构 - 清晰的职责分离
- ✅ 自适应控制 - 环境变化时自动重新规划
- ✅ 智能重试 - 可配置的重试机制
- ✅ 异常检测 - 多种异常类型检测（超时、卡住、振荡、传感器失效）
- ✅ 进度跟踪 - 实时任务进度显示
- ✅ **后台监控** - 执行时实时检测异常（新增）
- ✅ **属性转发** - 简化访问client/model（新增）

## 🧠 双层 LLM 架构

### 架构图

```
用户输入: "追击敌人"
    ↓
┌─────────────────────────────────────┐
│ 高层 LLM (任务规划)                  │
│ - 理解用户意图                       │
│ - 生成任务序列                       │
│ - 失败时重新规划                     │
└──────────────┬──────────────────────┘
               ↓
    任务队列: [Task1, Task2, ...]
               ↓
┌─────────────────────────────────────┐
│ 执行监控 (实时后台检测)              │
│ - 超时检测                          │
│ - 卡住检测                          │
│ - 振荡检测                          │
│ - 传感器失效检测                    │
│ - 环境变化检测                      │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│ 低层 LLM (任务执行)                  │
│ - 选择工具                          │
│ - 生成参数                          │
│ - 执行工具调用                      │
└──────────────┬──────────────────────┘
               ↓
    工具调用: robot_tool(params)
               ↓
        ROS2 Topic → Sim_Module
```

## 📝 使用方式

### 方式1: 兼容层（推荐用于现有代码）

```python
from LLM_Module import LLMAgent

# 旧代码无需修改，继续工作
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)

results = agent.run_pipeline(
    user_input="追击敌人",
    tools=tools,
    execute_tool_fn=your_execute_function
)
```

### 方式2: 启用自适应功能（推荐）

```python
from LLM_Module import LLMAgent

# 启用自适应控制 - 环境变化时自动重新规划
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml",
    enable_adaptive=True  # ← 只需添加这个！
)

results = agent.run_pipeline(
    user_input="追击敌人",
    tools=tools,
    execute_tool_fn=your_execute_function
)
```

### 方式3: 使用新架构（完全控制）

```python
from LLM_Module import AdaptiveController, HighLevelLLM, LowLevelLLM, ExecutionMonitor
import asyncio

# 初始化组件
high_level = HighLevelLLM(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)

low_level = LowLevelLLM(api_key="your_api_key")

monitor = ExecutionMonitor(
    monitoring_interval=0.1,    # 监控检查间隔（秒）
    timeout_threshold=30.0,     # 超时阈值（秒）
    stuck_threshold=5.0         # 卡住检测阈值（秒）
)

# 创建自适应控制器
controller = AdaptiveController(
    high_level_llm=high_level,
    low_level_llm=low_level,
    execution_monitor=monitor
)

# 运行（异步）
results = asyncio.run(
    controller.run(
        user_input="追击敌人",
        tools=tools,
        execute_tool_fn=your_execute_function,
        available_skills=["get_enemy_positions", "chase_enemy", "move_forward"],
        env_state={"position": {"x": 100, "y": 200}}  # 可选：环境状态
    )
)
```

### 方式4: 单独使用模块

```python
from LLM_Module import HighLevelLLM, LowLevelLLM, TaskQueue

# 使用任务队列
task_queue = TaskQueue()
task_queue.set_tasks([
    {"step": 1, "task": "获取敌人位置", "type": "感知"},
    {"step": 2, "task": "追击最近的敌人", "type": "追击"}
])

# 执行任务
while not task_queue.is_empty():
    task = task_queue.get_next_task()
    result = execute_task(task)

    if result["success"]:
        task_queue.mark_completed(task, result)
    else:
        task_queue.mark_failed(task, result["error"])

# 查看进度
progress = task_queue.get_progress()
print(f"进度: {progress['completed']}/{progress['total']}")

# 打印摘要
task_queue.print_summary()
```

## 🚀 新功能详解

### 1. 环境变化检测

```python
result = low_level.execute_task(
    task_description="追击敌人",
    tools=tools,
    execute_tool_fn=execute_fn,
    perception_data={  # 感知数据
        "environment_changed": True,  # 触发重新规划
        "target_missing": False,
        "new_obstacles": []
    }
)

if result["status"] == "requires_replanning":
    print("需要重新规划！")
```

### 2. 任务重试

```python
from LLM_Module import Task, TaskStatus

task = Task(step=1, task="移动到目标", type="move")

# 检查是否可以重试
if task.can_retry():
    task.increment_retry()
    # 重新执行...

# 设置最大重试次数
task.max_retries = 5
```

### 3. 插入新任务

```python
# 重新规划后插入新任务
new_tasks = [
    {"step": 1, "task": "搜索箱子", "type": "search"},
    {"step": 2, "task": "移动到箱子", "type": "move"}
]

# 插入到队列前端（优先执行）
task_queue.insert_tasks(new_tasks, at_front=True)
```

### 4. 异常检测

```python
from LLM_Module import ExecutionMonitor, AnomalyType

monitor = ExecutionMonitor(
    monitoring_interval=0.1,    # 监控检查间隔（秒）
    timeout_threshold=30.0,     # 超时阈值（秒）
    stuck_threshold=5.0         # 卡住检测阈值（秒）
)

# 检测异常
anomaly = monitor.detect_anomaly(
    current_state={
        "position": {"x": 100, "y": 200, "z": 0},
        "sensor_status": {"lidar": "ok", "camera": "ok"}
    },
    task={"task": "移动", "type": "move"}
)

if anomaly:
    print(f"检测到异常: {anomaly.type.value}")
    print(f"描述: {anomaly.description}")
    print(f"严重程度: {anomaly.severity}")
```

### 5. 多级重新规划

自适应控制器支持4级重新规划：

| 级别 | 名称 | 说明 | 触发场景 |
|------|------|------|----------|
| Level 1 | 参数调整 | 不改变任务，调整参数 | 超时、轻微卡住 |
| Level 2 | 技能替换 | 相同目标，不同方法 | 严重卡住、障碍物 |
| Level 3 | 任务重排 | 调整任务顺序 | 振荡行为 |
| Level 4 | 完全重新规划 | 重新生成整个计划 | 环境变化、传感器失效 |

### 6. 后台监控（v2.1 新增）

启用自适应模式后，系统会在后台自动监控任务执行：

```python
# 启用自适应后，监控会自动运行
agent = LLMAgent(enable_adaptive=True)

# 任务执行时，后台监控会定期检测：
# 1. 超时检测（默认30秒）
# 2. 卡住检测（位置5秒不变）
# 3. 振荡检测（来回移动）
# 4. 传感器失效检测

# 检测到异常后，自动触发重新规划
```

## 🔌 API 参考

### LLMAgent（兼容层）

```python
class LLMAgent:
    def __init__(self, api_key: str, base_url: str, prompt_path: str,
                 enable_adaptive: bool = False)
    def plan_tasks(self, user_input: str, tools: List[Dict]) -> List[Dict]
    def execute_single_task(self, task_description: str, tools: List[Dict],
                           execute_tool_fn: Callable, previous_result: Any = None) -> Dict
    def run_pipeline(self, user_input: str, tools: List[Dict],
                    execute_tool_fn: Callable) -> List[Dict]

    # 新增属性（v2.1）
    @property
    def client(self) -> OpenAI:
        """获取OpenAI客户端（转发到high_level_llm）"""

    @property
    def model(self) -> str:
        """获取模型名称（转发到high_level_llm）"""

    @property
    def planning_prompt_template(self) -> str:
        """获取规划提示词模板"""

    @planning_prompt_template.setter
    def planning_prompt_template(self, value: str):
        """设置规划提示词模板（会更新到high_level_llm）"""
```

### HighLevelLLM

```python
class HighLevelLLM:
    def __init__(self, api_key: str, base_url: str, model: str, prompt_path: str)

    def plan_tasks(self, user_input: str, available_skills: List[str],
                   env_state: Optional[Dict]) -> List[Dict]:
        """生成任务序列"""

    def replan_tasks(self, failed_task: Dict, env_state: Dict,
                     failure_reason: str, original_user_input: str,
                     available_skills: List[str]) -> List[Dict]:
        """任务失败时重新规划"""
```

### LowLevelLLM

```python
class LowLevelLLM:
    def __init__(self, api_key: str, base_url: str, model: str)

    def execute_task(self, task_description: str, tools: List[Dict],
                     execute_tool_fn: Callable, previous_result: Any = None,
                     perception_data: Optional[Dict] = None) -> Dict:
        """执行单个任务，支持环境变化检测"""
```

### TaskQueue

```python
class TaskQueue:
    def set_tasks(self, tasks_data: List[Dict])
    def get_next_task(self) -> Optional[Task]
    def mark_completed(self, task: Task, result: Dict)
    def mark_failed(self, task: Task, error: str)
    def insert_tasks(self, tasks_data: List[Dict], at_front: bool = True)
    def is_empty(self) -> bool
    def get_progress(self) -> Dict
    def print_summary(self)
```

### ExecutionMonitor

```python
class ExecutionMonitor:
    def __init__(self, monitoring_interval: float = 0.1,
                 timeout_threshold: float = 30.0,
                 stuck_threshold: float = 5.0)

    def detect_anomaly(self, current_state: Dict, task: Dict) -> Optional[Anomaly]:
        """检测异常"""

    def reset(self):
        """重置监控状态"""
```

### AdaptiveController

```python
class AdaptiveController:
    def __init__(self, high_level_llm: HighLevelLLM,
                 low_level_llm: LowLevelLLM,
                 execution_monitor: Optional[ExecutionMonitor] = None)

    async def run(self, user_input: str, tools: List[Dict],
                  execute_tool_fn: Callable, available_skills: List[str],
                  env_state: Optional[Dict] = None) -> List[Dict]:
        """运行自适应控制循环"""
```

## ⚙️ 配置说明

### 模型配置

```python
# 修改默认模型
agent = LLMAgent(
    api_key="your_api_key",
    base_url="https://dashscope.aliyuncs.com/compatible-mode/v1"
)
# 默认使用: qwen3-32b
```

支持的模型：
- `qwen3-32b` - 通义千问 3 32B（推荐）
- `qwen3-72b` - 通义千问 3 72B
- `qwen-turbo` - 通义千问 Turbo
- 其他兼容 OpenAI API 的模型

### 提示词配置

编辑 `prompts/planning_prompt_2d.yaml` 来自定义规划行为。

### 执行监控配置

```python
from LLM_Module import ExecutionMonitor

monitor = ExecutionMonitor(
    monitoring_interval=0.1,    # 监控检查间隔（秒）
    timeout_threshold=30.0,     # 超时阈值（秒）
    stuck_threshold=5.0         # 卡住检测阈值（秒）
)
```

## 💡 设计理念

### 为什么使用双层 LLM？

1. **职责分离**
   - 高层 LLM: 理解用户意图，进行高层规划
   - 低层 LLM: 将具体任务映射到工具调用

2. **提高准确性**
   - 规划 LLM 可以看到所有可用工具，做出全局最优规划
   - 执行 LLM 只需要关注单个任务，减少错误

3. **易于扩展**
   - 添加新工具只需更新可用工具列表
   - 提示词模板化，便于维护

### 为什么需要自适应控制？

1. **环境变化**: 机器人环境中，目标可能移动或消失
2. **执行失败**: 工具执行可能失败，需要重试或替代方案
3. **异常恢复**: 检测异常并自动恢复，提高鲁棒性

## 📊 迁移指南

### 从旧代码迁移

**旧代码（继续工作）：**
```python
from LLM_Module import LLMAgent
agent = LLMAgent(api_key="...", prompt_path="...")
results = agent.run_pipeline(user_input, tools, execute_fn)
```

**启用新功能（最简单）：**
```python
from LLM_Module import LLMAgent
agent = LLMAgent(api_key="...", prompt_path="...", enable_adaptive=True)
results = agent.run_pipeline(user_input, tools, execute_fn)
```

**使用新架构（完全控制）：**
```python
from LLM_Module import AdaptiveController, HighLevelLLM, LowLevelLLM
controller = AdaptiveController(high_level_llm=..., low_level_llm=...)
results = asyncio.run(controller.run(...))
```

## 🐛 调试技巧

### 查看任务队列状态

```python
task_queue.print_summary()
# 输出:
# 📊 [任务队列摘要]
#   总任务数: 3
#   已完成: 1 (1/3)
#   失败: 0
#   待执行: 2
#   进度: 33.3%
```

### 查看异常信息

```python
anomaly = monitor.detect_anomaly(current_state, task)
if anomaly:
    print(f"异常类型: {anomaly.type.value}")
    print(f"严重程度: {anomaly.severity}")
    print(f"数据: {anomaly.data}")
```

### 测试重新规划

```python
# 模拟环境变化
result = await controller.run(
    user_input="追击敌人",
    tools=tools,
    execute_tool_fn=execute_fn,
    available_skills=skills,
    env_state={
        "position": {"x": 100, "y": 200, "z": 0},
        "sensor_status": {"lidar": "ok", "camera": "ok"}
    }
)
```

## 🔗 相关模块

- `Interactive_Module/interactive.py` - 交互界面（已启用自适应模式）
- `Robot_Module/skill.py` - MCP 工具注册
- `ros_topic_comm.py` - ROS2 通信

## 📝 依赖

```
openai>=1.0.0  # OpenAI API
pyyaml>=6.0    # YAML 配置解析
asyncio         # 异步执行支持
```

## 📈 版本历史

### v2.1.0 (当前版本)
- ✅ **新增** 后台监控 - 任务执行时实时检测异常
- ✅ **新增** 属性转发 - 简化 client/model 访问
- ✅ **增强** 异常处理 - 根据异常类型智能选择重新规划级别
- ✅ **修复** 启用自适应模式 - `interactive.py` 默认启用

### v2.0.0
- ✅ 模块化架构重构
- ✅ 新增任务队列管理
- ✅ 新增执行监控器
- ✅ 新增自适应控制器
- ✅ 支持环境变化检测
- ✅ 支持多级重新规划
- ✅ 完全向后兼容

### v1.0.0
- 双层 LLM 架构
- 基础任务规划
- 工具调用支持

---

**智能规划，精确执行，自适应控制！** 🧠
