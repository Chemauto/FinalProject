# LLM Module - 详细技术文档

本文档详细介绍 LLM_Module 的架构设计、API 使用方法和扩展指南。

## 📋 目录

- [架构设计](#架构设计)
- [模块详解](#模块详解)
- [完整API参考](#完整api参考)
- [使用示例](#使用示例)
- [扩展指南](#扩展指南)

---

## 架构设计

### 设计理念

LLM_Module 实现了一个**双层 LLM 智能体架构**，模拟人类的认知过程：

1. **高层思考（High-Level LLM）**：像"大脑"一样理解全局，制定计划
2. **低层执行（Low-Level LLM）**：像"小脑"一样执行具体动作
3. **监控反馈（Execution Monitor）**：像"感知系统"一样监控状态
4. **自适应调整（Adaptive Controller）**：像"反思机制"一样应对变化

### 核心架构图

```
┌──────────────────────────────────────────────────────────────┐
│                    用户输入 (自然语言)                         │
│                    "追击最近的敌人"                          │
└────────────────────────┬─────────────────────────────────────┘
                         ↓
┌──────────────────────────────────────────────────────────────┐
│  High-Level LLM (规划器)                                     │
│  ┌────────────────────────────────────────────────────┐      │
│  │ 输入: 用户指令 + 可用技能列表                        │      │
│  │ 处理: 理解意图 → 分解任务 → 生成序列                │      │
│  │ 输出: [Task1, Task2, Task3, ...]                   │      │
│  └────────────────────────────────────────────────────┘      │
└────────────────────────┬─────────────────────────────────────┘
                         ↓
┌──────────────────────────────────────────────────────────────┐
│  Task Queue (任务队列)                                        │
│  ┌────────────────────────────────────────────────────┐      │
│  │ 状态管理: PENDING → IN_PROGRESS → COMPLETED         │      │
│  │ 支持操作: 获取、标记完成、标记失败、插入新任务       │      │
│  └────────────────────────────────────────────────────┘      │
└────────────────────────┬─────────────────────────────────────┘
                         ↓
              ┌──────────┴──────────┐
              ↓                     ↓
┌─────────────────────────┐  ┌──────────────────────────┐
│ Execution Monitor       │  │ Low-Level LLM (执行器)    │
│ ┌─────────────────────┐ │  │ ┌──────────────────────┐ │
│ │ 实时监控:           │ │  │ │ 输入: 单个任务描述    │ │
│ │ - 超时检测          │ │  │ │ 处理: 选择工具→生成参数│ │
│ │ - 卡住检测          │ │  │ │ 输出: 工具调用+结果    │ │
│ │ - 振荡检测          │ │  │ └──────────────────────┘ │
│ │ - 传感器失效        │ │  │                          │
│ └─────────────────────┘ │  │     ↓                   │
└──────────┬──────────────┘  │     │ Tool: chase_enemy   │
           │                 │     │ Params: {...}        │
           │ 检测到异常?     │                          │
           ↓                 └──────────┬───────────────┘
    ┌──────────────────────────────────┘
    ↓
┌──────────────────────────────────────────────────────────────┐
│  Adaptive Controller (自适应控制器)                           │
│  ┌────────────────────────────────────────────────────┐      │
│  │ 决策: 异常类型 → 重新规划级别                        │      │
│  │ 策略:                                             │      │
│  │   Level 1: 参数调整 (超时、轻微卡住)                 │      │
│  │   Level 2: 技能替换 (严重卡住、障碍物)               │      │
│  │   Level 3: 任务重排 (振荡行为)                      │      │
│  │   Level 4: 完全重新规划 (环境变化、传感器失效)       │      │
│  └────────────────────────────────────────────────────┘      │
└────────────────────────┬─────────────────────────────────────┘
                         ↓
              重新规划或继续执行
                         ↓
                  Robot_Module (技能调用)
```

---

## 模块详解

### 1. high_level_llm.py - 高层 LLM 规划器

**职责**：理解用户意图，结合可用技能，生成分解的任务序列

**核心方法**：

```python
class HighLevelLLM:
    def __init__(self, api_key: str, base_url: str, model: str, prompt_path: str):
        """初始化高层LLM"""
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = model
        self.prompt_template = self._load_prompt_template(prompt_path)

    def plan_tasks(self, user_input: str, available_skills: List[str],
                   env_state: Optional[Dict] = None) -> List[Dict]:
        """
        生成任务序列

        Args:
            user_input: 用户自然语言指令，如 "追击敌人"
            available_skills: 可用技能列表，如 ["get_enemy_positions", "chase_enemy"]
            env_state: 当前环境状态（可选），如 {"position": {"x": 100, "y": 200}}

        Returns:
            任务列表，格式:
            [
                {"step": 1, "task": "获取敌人位置", "type": "感知"},
                {"step": 2, "task": "追击最近的敌人", "type": "追击"}
            ]
        """
        # 1. 构建提示词（包含可用技能）
        prompt = self._build_prompt(user_input, available_skills, env_state)

        # 2. 调用LLM生成JSON格式的任务序列
        completion = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": "你是任务规划助手"},
                {"role": "user", "content": prompt}
            ]
        )

        # 3. 解析JSON响应
        plan = json.loads(completion.choices[0].message.content)
        tasks = plan.get("tasks", [])

        return tasks

    def replan_tasks(self, failed_task: Dict, env_state: Dict,
                     failure_reason: str, original_user_input: str,
                     available_skills: List[str]) -> List[Dict]:
        """
        失败后重新规划

        使用场景：
        - 任务执行失败
        - 环境发生变化
        - 检测到异常

        策略：
        - 分析失败原因
        - 评估环境状态
        - 生成替代方案
        """
```

**提示词模板**：

```yaml
# prompts/planning_prompt_2d.yaml
system_prompt: |
  你是一个机器人任务规划助手。

  机器人配置:
  {robot_config}

  可用技能:
  {available_skills}

  注意:
  - 输出必须是有效的JSON格式
  - 将复杂指令分解为简单任务
  - 只使用可用技能列表中的技能

prompt: |
  用户输入: {user_input}

  请将上述指令分解为子任务序列。

  输出格式（JSON）：
  {
    "tasks": [
      {"step": 1, "task": "子任务描述1", "type": "动作类型"},
      {"step": 2, "task": "子任务描述2", "type": "动作类型"}
    ],
    "summary": "整体任务概述"
  }
```

---

### 2. low_level_llm.py - 低层 LLM 执行器

**职责**：将具体任务映射到工具调用，生成参数并执行

**核心方法**：

```python
class LowLevelLLM:
    def __init__(self, api_key: str, base_url: str, model: str):
        """初始化低层LLM"""
        self.client = OpenAI(api_key=api_key, base_url=base_url)
        self.model = model

    def execute_task(self, task_description: str, tools: List[Dict],
                     execute_tool_fn: Callable, previous_result: Any = None,
                     perception_data: Optional[Dict] = None) -> Dict:
        """
        执行单个任务

        Args:
            task_description: 任务描述，如 "追击最近的敌人"
            tools: 工具列表（OpenAI function calling格式）
            execute_tool_fn: 工具执行函数
            previous_result: 上一步的执行结果
            perception_data: 感知数据（用于环境变化检测）

        Returns:
            执行结果:
            {
                "status": "success",  # success/failed/requires_replanning
                "action": "chase_enemy",
                "task": "追击最近的敌人",
                "result": {...}
            }
        """
        # 1. 构建系统提示词
        system_prompt = self._build_system_prompt(task_description, previous_result)

        # 2. 调用LLM（使用function calling）
        completion = self.client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": f"执行任务: {task_description}"}
            ],
            tools=tools,  # OpenAI function calling
            tool_choice="auto"
        )

        # 3. 提取工具调用
        tool_calls = completion.choices[0].message.tool_calls
        if not tool_calls:
            return {"status": "failed", "error": "No tool called"}

        tool_call = tool_calls[0]
        function_name = tool_call.function.name
        function_args = json.loads(tool_call.function.arguments)

        # 4. 执行工具
        result = execute_tool_fn(function_name, function_args)

        # 5. 返回结果
        return {
            "status": "success",
            "action": function_name,
            "task": task_description,
            "result": result
        }
```

**系统提示词示例**：

```python
system_prompt = f"""
你是机器人执行控制器，负责将任务转换为工具调用。

【当前任务】
{task_description}

【上一步结果】
{previous_result or '无'}

【可用工具】
{tools}

【执行规则】
1. 理解任务描述，选择最合适的工具
2. 根据任务和上一步结果，生成工具参数
3. 如果任务涉及"追击"，必须先调用 get_enemy_positions() 获取位置
4. 将位置传递给 chase_enemy(positions=...)

【重要】
- 如果上一步结果是敌人位置JSON，直接使用
- 不要重复调用 get_enemy_positions()
- 确保 chase_enemy 的参数是有效的JSON字符串
"""
```

---

### 3. task_queue.py - 任务队列管理

**职责**：管理任务状态，支持重试机制，动态插入任务

**核心类和方法**：

```python
class TaskStatus(Enum):
    """任务状态"""
    PENDING = "pending"          # 待执行
    IN_PROGRESS = "in_progress"  # 执行中
    COMPLETED = "completed"      # 已完成
    FAILED = "failed"            # 失败
    SKIPPED = "skipped"          # 已跳过

@dataclass
class Task:
    """任务数据类"""
    step: int                    # 步骤编号
    task: str                    # 任务描述
    type: str                    # 任务类型
    status: TaskStatus = TaskStatus.PENDING
    retry_count: int = 0         # 当前重试次数
    max_retries: int = 3         # 最大重试次数
    result: Optional[Dict] = None # 执行结果

    def can_retry(self) -> bool:
        """是否可以重试"""
        return self.retry_count < self.max_retries

class TaskQueue:
    """任务队列"""

    def set_tasks(self, tasks_data: List[Dict]):
        """
        设置初始任务列表

        Args:
            tasks_data: 任务数据列表，格式:
                [
                    {"step": 1, "task": "获取敌人位置", "type": "感知"},
                    {"step": 2, "task": "追击最近的敌人", "type": "追击"}
                ]
        """
        self.tasks = [Task(**data) for data in tasks_data]

    def get_next_task(self) -> Optional[Task]:
        """获取下一个待执行的任务"""
        for task in self.tasks:
            if task.status == TaskStatus.PENDING:
                task.status = TaskStatus.IN_PROGRESS
                return task
        return None

    def mark_completed(self, task: Task, result: Dict):
        """标记任务为已完成"""
        task.status = TaskStatus.COMPLETED
        task.result = result

    def mark_failed(self, task: Task, error: str):
        """标记任务为失败"""
        task.retry_count += 1
        if task.can_retry():
            task.status = TaskStatus.PENDING  # 可以重试，重置为待执行
        else:
            task.status = TaskStatus.FAILED    # 达到最大重试次数，彻底失败

    def insert_tasks(self, tasks_data: List[Dict], at_front: bool = True):
        """
        动态插入新任务（用于重新规划）

        Args:
            tasks_data: 新任务列表
            at_front: 是否插入到队列前端（优先执行）
        """
        new_tasks = [Task(**data) for data in tasks_data]

        # 重新编号后续任务
        for task in self.tasks:
            task.step += len(new_tasks)

        if at_front:
            self.tasks = new_tasks + self.tasks
        else:
            self.tasks.extend(new_tasks)

    def get_progress(self) -> Dict:
        """
        获取进度信息

        Returns:
            {
                "total": 5,
                "completed": 3,
                "failed": 0,
                "pending": 2,
                "progress_percent": 60.0
            }
        """
```

**使用示例**：

```python
# 创建任务队列
queue = TaskQueue()

# 设置任务
queue.set_tasks([
    {"step": 1, "task": "获取敌人位置", "type": "感知"},
    {"step": 2, "task": "追击最近的敌人", "type": "追击"}
])

# 执行循环
while not queue.is_empty():
    task = queue.get_next_task()
    result = execute_task(task)

    if result["success"]:
        queue.mark_completed(task, result)
    else:
        queue.mark_failed(task, result["error"])
        if not task.can_retry():
            # 重试失败，触发重新规划
            new_tasks = replan(...)
            queue.insert_tasks(new_tasks, at_front=True)
```

---

### 4. execution_monitor.py - 执行监控器

**职责**：实时监控任务执行，检测异常，触发重新规划

**核心类和方法**：

```python
class AnomalyType(Enum):
    """异常类型"""
    TIMEOUT = "timeout"                      # 超时
    STUCK = "stuck"                          # 卡住
    OSCILLATION = "oscillation"              # 振荡
    ENVIRONMENT_CHANGE = "environment_change" # 环境变化
    SENSOR_FAILURE = "sensor_failure"        # 传感器失效
    UNKNOWN = "unknown"

@dataclass
class Anomaly:
    """异常数据"""
    type: AnomalyType
    description: str
    severity: str  # low, medium, high
    data: Optional[Dict] = None

class ExecutionMonitor:
    """执行监控器"""

    def __init__(self,
                 monitoring_interval: float = 0.1,    # 监控间隔（秒）
                 timeout_threshold: float = 30.0,     # 超时阈值（秒）
                 stuck_threshold: float = 5.0):       # 卡住阈值（秒）
        """初始化监控器"""

    def detect_anomaly(self, current_state: Dict, task: Dict) -> Optional[Anomaly]:
        """
        检测异常

        TODO: 需要添加以下检测逻辑：

        1. 超时检测
        ```python
        if elapsed_time > self.timeout_threshold:
            return Anomaly(
                type=AnomalyType.TIMEOUT,
                description=f"任务执行超时（{elapsed_time:.1f}秒）",
                severity="high"
            )
        ```

        2. 卡住检测（位置不变）
        ```python
        if position_unchanged_duration > self.stuck_threshold:
            return Anomaly(
                type=AnomalyType.STUCK,
                description=f"机器人卡住（{duration:.1f}秒未移动）",
                severity="medium"
            )
        ```

        3. 振荡检测（来回移动）
        ```python
        if is_oscillating(position_history):
            return Anomaly(
                type=AnomalyType.OSCILLATION,
                description="检测到振荡行为",
                severity="medium"
            )
        ```

        4. 传感器失效
        ```python
        if sensor_status == "failed":
            return Anomaly(
                type=AnomalyType.SENSOR_FAILURE,
                description="传感器失效",
                severity="high"
            )
        ```

        5. 环境变化
        ```python
        if environment_changed(current_state, previous_state):
            return Anomaly(
                type=AnomalyType.ENVIRONMENT_CHANGE,
                description="检测到环境变化",
                severity="high"
            )
        ```
        """
        # 当前框架：暂不检测异常，返回None
        return None

    def reset(self):
        """重置监控状态"""
        self.execution_start_time = None
        self.last_position = None
        self.position_history = []
```

---

### 5. adaptive_controller.py - 自适应控制器

**职责**：协调规划、执行、监控，实现自适应重规划

**核心流程**：

```python
class AdaptiveController:
    """自适应控制器"""

    def __init__(self, high_level_llm, low_level_llm, execution_monitor):
        """初始化控制器"""
        self.high_level_llm = high_level_llm
        self.low_level_llm = low_level_llm
        self.execution_monitor = execution_monitor
        self.task_queue = TaskQueue()
        self.replan_count = 0
        self.max_replans = 3  # 最大重新规划次数

    async def run(self, user_input: str, tools: List[Dict],
                  execute_tool_fn: Callable, available_skills: List[str],
                  env_state: Optional[Dict] = None) -> List[Dict]:
        """
        运行完整的自适应控制流程

        流程：
        1. 初始规划 → 生成任务序列
        2. 执行循环：
           a. 取出任务
           b. 监控执行（后台检测异常）
           c. 处理结果（成功/失败/异常）
           d. 如果需要，触发重新规划
        3. 返回所有结果
        """
        # 阶段1: 初始规划
        tasks = self.high_level_llm.plan_tasks(
            user_input=user_input,
            available_skills=available_skills,
            env_state=env_state
        )
        self.task_queue.set_tasks(tasks)

        # 阶段2: 执行循环
        results = []
        while not self.task_queue.is_empty() and self.replan_count < self.max_replans:
            task = self.task_queue.get_next_task()

            # 执行任务（带监控）
            result = await self.execute_with_monitoring(
                task=task,
                tools=tools,
                execute_tool_fn=execute_tool_fn,
                env_state=env_state
            )
            results.append(result)

            # 处理结果（可能触发重新规划）
            await self.handle_execution_result(
                task=task,
                result=result,
                env_state=env_state,
                available_skills=available_skills
            )

        return results

    async def execute_with_monitoring(self, task, tools, execute_tool_fn, env_state):
        """
        带监控的任务执行

        TODO: 后续添加后台监控逻辑

        当前实现：直接执行任务，暂不启动后台监控
        """
        self.execution_monitor.reset()

        # 直接执行任务
        result = self.low_level_llm.execute_task(
            task_description=task.task,
            tools=tools,
            execute_tool_fn=execute_tool_fn,
            previous_result=self._get_previous_result()
        )

        return result

    async def handle_execution_result(self, task, result, env_state, available_skills):
        """
        处理执行结果

        决策逻辑：
        1. 成功 → 标记完成
        2. 失败 → 检查是否需要重新规划
        3. 检测到异常 → 触发重新规划

        TODO: 后续添加自动重试和重新规划逻辑
        """
        status = result.get("status")

        if status == "success":
            self.task_queue.mark_completed(task, result)

        elif status == "requires_replanning":
            # 低层LLM检测到环境变化
            await self.trigger_replanning(
                task=task,
                result=result,
                env_state=env_state,
                available_skills=available_skills,
                level=ReplanLevel.FULL_REPLAN
            )

        else:
            # 任务失败
            self.task_queue.mark_failed(task, result.get("error"))

    async def trigger_replanning(self, task, result, env_state, available_skills, level):
        """
        触发重新规划

        根据级别选择策略：
        - Level 1 (PARAMETER_ADJUSTMENT): 调整参数
        - Level 2 (SKILL_REPLACEMENT): 替换技能
        - Level 3 (TASK_REORDER): 重排任务
        - Level 4 (FULL_REPLAN): 完全重新规划
        """
        self.replan_count += 1
        print(f"🔄 [重新规划] 第 {self.replan_count} 次 (级别: {level.name})")

        # 调用高层LLM重新规划
        new_tasks = self.high_level_llm.replan_tasks(
            failed_task={"task": task.task, "type": task.type},
            env_state=env_state,
            failure_reason=result.get("error", "Unknown"),
            original_user_input=self.original_user_input,
            available_skills=available_skills
        )

        if new_tasks:
            self.task_queue.insert_tasks(new_tasks, at_front=True)
```

**重新规划级别说明**：

```python
class ReplanLevel(Enum):
    """重新规划级别"""
    PARAMETER_ADJUSTMENT = 1  # 参数调整
    SKILL_REPLACEMENT = 2     # 技能替换
    TASK_REORDER = 3          # 任务重排
    FULL_REPLAN = 4           # 完全重新规划
```

**级别选择策略**：

| 异常类型 | 级别 | 说明 | 示例 |
|---------|------|------|------|
| 超时 | Level 1 | 调整参数（增大速度） | `move_forward(speed=0.3)` → `move_forward(speed=0.5)` |
| 轻度卡住 | Level 1 | 调整参数 | `turn(angle=90)` → `turn(angle=95)` |
| 严重卡住 | Level 2 | 技能替换 | `move_forward` → `move_backward` (后退解除卡住) |
| 障碍物 | Level 2 | 技能替换 | `move_forward` → `turn` + `move_forward` |
| 振荡 | Level 3 | 任务重排 | 任务A→B → 任务B→A |
| 环境变化 | Level 4 | 完全重新规划 | "追击敌人" → "搜索敌人"→"追击" |
| 传感器失效 | Level 4 | 完全重新规划 | 改用其他传感器或方法 |

---

### 6. llm_core.py - LLMAgent 兼容层

**职责**：提供向后兼容的接口，内部使用新的模块化架构

**核心类**：

```python
class LLMAgent:
    """LLM Agent（兼容层）"""

    def __init__(self, api_key: str, base_url: str, prompt_path: str,
                 enable_adaptive: bool = False):
        """
        初始化LLM Agent

        Args:
            enable_adaptive: 是否启用自适应控制
        """
        # 创建新的模块化架构
        self.high_level_llm = HighLevelLLM(api_key, base_url, prompt_path)
        self.low_level_llm = LowLevelLLM(api_key, base_url)

        if enable_adaptive:
            from .execution_monitor import ExecutionMonitor
            from .adaptive_controller import AdaptiveController

            self.adaptive_controller = AdaptiveController(
                high_level_llm=self.high_level_llm,
                low_level_llm=self.low_level_llm,
                execution_monitor=ExecutionMonitor()
            )
        else:
            self.adaptive_controller = None

    def run_pipeline(self, user_input: str, tools: List[Dict],
                    execute_tool_fn: Callable) -> List[Dict]:
        """
        运行双层LLM流程

        根据是否启用自适应，选择不同的执行路径：
        - enable_adaptive=True: 使用 AdaptiveController（异步）
        - enable_adaptive=False: 使用同步流程（向后兼容）
        """
        if self.enable_adaptive and self.adaptive_controller:
            # 使用异步自适应控制器
            loop = asyncio.get_event_loop()
            results = loop.run_until_complete(
                self.adaptive_controller.run(
                    user_input=user_input,
                    tools=tools,
                    execute_tool_fn=execute_tool_fn,
                    available_skills=[...]
                )
            )
            return results
        else:
            # 使用同步流程（向后兼容）
            tasks = self.plan_tasks(user_input, tools)
            results = []
            for task in tasks:
                result = self.execute_single_task(task["task"], tools, execute_tool_fn)
                results.append(result)
            return results
```

---

## 完整API参考

### HighLevelLLM

```python
class HighLevelLLM:
    def __init__(self, api_key: str, base_url: str, model: str, prompt_path: str)

    def plan_tasks(self, user_input: str, available_skills: List[str],
                   env_state: Optional[Dict] = None) -> List[Dict]:
        """生成任务序列"""

    def replan_tasks(self, failed_task: Dict, env_state: Dict,
                     failure_reason: str, original_user_input: str,
                     available_skills: List[str]) -> List[Dict]:
        """任务失败时重新规划"""

    def validate_plan(self, tasks: List[Dict]) -> bool:
        """验证生成的任务计划是否有效"""
```

### LowLevelLLM

```python
class LowLevelLLM:
    def __init__(self, api_key: str, base_url: str, model: str)

    def execute_task(self, task_description: str, tools: List[Dict],
                     execute_tool_fn: Callable, previous_result: Any = None,
                     perception_data: Optional[Dict] = None) -> Dict:
        """执行单个任务"""
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

---

## 使用示例

### 示例1：基础使用（非自适应模式）

```python
from LLM_Module import LLMAgent

# 创建Agent
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)

# 定义工具
tools = [
    {
        "type": "function",
        "function": {
            "name": "move_forward",
            "description": "向前移动指定距离",
            "parameters": {
                "type": "object",
                "properties": {
                    "distance": {"type": "number"},
                    "speed": {"type": "number"}
                }
            }
        }
    },
    # ... 更多工具
]

# 定义执行函数
def execute_tool(function_name: str, parameters: dict):
    """执行工具调用"""
    if function_name == "move_forward":
        return {"result": f"已前进{parameters['distance']}米", "delay": 3.0}
    # ... 其他工具

# 运行
results = agent.run_pipeline(
    user_input="前进1米然后左转90度",
    tools=tools,
    execute_tool_fn=execute_tool
)

# 结果
for result in results:
    print(f"任务: {result['task']}")
    print(f"状态: {result['success']}")
    print(f"结果: {result.get('result')}")
```

### 示例2：启用自适应模式

```python
from LLM_Module import LLMAgent
import asyncio

# 创建Agent（启用自适应）
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml",
    enable_adaptive=True  # ← 启用自适应
)

# 提供环境状态
env_state = {
    "position": {"x": 100, "y": 200},
    "sensor_status": {"lidar": "ok", "camera": "ok"}
}

# 运行（异步）
results = agent.run_pipeline(
    user_input="追击敌人",
    tools=tools,
    execute_tool_fn=execute_tool
)

# 如果检测到异常，会自动重新规划
# 如果任务失败，会自动重试或重新规划
```

### 示例3：使用新架构（完全控制）

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
    monitoring_interval=0.1,
    timeout_threshold=30.0,
    stuck_threshold=5.0
)

# 创建控制器
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
        execute_tool_fn=execute_tool,
        available_skills=["get_enemy_positions", "chase_enemy"],
        env_state={"position": {"x": 100, "y": 200}}
    )
)
```

### 示例4：单独使用模块

```python
from LLM_Module import HighLevelLLM, TaskQueue

# 高层规划
high_level = HighLevelLLM(api_key="...", prompt_path="...")

tasks = high_level.plan_tasks(
    user_input="追击敌人",
    available_skills=["get_enemy_positions", "chase_enemy"],
    env_state={"position": {"x": 100, "y": 200}}
)

# 任务队列管理
queue = TaskQueue()
queue.set_tasks(tasks)

# 执行循环
while not queue.is_empty():
    task = queue.get_next_task()
    result = execute_task(task)

    if result["success"]:
        queue.mark_completed(task, result)
    else:
        queue.mark_failed(task, result["error"])

    # 查看进度
    progress = queue.get_progress()
    print(f"进度: {progress['completed']}/{progress['total']}")
```

---

## 扩展指南

### 1. 添加新的异常检测

在 `execution_monitor.py` 的 `detect_anomaly()` 方法中添加：

```python
def detect_anomaly(self, current_state: Dict, task: Dict) -> Optional[Anomaly]:
    """检测异常"""

    # 示例1: 超时检测
    if self.execution_start_time:
        elapsed = time.time() - self.execution_start_time
        if elapsed > self.timeout_threshold:
            return Anomaly(
                type=AnomalyType.TIMEOUT,
                description=f"任务超时（{elapsed:.1f}秒）",
                severity="high"
            )

    # 示例2: 卡住检测
    if current_state and "position" in current_state:
        current_position = current_state["position"]
        if self.last_position:
            distance = self._calculate_distance(current_position, self.last_position)
            if distance < 0.01:  # 位置几乎不变
                stuck_duration = time.time() - self.last_position_update_time
                if stuck_duration > self.stuck_threshold:
                    return Anomaly(
                        type=AnomalyType.STUCK,
                        description=f"卡住（{stuck_duration:.1f}秒）",
                        severity="medium"
                    )

    # ... 添加更多检测逻辑

    return None
```

### 2. 自定义重新规划策略

在 `adaptive_controller.py` 中修改级别选择逻辑：

```python
def _determine_replan_level(self, result: Dict) -> ReplanLevel:
    """根据错误类型选择重新规划级别"""

    error = result.get("error", "").lower()

    # 自定义策略
    if "timeout" in error:
        return ReplanLevel.PARAMETER_ADJUSTMENT  # 调整参数
    elif "obstacle" in error:
        return ReplanLevel.SKILL_REPLACEMENT    # 替换技能
    elif "environment" in error:
        return ReplanLevel.FULL_REPLAN          # 完全重新规划
    else:
        return ReplanLevel.PARAMETER_ADJUSTMENT
```

### 3. 集成VLM感知

在 `low_level_llm.py` 中添加VLM支持：

```python
def execute_task(self, task_description: str, tools: List[Dict],
                 execute_tool_fn: Callable, previous_result: Any = None,
                 perception_data: Optional[Dict] = None):
    """执行任务"""

    # 如果提供了感知数据，检查环境变化
    if perception_data and perception_data.get("environment_changed"):
        return {
            "status": "requires_replanning",
            "reason": "environment_changed",
            "task": task_description
        }

    # 正常执行...
```

### 4. 添加新的重新规划级别

```python
class ReplanLevel(Enum):
    PARAMETER_ADJUSTMENT = 1
    SKILL_REPLACEMENT = 2
    TASK_REORDER = 3
    FULL_REPLAN = 4
    # 添加新级别
    HUMAN_INTERVENTION = 5  # 请求人工干预
```

---

## 最佳实践

### 1. 提示词设计

- 明确指定输出格式（JSON）
- 提供清晰的示例
- 说明可用技能的限制
- 包含错误处理指导

### 2. 错误处理

- 所有LLM调用都包装在 try-except 中
- 提供有意义的错误信息
- 实现回退机制（如默认行为）

### 3. 性能优化

- 缓存LLM客户端连接
- 异步执行工具调用
- 避免不必要的LLM调用

### 4. 调试技巧

- 打印中间结果
- 验证JSON格式
- 检查工具参数

---

## 常见问题

### Q1: 如何调试LLM生成的JSON？

```python
try:
    plan = json.loads(response_text)
except json.JSONDecodeError as e:
    print(f"JSON解析失败: {e}")
    print(f"原始响应: {response_text}")
    # 使用默认计划或回退策略
```

### Q2: 如何处理工具参数错误？

```python
def execute_tool(function_name: str, parameters: dict):
    try:
        result = skill_func(**parameters)
        return {"success": True, "result": result}
    except TypeError as e:
        # 参数错误，让LLM重新生成
        return {"success": False, "error": str(e)}
```

### Q3: 如何避免无限重新规划？

```python
# 设置最大重新规划次数
self.max_replans = 3

# 在执行循环中检查
while not queue.is_empty() and self.replan_count < self.max_replans:
    # ...
```

---

## 总结

LLM_Module 提供了一个完整的双层 LLM 智能体框架：

✅ **已实现**：
- 任务分解（High-Level LLM）
- 任务执行（Low-Level LLM）
- 任务队列管理
- 执行监控框架
- 自适应控制框架

⚠️ **待完善**：
- 具体的异常检测逻辑
- 后台监控任务启动
- 自动重试和重新规划触发
- VLM集成

🎯 **设计目标**：
- 职责分离（规划 vs 执行）
- 向后兼容（旧代码无需修改）
- 可扩展（预留扩展接口）
- 模块化（独立可测试）

---

**详细文档，深入理解！** 🧠
