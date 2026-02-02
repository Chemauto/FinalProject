# LLM Module - 详细技术文档 (v3.0.0)

本文档详细介绍 LLM_Module v3.0.0 的架构设计、API 使用方法和扩展指南。

## 📋 目录

- [架构设计](#架构设计)
- [模块详解](#模块详解)
- [完整API参考](#完整api参考)
- [使用示例](#使用示例)
- [异常检测系统](#异常检测系统)
- [自适应控制](#自适应控制)
- [测试](#测试)

---

## 架构设计

### 设计理念

LLM_Module v3.0.0 实现了一个**支持视觉理解和自适应控制的双层 LLM 智能体架构**，模拟人类的认知过程：

1. **视觉感知（VLM Core）**：像"眼睛"一样理解环境图像
2. **高层思考（High-Level LLM）**：像"大脑"一样理解全局，制定计划
3. **低层执行（Low-Level LLM）**：像"小脑"一样执行具体动作
4. **监控反馈（Execution Monitor）**：像"感知系统"一样监控状态
5. **自适应调整（Adaptive Controller）**：像"反思机制"一样应对变化

### 核心架构图

```
┌──────────────────────────────────────────────────────────────┐
│              用户输入 (自然语言 + 可选图片)                    │
│              "根据图片前进" + image.png                       │
└────────────────────────┬─────────────────────────────────────┘
                         ↓
┌──────────────────────────────────────────────────────────────┐
│  VLM Core (视觉感知模块)                                     │
│  ┌────────────────────────────────────────────────────┐      │
│  │ 输入: 环境图像 (image_path)                         │      │
│  │ 处理: Ollama/API → 分析环境 → 生成描述              │      │
│  │ 输出: "机器人视觉感知到：画面中央有红色方块..."      │      │
│  └────────────────────────────────────────────────────┘      │
└────────────────────────┬─────────────────────────────────────┘
                         ↓
┌──────────────────────────────────────────────────────────────┐
│  High-Level LLM (规划器)                                     │
│  ┌────────────────────────────────────────────────────┐      │
│  │ 输入: 用户指令 + VLM环境理解 + 可用技能列表          │      │
│  │ 处理: 显示思考过程 → 理解意图 → 分解任务            │      │
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
│ │ 5种异常检测:        │ │  │ │ 输入: 单个任务描述    │ │
│ │ - 超时 ✅           │ │  │ │ 处理: 选择工具→生成参数│ │
│ │ - 卡住 ✅           │ │  │ │ 输出: 工具调用+结果    │ │
│ │ - 振荡 ✅           │ │  │ └──────────────────────┘ │
│ │ - 传感器失效 ✅     │ │  │                          │
│ │ - 环境变化 ✅       │ │  │     ↓                   │
│ └─────────────────────┘ │  │     │ Tool: move_forward  │
└──────────┬──────────────┘  │     │ Params: {dist: 1.0} │
           │                 │                          │
           │ 检测到异常?     │                          │
           ↓                 └──────────┬───────────────┘
    ┌──────────────────────────────────┘
    ↓
┌──────────────────────────────────────────────────────────────┐
│  Adaptive Controller (自适应控制器)                           │
│  ┌────────────────────────────────────────────────────┐      │
│  │ 后台监控: 异步任务实时检测异常 ✅                    │      │
│  │ 重新规划: 4个级别的智能恢复策略 ✅                    │      │
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

### v3.0 新特性

| 特性 | 说明 | 状态 |
|------|------|------|
| **VLM 环境理解** | `vlm_core.py` 专门处理视觉理解 | ✅ 完成 |
| **思考过程显示** | 高层LLM显示reasoning字段（200-300字） | ✅ 完成 |
| **5种异常检测** | 超时、卡住、振荡、传感器失效、环境变化 | ✅ 完成 |
| **后台实时监控** | 异步后台任务监控执行状态 | ✅ 完成 |
| **4级智能重新规划** | 根据异常类型自动选择恢复策略 | ✅ 完成 |
| **本地 Ollama 支持** | 使用 `qwen3-vl:4b` 本地模型 | ✅ 完成 |

---

## 模块详解

### 1. vlm_core.py - VLM 核心模块

**职责**：专门处理视觉语言模型相关功能，提供环境理解能力

**关键特性**：
- ✅ 支持本地 Ollama（推荐）：`qwen3-vl:4b`
- ✅ 支持远程 API（可选）：`qwen-vl-plus`
- ✅ 默认图片：`/home/xcj/work/FinalProject/VLM_Module/assets/red.png`
- ✅ 独立的提示词管理
- ✅ 懒加载客户端

**核心方法**：

```python
class VLMCore:
    def __init__(self,
                 vlm_prompt_path: Optional[str] = None,
                 default_image: str = "/home/xcj/work/FinalProject/VLM_Module/assets/red.png",
                 use_ollama: bool = True,
                 ollama_model: str = "qwen3-vl:4b",
                 ollama_host: str = "http://localhost:11434",
                 api_key: Optional[str] = None,
                 api_model: str = "qwen-vl-plus"):
        """初始化 VLM 核心"""

    def analyze_environment(self, image_path: Optional[str] = None) -> Optional[str]:
        """分析环境图像

        Args:
            image_path: 图像文件路径（可选，默认使用 red.png）

        Returns:
            环境理解文本，失败时返回 None
        """
```

**使用示例**：

```python
from LLM_Module.vlm_core import VLMCore

# 本地 Ollama 模式（推荐）
vlm = VLMCore(use_ollama=True)
result = vlm.analyze_environment("/path/to/image.png")
print(result)
# 输出: "机器人视觉感知到：画面中央偏下位置有一个红色正方形物体..."

# 远程 API 模式
vlm_api = VLMCore(use_ollama=False, api_key="your_key")
result = vlm_api.analyze_environment("/path/to/image.png")
```

### 2. high_level_llm.py - 高层 LLM 规划器

**职责**：理解用户意图，结合 VLM 环境理解和可用技能，生成分解的任务序列

**v3.0 变更**：
- ✅ 接收 `VLMCore` 实例作为参数
- ✅ 在 `plan_tasks()` 中自动调用 VLM 分析图片
- ✅ 将 VLM 理解结果加入 prompt
- ✅ 显示思考过程（reasoning字段）

**核心方法**：

```python
class HighLevelLLM:
    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 model: str = "qwen3-32b",
                 prompt_path: str = None,
                 vlm_core: Optional['VLMCore'] = None):
        """初始化高层LLM"""

    def plan_tasks(self,
                   user_input: str,
                   available_skills: List[str],
                   env_state: Optional[Dict[str, Any]] = None,
                   image_path: Optional[str] = None) -> List[Dict[str, Any]]:
        """根据用户输入和环境状态生成任务序列"""
```

**输出示例**：

```json
{
  "reasoning": "用户要求根据图片前进。VLM分析显示前方有红色方块...",
  "tasks": [
    {"step": 1, "task": "向红色方块前进1米", "type": "移动"},
    {"step": 2, "task": "停止", "type": "停止"}
  ],
  "summary": "根据VLM环境理解，规划了前进和停止两个步骤"
}
```

### 3. execution_monitor.py - 执行监控器

**职责**：检测任务执行过程中的5种异常情况

**异常类型**：

```python
class AnomalyType(Enum):
    TIMEOUT = "timeout"                  # 超时
    STUCK = "stuck"                     # 卡住
    OSCILLATION = "oscillation"         # 振荡
    SENSOR_FAILURE = "sensor_failure"   # 传感器失效
    ENVIRONMENT_CHANGE = "environment_change"  # 环境变化
```

**核心方法**：

```python
class ExecutionMonitor:
    def __init__(self,
                 monitoring_interval: float = 0.1,
                 timeout_threshold: float = 30.0,
                 stuck_threshold: float = 5.0):
        """初始化执行监控器"""

    def detect_anomaly(self,
                       current_state: Dict[str, Any],
                       task: Dict[str, Any]) -> Optional[Anomaly]:
        """检测异常

        Returns:
            检测到的异常，如果没有异常则返回None
        """
```

**辅助方法**：

```python
def _position_unchanged(self, pos1: Dict, pos2: Dict, threshold: float = 0.01) -> bool:
    """检查两个位置是否相同"""

def _calculate_distance(self, pos1: Dict, pos2: Dict) -> float:
    """计算两点间欧几里得距离"""

def _detect_oscillation(self, window_size: int = 6) -> bool:
    """检测振荡行为（来回移动）"""

def _calculate_average_position(self, position_records: list) -> Dict:
    """计算平均位置"""
```

### 4. adaptive_controller.py - 自适应控制器

**职责**：协调高层和低层 LLM，根据执行反馈自适应调整

**重新规划级别**：

```python
class ReplanLevel(Enum):
    PARAMETER_ADJUSTMENT = 1  # 参数调整（不改变任务）
    SKILL_REPLACEMENT = 2     # 技能替换（相同目标，不同方法）
    TASK_REORDER = 3          # 任务重排（调整顺序）
    FULL_REPLAN = 4           # 完全重新规划
```

**核心方法**：

```python
class AdaptiveController:
    async def run(self,
                  user_input: str,
                  tools: List[Dict],
                  execute_tool_fn: Callable,
                  available_skills: List[str],
                  env_state: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """运行自适应控制循环"""

    async def execute_with_monitoring(self,
                                      task: Task,
                                      tools: List[Dict],
                                      execute_tool_fn: Callable,
                                      env_state: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """带监控的任务执行"""

    async def _monitor_task_execution(self,
                                      task: Task,
                                      env_state: Dict[str, Any]) -> Optional[Anomaly]:
        """监控任务执行（后台运行）"""
```

---

## 完整API参考

### VLMCore API

#### `__init__`

```python
VLMCore(
    vlm_prompt_path: Optional[str] = None,
    default_image: str = "/home/xcj/work/FinalProject/VLM_Module/assets/red.png",
    use_ollama: bool = True,
    ollama_model: str = "qwen3-vl:4b",
    ollama_host: str = "http://localhost:11434",
    api_key: Optional[str] = None,
    api_model: str = "qwen-vl-plus"
)
```

**参数说明**：
- `vlm_prompt_path`: VLM 提示词文件路径（可选）
- `default_image`: 默认图片路径
- `use_ollama`: 是否使用本地 Ollama
- `ollama_model`: Ollama 模型名称
- `ollama_host`: Ollama 服务地址
- `api_key`: 远程 API 密钥
- `api_model`: 远程 API 模型名称

#### `analyze_environment`

```python
def analyze_environment(self, image_path: Optional[str] = None) -> Optional[str]:
```

**返回示例**：
```python
"机器人视觉感知到：画面中央偏下位置有一个红色正方形物体，位于白色平面上，距离机器人约50厘米。背景为纯白色，无其他障碍物。通道畅通，可安全移动。"
```

### ExecutionMonitor API

#### `__init__`

```python
ExecutionMonitor(
    monitoring_interval: float = 0.1,      # 监控检查间隔（秒）
    timeout_threshold: float = 30.0,       # 超时阈值（秒）
    stuck_threshold: float = 5.0           # 卡住检测阈值（秒）
)
```

#### `detect_anomaly`

```python
def detect_anomaly(self,
                   current_state: Dict[str, Any],
                   task: Dict[str, Any]) -> Optional[Anomaly]:
```

**current_state 格式**：
```python
{
    "position": {"x": 1.0, "y": 2.0, "z": 0.0},  # 机器人位置
    "sensor_status": {                         # 传感器状态
        "lidar": "ok",
        "camera": "ok",
        "imu": "ok"
    },
    "environment_version": 1                   # 环境版本号
}
```

**返回的 Anomaly 对象**：
```python
Anomaly(
    type=AnomalyType.TIMEOUT,
    description="任务执行超时（30.5秒）",
    severity="high",
    data={"elapsed_time": 30.5, "threshold": 30.0}
)
```

### AdaptiveController API

#### `__init__`

```python
AdaptiveController(
    high_level_llm: HighLevelLLM,
    low_level_llm: LowLevelLLM,
    execution_monitor: Optional[ExecutionMonitor] = None
)
```

#### `run`

```python
async def run(self,
              user_input: str,
              tools: List[Dict],
              execute_tool_fn: Callable,
              available_skills: List[str],
              env_state: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
```

---

## 使用示例

### 示例 1：基础使用（带 VLM）

```python
from LLM_Module import LLMAgent

# 初始化（默认启用 VLM）
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml"
)

# 运行任务（带图片）
results = agent.run_pipeline(
    user_input="根据图片前进",
    tools=[
        {"function": {"name": "move_forward"}},
        {"function": {"name": "turn"}}
    ],
    execute_tool_fn=lambda name, params: execute_tool(name, **params),
    image_path="/path/to/image.png"
)

# 结果
for result in results:
    if result["success"]:
        print(f"✅ {result['task']}: {result['result']}")
    else:
        print(f"❌ {result['task']}: {result['error']}")
```

### 示例 2：独立使用 VLM

```python
from LLM_Module.vlm_core import VLMCore

# 初始化 VLM
vlm = VLMCore(
    use_ollama=True,
    ollama_model="qwen3-vl:4b"
)

# 分析默认图片
result1 = vlm.analyze_environment()
print(result1)

# 分析指定图片
result2 = vlm.analyze_environment("/path/to/green.png")
print(result2)
```

### 示例 3：启用自适应控制

```python
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml",
    enable_adaptive=True  # 启用自适应
)

# 自适应模式会在任务失败或异常时自动重新规划
results = agent.run_pipeline(
    user_input="追击敌人",
    tools=tools,
    execute_tool_fn=execute_tool,
    env_state=current_state  # 提供环境状态用于监控
)
```

---

## 异常检测系统

### 5种异常类型详解

#### 1. 超时检测（TIMEOUT）

**触发条件**：任务执行时间超过 `timeout_threshold`（默认30秒）

**示例**：
```python
# 超过30秒未完成
anomaly = Anomaly(
    type=AnomalyType.TIMEOUT,
    description="任务执行超时（30.5秒）",
    severity="high"
)
```

**重新规划级别**：`PARAMETER_ADJUSTMENT`

#### 2. 卡住检测（STUCK）

**触发条件**：机器人位置在 `stuck_threshold`（默认5秒）时间内不变

**示例**：
```python
# 位置长时间不变
anomaly = Anomaly(
    type=AnomalyType.STUCK,
    description="机器人卡住（5.2秒未移动）",
    severity="medium"
)
```

**重新规划级别**：
- `medium` → `PARAMETER_ADJUSTMENT`
- `high` → `SKILL_REPLACEMENT`

#### 3. 振荡检测（OSCILLATION）

**触发条件**：机器人在小范围内来回移动（至少6个位置点）

**检测方法**：
1. 位置偏移法：起始和结束位置很近（<0.5米），但中间有明显偏离（>0.5米）
2. 方向变化法：方向变化超过2次

**示例**：
```python
anomaly = Anomaly(
    type=AnomalyType.OSCILLATION,
    description="检测到振荡行为（来回移动）",
    severity="medium"
)
```

**重新规划级别**：`TASK_REORDER`

#### 4. 传感器失效检测（SENSOR_FAILURE）

**触发条件**：传感器状态为 "failed"、"error" 或 False

**示例**：
```python
anomaly = Anomaly(
    type=AnomalyType.SENSOR_FAILURE,
    description="传感器失效: camera, imu",
    severity="high"
)
```

**重新规划级别**：`FULL_REPLAN`

#### 5. 环境变化检测（ENVIRONMENT_CHANGE）

**触发条件**：
- 方法1：`environment_version` 版本号变化
- 方法2：`environment_changed` 标志位为 True

**示例**：
```python
anomaly = Anomaly(
    type=AnomalyType.ENVIRONMENT_CHANGE,
    description="检测到环境变化",
    severity="high"
)
```

**重新规划级别**：`FULL_REPLAN`

---

## 自适应控制

### 4级重新策略

| 级别 | 名称 | 触发条件 | 策略 |
|------|------|----------|------|
| 1 | 参数调整 | 超时、轻微卡住 | 调整参数，重试相同任务 |
| 2 | 技能替换 | 严重卡住、障碍物 | 使用不同方法完成相同目标 |
| 3 | 任务重排 | 振荡行为 | 调整任务执行顺序 |
| 4 | 完全重新规划 | 环境变化、传感器失效 | 重新理解环境，生成新任务序列 |

### 重新规划示例

```python
# 场景1：机器人卡住
anomaly = {
    "type": "stuck",
    "severity": "medium"
}
# → PARAMETER_ADJUSTMENT
# 新任务: 后退0.5米，然后重试原任务

# 场景2：遇到障碍物
error = "遇到障碍物，无法前进"
# → SKILL_REPLACEMENT
# 新任务: 左转45度 → 前进1米 → 右转45度

# 场景3：环境变化
anomaly = {
    "type": "environment_change",
    "severity": "high"
}
# → FULL_REPLAN
# 完全重新规划，生成新任务序列
```

---

## 测试

### 运行测试

```bash
# 测试异常检测（7个测试）
python3 LLM_Module/test_execution_monitor.py

# 测试自适应控制（5个测试）
python3 LLM_Module/test_adaptive_controller.py
```

### 测试结果

**执行监控测试**: 7/7 通过 ✅

| 测试项 | 说明 |
|--------|------|
| 超时检测 | 2.5秒超时正确检测 ✅ |
| 卡住检测 | 1.5秒位置不变正确检测 ✅ |
| 振荡检测 | 来回移动模式正确检测 ✅ |
| 传感器失效检测 | 正确识别camera和imu失效 ✅ |
| 环境变化检测（版本号） | 版本号1→2变化正确检测 ✅ |
| 环境变化检测（标志位） | 标志位True正确检测 ✅ |
| 辅助方法 | 所有辅助方法正确工作 ✅ |

**自适应控制测试**: 5/5 通过 ✅

| 场景 | 说明 |
|------|------|
| 场景1：正常执行 | 规划1次，执行1次，无重新规划 ✅ |
| 场景2：卡住异常 | 检测到卡住 → 触发重新规划（PARAMETER_ADJUSTMENT）→ 新任务成功执行 ✅ |
| 场景3：障碍物失败 | 失败后重试3次 → 第4次成功 → 无需重新规划 ✅ |
| 场景4：多步骤任务 | 正确执行2个子任务，进度跟踪正常 ✅ |
| 场景5：超时异常 | 检测到超时 → 触发重新规划（PARAMETER_ADJUSTMENT）→ 新任务成功执行 ✅ |

---

## 版本历史

### v3.0.0 (2025-02-02) - 完整自适应控制系统

**新增**：
- ✅ 5种异常检测（超时、卡住、振荡、传感器失效、环境变化）
- ✅ 后台实时监控（异步任务）
- ✅ 4级智能重新规划
- ✅ LLM 思考过程显示（reasoning字段）
- ✅ VLM 环境理解结果显示（200字）
- ✅ VLM 独立模块（`vlm_core.py`）

**测试**：
- ✅ 12个单元测试全部通过
- ✅ 100% 功能覆盖

**修改**：
- ✅ `llm_core.py` - 重写为适配层
- ✅ `high_level_llm.py` - 集成VLM、显示思考过程
- ✅ `execution_monitor.py` - 实现完整异常检测
- ✅ `adaptive_controller.py` - 启用后台监控和重新规划

### v2.0.0 (2025-01-XX)

- ✅ 模块化架构重构
- ✅ 双层 LLM 分离
- ✅ 任务队列管理
- ✅ 执行监控框架
- ✅ 自适应控制框架

---

**详细文档，助力开发！** 🚀
