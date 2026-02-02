# LLM Module - 详细技术文档 (v3.0.0)

本文档详细介绍 LLM_Module v3.0.0 的架构设计、API 使用方法和扩展指南。

## 📋 目录

- [架构设计](#架构设计)
- [模块详解](#模块详解)
- [完整API参考](#完整api参考)
- [使用示例](#使用示例)
- [扩展指南](#扩展指南)
- [VLM 集成](#vlm-集成)

---

## 架构设计

### 设计理念

LLM_Module v3.0.0 实现了一个**支持视觉理解的双层 LLM 智能体架构**，模拟人类的认知过程：

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
│  VLM Core (视觉感知模块) ✨ v3.0 新增                         │
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
└──────────┬──────────────┘  │     │ Tool: move_forward  │
           │                 │     │ Params: {dist: 1.0} │
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

### v3.0 新特性

| 特性 | 说明 | 状态 |
|------|------|------|
| **VLM 独立模块** | `vlm_core.py` 专门处理视觉理解 | ✅ 完成 |
| **本地 Ollama 支持** | 使用 `qwen3-vl:4b` 本地模型 | ✅ 完成 |
| **默认图片支持** | 默认使用 `red.png` 作为测试图片 | ✅ 完成 |
| **灵活配置** | 支持本地 Ollama 和远程 API | ✅ 完成 |
| **向后兼容** | 旧代码无需修改即可使用 | ✅ 完成 |

---

## 模块详解

### 1. vlm_core.py - VLM 核心模块 ✨ 新增

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
        """分析环境图像"""
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

**核心方法**：

```python
class HighLevelLLM:
    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 model: str = "qwen3-32b",
                 prompt_path: str = None,
                 vlm_core: Optional['VLMCore'] = None):  # ← 新增
        """初始化高层LLM"""

    def plan_tasks(self,
                   user_input: str,
                   available_skills: List[str],
                   env_state: Optional[Dict[str, Any]] = None,
                   image_path: Optional[str] = None) -> List[Dict[str, Any]]:  # ← 新增参数
        """根据用户输入和环境状态生成任务序列"""
```

**工作流程**：

```python
# 1. 如果提供了 image_path 且 vlm_core 已初始化
if image_path and self.vlm_core:
    vlm_result = self.vlm_core.analyze_environment(image_path)
    vlm_understanding = f"【环境观察】\n{vlm_result}"

# 2. 将 VLM 理解加入用户输入
user_input_section = f"{vlm_understanding}\n\n【用户指令】\n{user_input}"

# 3. 调用文本 LLM 生成任务序列
tasks = llm_call(user_input_section, available_skills)
```

### 3. low_level_llm.py - 低层 LLM 执行控制器

**职责**：执行单个子任务，选择合适的工具并生成参数

**核心方法**：

```python
class LowLevelLLM:
    def execute_task(self,
                    task_description: str,
                    tools: List[Dict],
                    execute_tool_fn: Callable,
                    previous_result: Any = None) -> Dict:
        """执行单个子任务"""
```

**执行流程**：

```
任务描述 → 选择工具 → 生成参数 → 调用执行函数 → 返回结果
```

### 4. task_queue.py - 任务队列管理

**职责**：管理任务状态，支持重试和动态插入

**任务状态**：

```python
class TaskStatus(Enum):
    PENDING = "pending"           # 等待执行
    IN_PROGRESS = "in_progress"   # 执行中
    COMPLETED = "completed"       # 已完成
    FAILED = "failed"            # 失败
    CANCELLED = "cancelled"       # 已取消
```

**核心方法**：

```python
class TaskQueue:
    def add_task(self, task: Task) -> int:
        """添加任务到队列"""

    def get_next_task(self) -> Optional[Task]:
        """获取下一个待执行任务"""

    def mark_task_completed(self, task_id: int, result: Any):
        """标记任务为已完成"""

    def mark_task_failed(self, task_id: int, error: str):
        """标记任务为失败"""
```

### 5. execution_monitor.py - 执行监控器

**职责**：检测任务执行过程中的异常情况

**监控类型**：

```python
class AnomalyType(Enum):
    TIMEOUT = "timeout"           # 超时
    STUCK = "stuck"               # 卡住
    OSCILLATION = "oscillation"   # 振荡
    SENSOR_FAILURE = "sensor_failure"  # 传感器失效
    ENV_CHANGE = "env_change"     # 环境变化
```

**核心方法**：

```python
class ExecutionMonitor:
    def detect_anomaly(self,
                      execution_context: Dict[str, Any]) -> Optional[AnomalyType]:
        """检测执行异常"""
```

### 6. adaptive_controller.py - 自适应控制器

**职责**：协调高层和低层 LLM，根据执行反馈自适应调整

**重新规划级别**：

```python
class ReplanLevel(Enum):
    PARAMETER_ADJUST = "parameter_adjust"   # 参数调整
    SKILL_REPLACE = "skill_replace"         # 技能替换
    TASK_REORDER = "task_reorder"           # 任务重排
    FULL_REPLAN = "full_replan"             # 完全重新规划
```

**核心方法**：

```python
class AdaptiveController:
    async def run(self,
                  user_input: str,
                  tools: List[Dict],
                  execute_tool_fn: Callable,
                  available_skills: List[str],
                  env_state: Dict[str, Any]) -> List[Dict]:
        """运行自适应控制流程"""
```

### 7. llm_core.py - 主入口（适配层）

**职责**：整合所有模块，提供向后兼容的接口

**v3.0 变更**：
- ✅ 自动初始化 `VLMCore`
- ✅ 传递 `VLMCore` 给 `HighLevelLLM`
- ✅ 保持向后兼容

**核心类**：

```python
class LLMAgent:
    def __init__(self,
                 api_key: str,
                 base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1",
                 model: str = "qwen3-32b",
                 prompt_path: str = None,
                 enable_vlm: bool = True,           # ← 新增：是否启用 VLM
                 vlm_prompt_path: str = None,       # ← 新增：VLM 提示词
                 enable_adaptive: bool = False):
        """初始化LLM代理"""
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
    """分析环境图像

    Args:
        image_path: 图像文件路径（可选，默认使用 red.png）

    Returns:
        环境理解文本，失败时返回 None
    """
```

**返回示例**：
```python
"机器人视觉感知到：画面中央偏下位置有一个红色正方形物体，位于白色平面上，距离机器人约50厘米。背景为纯白色，无其他障碍物。通道畅通，可安全移动。"
```

### HighLevelLLM API

#### `plan_tasks`

```python
def plan_tasks(self,
               user_input: str,
               available_skills: List[str],
               env_state: Optional[Dict[str, Any]] = None,
               image_path: Optional[str] = None) -> List[Dict[str, Any]]:
    """根据用户输入和环境状态生成任务序列

    Args:
        user_input: 用户自然语言指令
        available_skills: 可用技能列表
        env_state: 当前环境状态（可选）
        image_path: 环境图像路径（可选，用于VLM理解）

    Returns:
        任务序列列表，格式：[{"step": 1, "task": "...", "type": "..."}, ...]
    """
```

**返回示例**：
```python
[
    {"step": 1, "task": "向红色方块前进1米", "type": "移动"},
    {"step": 2, "task": "停止", "type": "停止"}
]
```

### LLMAgent API

#### `run_pipeline`

```python
def run_pipeline(self,
                 user_input: str,
                 tools: List[Dict],
                 execute_tool_fn: Callable,
                 image_path: str = None) -> List[Dict]:
    """运行完整的双层LLM流程

    Args:
        user_input: 用户输入
        tools: 可用工具列表
        execute_tool_fn: 工具执行函数
        image_path: 环境图像路径（可选，用于VLM理解）

    Returns:
        执行结果列表
    """
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

### 示例 3：手动组合模块

```python
from LLM_Module.vlm_core import VLMCore
from LLM_Module.high_level_llm import HighLevelLLM

# 创建自定义 VLM
vlm = VLMCore(
    use_ollama=True,
    ollama_model="qwen3-vl:4b",
    default_image="/custom/path/default.png"
)

# 创建 High-Level LLM
high_llm = HighLevelLLM(
    api_key="your_api_key",
    vlm_core=vlm
)

# 规划任务
tasks = high_llm.plan_tasks(
    user_input="前进1米",
    available_skills=["move_forward", "turn"],
    image_path="/path/to/image.png"
)

for task in tasks:
    print(f"{task['step']}. {task['task']} ({task['type']})")
```

### 示例 4：禁用 VLM

```python
# 不使用 VLM（纯文本规划）
agent = LLMAgent(
    api_key="your_api_key",
    enable_vlm=False
)

# 只能使用文本输入
results = agent.run_pipeline(
    user_input="前进1米",
    tools=tools,
    execute_tool_fn=execute_tool
    # 没有 image_path 参数
)
```

### 示例 5：启用自适应控制

```python
agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml",
    enable_adaptive=True  # 启用自适应
)

# 自适应模式会在任务失败时自动重新规划
results = agent.run_pipeline(
    user_input="追击敌人",
    tools=tools,
    execute_tool_fn=execute_tool
)
```

---

## 扩展指南

### 添加新的监控类型

在 `execution_monitor.py` 中添加：

```python
def detect_anomaly(self, execution_context: Dict[str, Any]) -> Optional[AnomalyType]:
    # 现有检测逻辑...

    # 添加新的检测类型
    if self._check_custom_condition(execution_context):
        return AnomalyType.CUSTOM  # 需要在 AnomalyType 中添加
```

### 自定义 VLM 提示词

创建自定义提示词文件 `custom_vlm_prompt.yaml`：

```yaml
system_prompt: |
  你是一个专业的机器人视觉导航助手。

prompt: |
  请分析图像并提供导航建议：
  1. 识别目标物体
  2. 计算距离和方向
  3. 评估路径可行性
  4. 给出具体行动建议
```

使用自定义提示词：

```python
vlm = VLMCore(
    vlm_prompt_path="custom_vlm_prompt.yaml"
)
```

### 切换 VLM 模型

```python
# 使用更强的远程模型
vlm = VLMCore(
    use_ollama=False,
    api_key="your_api_key",
    api_model="qwen-vl-max"  # 更强的模型
)
```

---

## VLM 集成

### VLM 架构

```
VLMCore (vlm_core.py)
    ├─ 本地 Ollama 模式
    │   └─ qwen3-vl:4b（推荐）
    └─ 远程 API 模式
        └─ qwen-vl-plus / qwen-vl-max
```

### 配置选项

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_ollama` | `True` | 是否使用本地 Ollama |
| `ollama_model` | `"qwen3-vl:4b"` | Ollama 模型名称 |
| `ollama_host` | `"localhost:11434"` | Ollama 服务地址 |
| `default_image` | `/home/xcj/work/FinalProject/VLM_Module/assets/red.png` | 默认图片 |
| `api_model` | `"qwen-vl-plus"` | API 模型名称 |

### 环境准备

**本地 Ollama 模式**：

```bash
# 安装 Ollama
curl -fsSL https://ollama.com/install.sh | sh

# 启动服务
ollama serve

# 运行模型
ollama run qwen3-vl:4b
```

**远程 API 模式**：

```python
# 设置 API Key
import os
os.environ['Test_API_KEY'] = 'your_api_key'

# 使用远程 API
vlm = VLMCore(use_ollama=False)
```

### 故障排除

**Q: VLM 客户端初始化失败？**

```bash
# 检查 Ollama 是否运行
ps aux | grep ollama

# 重启 Ollama
ollama serve

# 检查模型是否已下载
ollama list | grep qwen3-vl

# 重新下载模型
ollama pull qwen3-vl:4b
```

**Q: 环境理解结果不准确？**

1. 调整 VLM 提示词
2. 尝试使用更强的模型
3. 提供更清晰的图片

---

## 版本历史

### v3.0.0 (2025-02-02) - ✨ VLM 集成版本

**新增**：
- ✅ `vlm_core.py` - 独立 VLM 模块
- ✅ `prompts/vlm_perception.yaml` - VLM 提示词
- ✅ 支持本地 Ollama（qwen3-vl:4b）
- ✅ 支持远程 API（qwen-vl-plus）
- ✅ 默认图片支持（red.png）

**修改**：
- ✅ `llm_core.py` - 重写为适配层
- ✅ `high_level_llm.py` - 接收 VLM 实例

**废弃**：
- ⚠️ `llm_core_old.py` - 旧版本备份

### v2.1.0 (2025-01-XX)

- ✅ 简化监控器，移除具体检测逻辑（待后续添加）
- ✅ 简化自适应控制器，保留框架
- ✅ 修复统计逻辑兼容性问题

### v2.0.0 (2025-01-XX)

- ✅ 模块化架构重构
- ✅ 双层 LLM 分离
- ✅ 任务队列管理
- ✅ 执行监控框架
- ✅ 自适应控制框架

---

**详细文档，助力开发！** 🚀
