# LLM_Module - 双层 LLM 核心模块

实现双层 LLM 架构，支持任务规划、执行控制、视觉理解和自适应重规划。

## 📁 模块结构

```
LLM_Module/
├── __init__.py                 # 模块导出
├── vlm_core.py                 # ✨ VLM 核心模块（新增）
├── llm_core.py                 # LLMAgent 适配层（已重写）
├── high_level_llm.py           # 高层LLM - 任务规划器
├── low_level_llm.py            # 低层LLM - 执行控制器
├── task_queue.py               # 任务队列管理
├── execution_monitor.py        # 执行监控器
├── adaptive_controller.py      # 自适应控制器
├── prompts/
│   ├── planning_prompt_2d.yaml # 规划提示词模板
│   └── vlm_perception.yaml     # ✨ VLM 环境理解提示词（新增）
├── README.md                   # 本文档（简洁版）
└── README4AI.md                # 详细技术文档
```

## 🎯 核心功能

| 功能 | 模块 | 状态 |
|------|------|------|
| **任务分解** | `high_level_llm.py` | ✅ 完成 |
| **视觉理解** | `vlm_core.py` | ✅ 完成 |
| **任务执行** | `low_level_llm.py` | ✅ 完成 |
| **任务队列管理** | `task_queue.py` | ✅ 完成 |
| **执行监控** | `execution_monitor.py` | ⚠️ 框架完成，检测逻辑待添加 |
| **自适应重规划** | `adaptive_controller.py` | ⚠️ 框架完成，触发逻辑待添加 |

## 🔄 完整流程

```
用户输入 + 图片（可选）
    ↓
┌─────────────────────────────────────┐
│ vlm_core.py (VLM 模块)              │  ← ✨ 新增
│ - 分析环境图像                      │
│ - 生成环境理解文本                  │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│ high_level_llm.py (高层LLM)         │
│ - 理解用户意图                      │
│ - 结合 VLM 环境理解                 │
│ - 分解为任务序列                    │
└──────────────┬──────────────────────┘
               ↓
    [任务1, 任务2, 任务3, ...]
               ↓
┌─────────────────────────────────────┐
│ task_queue.py (任务队列)            │
│ - 管理任务状态                      │
│ - 支持重试机制                      │
│ - 动态插入新任务                    │
└──────────────┬──────────────────────┘
               ↓
    逐个取出任务执行
               ↓
┌─────────────────────────────────────┐
│ execution_monitor.py (监控器)       │
│ - 检测异常（框架）                  │
│ - 支持超时/卡住/振荡检测（待添加）   │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│ low_level_llm.py (低层LLM)          │
│ - 选择合适工具                      │
│ - 生成工具参数                      │
│ - 调用 Robot_Module 执行            │
└──────────────┬──────────────────────┘
               ↓
        Robot_Module (技能调用)
```

## 📖 快速开始

### 基础使用（启用 VLM）

```python
from LLM_Module import LLMAgent

agent = LLMAgent(
    api_key="your_api_key",
    prompt_path="LLM_Module/prompts/planning_prompt_2d.yaml",
    enable_vlm=True  # 启用视觉理解（默认）
)

# 带图片的输入
results = agent.run_pipeline(
    user_input="根据图片前进",
    tools=available_tools,
    execute_tool_fn=execute_function,
    image_path="/path/to/image.png"  # VLM 自动分析
)
```

### 禁用 VLM（纯文本模式）

```python
agent = LLMAgent(
    api_key="your_api_key",
    enable_vlm=False  # 禁用视觉理解
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

### 独立使用 VLM

```python
from LLM_Module.vlm_core import VLMCore

vlm = VLMCore(
    use_ollama=True,
    ollama_model="qwen3-vl:4b"
)

# 分析默认图片（red.png）
result = vlm.analyze_environment()

# 分析指定图片
result = vlm.analyze_environment("/path/to/image.png")
print(result)
```

## 📝 各文件说明

| 文件 | 行数 | 主要功能 |
|------|------|----------|
| `vlm_core.py` | 330 | ✨ VLM 核心模块，支持本地 Ollama 和远程 API |
| `llm_core.py` | 298 | LLMAgent 适配层，自动初始化 VLM |
| `high_level_llm.py` | 316 | 高层LLM，使用 VLM 理解环境并分解任务 |
| `low_level_llm.py` | 282 | 低层LLM，选择工具并执行 |
| `task_queue.py` | 254 | 任务队列，状态管理和重试 |
| `execution_monitor.py` | 148 | 监控器，检测执行异常 |
| `adaptive_controller.py` | 383 | 自适应控制器，协调规划和执行 |

## 🎨 VLM 配置

### 默认配置

```python
VLMCore(
    vlm_prompt_path=None,                        # 使用默认提示词
    default_image="/home/xcj/work/FinalProject/VLM_Module/assets/red.png",
    use_ollama=True,                             # 使用本地 Ollama
    ollama_model="qwen3-vl:4b",                  # 模型名称
    ollama_host="http://localhost:11434",        # 服务地址
    api_key=None,                                # 自动从环境变量读取
    api_model="qwen-vl-plus"                     # API 模型
)
```

### 本地 Ollama 模式（推荐）

```python
vlm = VLMCore(
    use_ollama=True,
    ollama_model="qwen3-vl:4b"
)
```

**前提**：
```bash
# 启动 Ollama
ollama serve

# 运行模型
ollama run qwen3-vl:4b
```

### 远程 API 模式

```python
vlm = VLMCore(
    use_ollama=False,
    api_key="your_api_key",
    api_model="qwen-vl-plus"
)
```

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

1. **职责分离** - 规划、执行、视觉完全解耦
2. **VLM 独立** - 视觉模块可独立使用和测试
3. **向后兼容** - 旧代码无需修改
4. **可扩展** - 预留监控和重规划接口
5. **模块化** - 每个模块独立可测试
6. **灵活配置** - 支持本地 Ollama 和远程 API

## 📊 数据流

```python
# 用户输入 + 图片
user_input = "根据图片前进"
image_path = "/path/to/image.png"

# 1. VLM 分析环境
vlm_understanding = vlm.analyze_environment(image_path)
# 返回: "机器人视觉感知到：画面中央偏下位置有一个红色正方形物体..."

# 2. 高层LLM分解任务（结合 VLM 理解）
tasks = high_level_llm.plan_tasks(
    user_input=user_input,
    available_skills=["move_forward", "turn"],
    image_path=image_path  # VLM 会自动分析
)
# 返回: [
#   {"step": 1, "task": "向红色方块前进1米", "type": "移动"},
#   {"step": 2, "task": "停止", "type": "停止"}
# ]

# 3. 低层LLM执行任务
for task in tasks:
    result = low_level_llm.execute_task(
        task_description=task["task"],
        tools=available_tools,
        execute_tool_fn=execute_tool
    )
    # 返回: {"status": "success", "action": "move_forward", ...}
```

## 🔗 相关模块

- `Interactive_Module/interactive.py` - 交互界面（自动提取图片路径）
- `Robot_Module/skill.py` - MCP 工具注册
- `VLM_Module/vlm_core.py` - 颜色检测（执行层）

## 📚 详细文档

查看 **[README4AI.md](README4AI.md)** 了解：
- 详细的架构设计
- 完整的 API 参考
- 使用示例和最佳实践
- 扩展指南

查看 **[NEW_ARCHITECTURE.md](NEW_ARCHITECTURE.md)** 了解：
- 新架构说明
- VLM 集成详情
- 迁移指南

查看 **[VLM_USAGE.md](VLM_USAGE.md)** 了解：
- VLM 使用指南
- 配置选项
- 故障排除

## 📈 版本历史

### v3.0.0 (当前) - ✨ VLM 集成版本
- ✅ 新增 `vlm_core.py` 独立 VLM 模块
- ✅ 重写 `llm_core.py` 为适配层
- ✅ 修改 `high_level_llm.py` 接收 VLM 实例
- ✅ 支持本地 Ollama 和远程 API
- ✅ 默认图片支持（red.png）
- ✅ 向后兼容旧代码

### v2.1.0
- ✅ 简化监控器，移除具体检测逻辑（待后续添加）
- ✅ 简化自适应控制器，保留框架
- ✅ 修复统计逻辑兼容性问题

### v2.0.0
- ✅ 模块化架构重构
- ✅ 双层 LLM 分离
- ✅ 任务队列管理
- ✅ 执行监控框架
- ✅ 自适应控制框架

---

**简洁文档，快速上手！** 🚀
