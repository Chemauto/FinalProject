# Interactive_Module - 交互界面模块

用户交互界面的核心模块，提供 CLI 命令行交互功能，协调 LLM 和 Robot_Module。

## 📁 模块结构

```
Interactive_Module/
├── README.md              # 本文档
└── interactive.py         # CLI 交互主程序
```

## 🎯 功能特性

### 核心功能
- **自然语言交互**: 支持中文指令输入
- **双层 LLM 协调**: 自动协调规划 LLM 和执行 LLM
- **动态提示词加载**: 根据可用技能动态生成提示词
- **实时结果反馈**: 显示任务规划和执行过程
- **环境变量支持**: 自动加载 .env 文件
- **错误处理**: 友好的错误提示信息
- **自适应控制** (v2.1): 默认启用环境变化检测和自动重新规划

### v2.1 新增
- ✅ **启用自适应模式** - 默认启用 `enable_adaptive=True`
- ✅ **后台监控** - 执行时实时检测异常
- ✅ **自动重新规划** - 检测到环境变化时自动重新规划

## 🚀 快速开始

### 启动系统

```bash
# 1. 设置 API 密钥
export Test_API_KEY=your_api_key_here

# 2. 启动仿真器（终端1）
python3 Sim_Module/sim2d/simulator.py

# 3. 启动交互界面（终端2）
python3 Interactive_Module/interactive.py
```

### 交互示例

```bash
💬 请输入指令: 前进1米然后左转90度

============================================================
🧠 [高层LLM] 任务规划中...
============================================================
✅ [规划完成] 共分解为 2 个子任务
📋 [任务概述] 先转向再前进

子任务序列：
  步骤 1: 左转90度 (转向)
  步骤 2: 向前移动1米 (移动)

████████████████████████████████████████████████████████████
🚀 [开始执行] 按顺序执行子任务
████████████████████████████████████████████████████████████

【步骤 1/2】
🔧 [工具调用] turn(angle=90.0, angular_speed=0.5)
✅ 成功

【步骤 2/2】
🔧 [工具调用] move_forward(distance=1.0, speed=0.3)
✅ 成功

📊 [完成] 2/2 个任务成功
```

## 📊 系统流程

```
用户输入
    ↓
┌─────────────────────────────────────────┐
│ Interactive_Module.interactive.py      │
│                                         │
│ 1. 接收用户自然语言指令                  │
│ 2. 注册 Robot_Module 工具（自动）       │
│ 3. 动态加载并填充提示词                  │
│ 4. 调用 LLM_Module 双层LLM               │
│ 5. 执行 MCP 工具函数                     │
│ 6. 显示执行结果                          │
└──────────────┬──────────────────────────┘
               ↓
        ┌──────┴──────┐
        ↓             ↓
    LLM_Module    Robot_Module
    (规划+执行)    (工具调用 + ROS2发布)
        ↓             ↓
    ROS2 Topic (/robot/command)
        ↓
    Sim_Module (仿真执行)
```

## 🧠 自适应控制流程（v2.1）

启用自适应模式后（默认已启用），系统会自动监控和重新规划：

```
用户输入
    ↓
┌─────────────────────────────────────┐
│ 1. 高层LLM规划任务序列               │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│ 2. 执行循环（带后台监控）            │
│    - 取出任务                        │
│    - 启动后台监控任务                │
│    - 低层LLM执行                     │
│    - 检测异常？                      │
│      - 是 → 触发重新规划             │
│      - 否 → 继续执行                 │
└──────────────┬──────────────────────┘
               ↓
       如果检测到异常
             ↓
┌─────────────────────────────────────┐
│ 3. 自适应重新规划                    │
│    - 分析失败/异常原因               │
│    - 选择重新规划级别（4级）         │
│    - 调用高层LLM生成新任务           │
│    - 插入任务队列                    │
└─────────────────────────────────────┘
```

## 📝 输入指令格式

### 格式1：基础指令
```
前进1米
左转90度
停止
后退0.5米
```

### 格式2：组合指令
```
前进1米，然后左转90度
先右转45度，再前进2米
左转90度，后退1米，停止
```

### 格式3：追击敌人
```
追击敌人
追击最近的敌人
```

### 格式4：带图片路径的指令（VLM）
```
检测颜色并移动
根据 /home/robot/work/FinalProject/VLM_Module/assets/green.png 检测颜色并执行动作
前进1米，然后根据 /path/to/red.png 检测颜色
```

### 格式5：复杂组合指令
```
前进1米，左转90度，再根据 /home/robot/work/FinalProject/VLM_Module/assets/blue.png 检测颜色并执行相应动作
```

## 🔧 核心组件

### interactive.py - CLI 交互主程序

**主要功能：**

1. **环境初始化**
   - 加载 .env 文件
   - 设置 API 密钥
   - 初始化 ROS2 通讯

2. **MCP 工具加载**
   - 启动 FastMCP 服务器
   - 注册所有 Robot_Module 工具
   - 获取可用工具列表

3. **动态提示词生成**
   - 读取规划提示词模板
   - 填充机器人配置
   - 填充可用技能列表

4. **双层 LLM 流程 + 自适应控制**
   - 高层 LLM: 任务规划
   - 低层 LLM: 执行控制
   - 执行监控: 实时检测异常
   - 自动重新规划: 环境变化时重新规划
   - 结果汇总和反馈

## ⚙️ 配置说明

### 环境变量配置

创建 `.env` 文件（项目根目录）：

```bash
# API 密钥（必需）
Test_API_KEY=your_api_key_here

# LLM 模型配置（可选）
LLM_MODEL=qwen3-32b
LLM_BASE_URL=https://your-api-endpoint

# VLM 模型配置（可选）
VLM_MODEL=qwen-vl-plus
```

### 启用/禁用自适应控制

编辑 `interactive.py` 第 150-154 行：

```python
# 初始化 LLM Agent（启用自适应控制）
llm_agent = LLMAgent(
    api_key=api_key,
    prompt_path=str(prompt_path),
    enable_adaptive=True  # ← 改为 False 禁用自适应
)
```

### 提示词配置

规划提示词位于 `LLM_Module/prompts/planning_prompt_2d.yaml`

## 🔌 API 接口

### LLMAgent Pipeline

```python
def run_pipeline(
    user_input: str,          # 用户输入
    tools: list,              # 可用工具列表
    execute_tool_fn: callable # 工具执行函数
) -> list:
    """
    双层 LLM 流水线（启用自适应时为异步）

    Returns:
        执行结果列表
    """
```

### 工具执行函数

```python
def execute_tool(function_name: str, parameters: dict) -> dict:
    """
    执行单个工具

    Args:
        function_name: 工具名称（如 'move_forward'）
        parameters: 参数字典（如 {'distance': 1.0}）

    Returns:
        执行结果字典 {"result": ..., "delay": ...}
    """
```

## 📖 使用示例

### 示例 1: 基础运动控制

```bash
💬 请输入指令: 前进2米，然后后退1米

[高层LLM] 任务规划:
  步骤1: 前进2米
  步骤2: 后退1米

[下层LLM] 执行控制:
  ✓ 调用工具: move_forward(distance=2.0, speed=0.3)
  ✓ 调用工具: move_backward(distance=1.0, speed=0.3)

📊 [完成] 2/2 个任务成功
```

### 示例 2: 追击敌人

```bash
💬 请输入指令: 追击最近的敌人

[高层LLM] 任务规划:
  步骤1: 获取敌人位置
  步骤2: 追击最近的敌人

[下层LLM] 执行控制:
  ✓ 调用工具: get_enemy_positions()
  ✓ 调用工具: chase_enemy(positions=...)
  [追击] 找到敌人 1 at (612, 114)
  [追击] 距离: 282.0 像素 (2.82 米)
  [追击] ✓ 已到达目标！

📊 [完成] 2/2 个任务成功
```

### 示例 3: 复杂任务

```bash
💬 请输入指令: 先左转45度，前进1.5米，然后右转90度

[高层LLM] 任务规划:
  步骤1: 左转45度
  步骤2: 前进1.5米
  步骤3: 右转90度

[下层LLM] 执行控制:
  ✓ 调用工具: turn(angle=-45.0, angular_speed=0.5)
  ✓ 调用工具: move_forward(distance=1.5, speed=0.3)
  ✓ 调用工具: turn(angle=90.0, angular_speed=0.5)

📊 [完成] 3/3 个任务成功
```

### 示例 4: 自适应重新规划（v2.1）

```bash
💬 请输入指令: 前进到目标

🔵 [阶段1] 初始任务规划
✅ [规划完成] 共分解为 1 个子任务
📋 [任务概述] 前进到目标位置

🔵 [阶段2] 执行任务序列

【步骤 1/1】
⚙️  [执行中] 前进到目标位置
⚠️  [监控检测] 机器人卡住（5.2秒未移动）
⚠️  [异常] 任务执行成功但检测到异常: 机器人卡住

🔄 [重新规划] 第 1 次 (级别: SKILL_REPLACEMENT)
✅ [重新规划] 策略: 使用不同方法完成相同目标
✅ [重新规划] 已添加 2 个新任务

【步骤 1/2】
✅ [成功] 任务完成: 后退一点解除卡住

【步骤 2/2】
✅ [成功] 任务完成: 重新前进到目标

📊 [进度] 2/2 (100.0%)
```

## 🐛 调试技巧

### 1. 查看可用工具列表

启动时会显示所有可用工具：

```
可用工具: 8 个
  • move_forward
    参数: distance(number), speed(number)
    描述: 向前移动指定距离

  • turn
    参数: angle(number), angular_speed(number)
    描述: 原地旋转指定角度
  ...
```

### 2. 测试单个工具

在交互程序中输入简单指令：

```bash
💬 请输入指令: move_forward(distance=1.0, speed=0.3)
```

### 3. 禁用自适应模式

如果只想测试基础功能，可以临时禁用自适应：

```python
# 在 interactive.py 中修改
llm_agent = LLMAgent(
    api_key=api_key,
    prompt_path=str(prompt_path),
    enable_adaptive=False  # 禁用自适应
)
```

## ❓ 常见问题

### Q1: 提示 "API Key 未配置"

**A:** 检查以下内容：
- `.env` 文件是否存在
- `Test_API_KEY` 是否设置
- 使用 `export Test_API_KEY=your_key` 设置环境变量

### Q2: 工具调用失败

**A:** 检查以下内容：
- Robot_Module 是否正常工作
- ROS2 通讯是否正常
- 仿真器是否已启动

### Q3: LLM 返回无效 JSON

**A:** 检查以下内容：
- 提示词格式是否正确
- LLM 模型是否支持 JSON 输出
- 尝试更换模型（如 qwen3-72b）

### Q4: 自适应重新规划不触发

**A:** 检查以下内容：
- 确认 `enable_adaptive=True`
- 环境状态是否正确传递
- 监控阈值是否设置合理（默认30秒超时，5秒卡住）

## 🔗 相关模块

- `LLM_Module/llm_core.py` - 双层 LLM 核心
- `LLM_Module/adaptive_controller.py` - 自适应控制器
- `LLM_Module/execution_monitor.py` - 执行监控器
- `Robot_Module/skill.py` - MCP 工具注册
- `ros_topic_comm.py` - ROS2 通讯

## 📝 依赖

```
openai>=1.0.0         # OpenAI API
python-dotenv>=1.0.0  # 环境变量管理
pyyaml>=6.0           # YAML 配置解析
asyncio               # 异步执行支持
```

## 📈 版本历史

### v2.1.0 (当前版本)
- ✅ **默认启用自适应模式** - `enable_adaptive=True`
- ✅ **后台监控** - 执行时实时检测异常
- ✅ **自动重新规划** - 根据异常类型智能选择策略
- ✅ **简化访问** - 通过属性访问 `client` 和 `model`

### v2.0.0
- 支持双层 LLM 架构
- 动态提示词加载
- MCP 工具集成

---

**简单交互，强大功能，智能自适应！** 🚀
