# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 在此代码库中工作时提供指导。

## 项目概述

这是一个将视觉-语言模型（VLM）与 ROS2 集成的毕业设计项目，用于机器人控制。系统使用阿里云的通义千问模型（通过 DashScope API）处理自然语言指令，并将其转换为 ROS2 动作命令以实现机器人导航和操作。

## 架构设计

代码库采用模块化设计：

### 核心组件

- **LLM 集成层** (`VLM_Modele/DemoSimple.py`, `DemoStart.py`, `Demo2Robot.py`)
  - 使用 OpenAI 兼容 API 通过 DashScope 访问通义千问模型
  - 支持纯文本模型 (`qwen-plus`) 和视觉-语言模型 (`qwen-vl-plus`)
  - 基础 URL: `https://dashscope.aliyuncs.com/compatible-mode/v1`

- **Prompt 管理系统** (`VLM_Modele/LLM_prompts/`)
  - 基于 YAML 的 prompt 模板存储在 `LLM_prompts/Basic_prompts/` 目录
  - 每个 YAML 文件包含多个带有唯一 ID 的 prompt 条目
  - 模板变量使用 `{variable_name}` 语法进行运行时渲染
  - Prompt 结构包含 `id`、`description` 和 `messages` 字段

- **ROS2 集成** (`VLM_Modele/Demo2Ros.py`, `2Demo2Ros.py`, `WARNROS2.py`)
  - 将 LLM 的 JSON 输出转换为 ROS2 动作调用
  - 支持的动作类型: `navigate`（导航）、`pick`（抓取）、`place`（放置）
  - 使用 `nav2_msgs.action.NavigateToPose` 进行导航
  - 提供模拟实现，可在无 ROS2 运行时的情况下测试

### 数据流程

1. 用户提供自然语言输入
2. `load_prompt()` 从 YAML 文件中检索相应的 prompt 模板
3. `render_messages()` 替换模板变量
4. LLM 生成 JSON 命令: `{"action": "<名称>", "parameters": {...}}`
5. 动作分发器调用相应的 ROS2 动作客户端

## 配置说明

### API Key 设置

项目需要在 `VLM_Modele/.env` 文件中配置 DashScope API key：

```
Test_API_KEY=你的API密钥
```

所有 demo 脚本通过 `python-dotenv` 加载 API key。

### 依赖项

必需的 Python 包：
- `openai` - OpenAI 兼容 API 客户端
- `python-dotenv` - 环境变量管理
- `pyyaml` - Prompt 模板解析
- `rclpy` - ROS2 Python 客户端库（用于 ROS2 集成）
- `nav2_msgs` - Navigation2 动作消息

## 运行 Demo

### 基础 LLM 调用
```bash
cd VLM_Modele
python DemoSimple.py
```
最简单的示例：向 `qwen-vl-plus` 模型发送图片 URL。

### Prompt 模板系统
```bash
cd VLM_Modele
python DemoStart.py
```
演示 YAML prompt 加载和模板变量渲染。

### 机器人任务解析（JSON 输出）
```bash
cd VLM_Modele
python Demo2Robot.py
```
展示机器人任务解释的结构化 JSON 输出。

### 完整 ROS2 集成
```bash
cd VLM_Modele
python Demo2Ros.py
```
生产级 ROS2 动作客户端集成（需要 ROS2 环境）。

### 模拟测试（无需 ROS2）
```bash
cd VLM_Modele
python 2Demo2Ros.py
```
运行完整流程，使用模拟的 ROS2 动作，无需 ROS2 运行时即可测试。

## Prompt 模板格式

Prompt 存储在 `VLM_Modele/LLM_prompts/Basic_prompts/*.yaml`：

```yaml
- id: task-to-ros2-action
  description: 将自然语言转换为 ROS2 动作命令
  messages:
    - role: system
      content:
        - type: text
          text: |
            系统 prompt 内容...
    - role: user
      content:
        - type: text
          text: "{user_input}"  # 模板变量
```

`load_prompt(prompt_id, file)` 函数搜索匹配的 ID，`render_messages()` 替换模板变量。

## ROS2 动作模式

ROS2 动作遵循以下模式（来自 `WARNROS2.py`）：
- 动作客户端继承自 `rclpy.node.Node`
- 使用 `rclpy.action.ActionClient` 指定具体的动作类型
- 目标通过 `send_goal_async()` 异步发送
- 示例动作: `NavigateToPose`，使用 `PoseStamped` 目标消息

## 关键设计决策

- **YAML 替代硬编码 prompt**: 无需修改代码即可快速迭代 prompt
- **模拟函数**: `2Demo2Ros.py` 包含模拟实现（`mock_send_navigate_goal` 等），可在没有 ROS2 的环境下开发
- **模板安全性**: `render_messages()` 使用字符串替换而非 `.format()`，避免缺失变量时的 KeyError
- **JSON 解析容错**: 当模型在输出中添加说明文本时，使用正则表达式回退机制提取 JSON

## 文件命名规范

- `Demo*.py`: 递进式演示脚本，展示功能逐步添加
- `2Demo*.py`: 增强版本，包含额外测试功能
- `WARN*.py`: WIP（开发中）或参考实现
- YAML 文件使用描述性名称，与对应的 demo 脚本匹配（如 `demo2ros.yaml`）
