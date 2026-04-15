# Interactive_Module

## 作用

`Interactive_Module/interactive.py` 是整个项目的统一终端交互入口。
它使用 `rich` 构建 TUI，负责：

- 显示欢迎面板、状态栏和 slash 命令
- 接收用户任务输入
- 以"机器人智能体"身份决定直接回复，还是调用技能
- 最外层只暴露两个高层技能：`vlm_observe` 和 `robot_act`
- **渐进式渲染**：Thinking 面板、工具结果、Assistant 回复按执行顺序实时显示

当前只保留文字 TUI 入口，不再提供语音交互模式。

## 渐进式渲染流程

用户输入指令后的输出顺序：

```text
1. Thinking 面板        — agent 的思考过程（环境感知需求、动作规划、执行策略）
2. vlm_observe 面板    — 环境观测结果（仅显示 visual_context）
3. Thinking 面板        — agent 根据观测结果的进一步思考
4. robot_act 内部日志  — llm_core 规划输出（████ 块）、llm_lowlevel 执行输出（⚙️🔧），通过 callback 流式显示
5. robot_act 摘要面板  — total_tasks / success_count / failure_count
6. Assistant 面板      — 最终文本回复
```

## 日志抑制

- `llm_core.py` / `llm_lowlevel.py` 的 stdout 输出通过 `agent_tools.py` 的 `_StreamingBuffer` 机制转发到终端（实时可见）
- `navigation.py` 的 stderr 输出（`[go2.skill]`、`[go2.speech]`）在交互模式下被抑制
- `agent_tools.py` 内部使用 `redirect_stdout` 捕获 stdout，`interactive.py` 使用 `redirect_stderr` 抑制噪音

## 依赖

- `openai`
- `pyyaml`
- `rich`
- 可选：`python-dotenv`
- 项目根目录 `.env` 中需要有 `Test_API_KEY`

## 使用

在项目根目录运行：

```bash
python3 Interactive_Module/interactive.py
```

## 命令

- `/help`
  显示命令说明和当前 `Agent Tools -> Skills` 分层关系
- `/tools`
  展示上层 `Perception Tool / Action Tool`，以及下层 `Vision Skills / Action Skills`
- `/reset`
- `/status`
- `/vlm`
- `/quit`

## 输入输出

- 输入：终端文字指令
- 输出：TUI 面板化展示，按执行顺序渐进渲染
