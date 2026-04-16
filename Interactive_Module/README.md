# Interactive_Module

## 作用

`Interactive_Module/interactive.py` 是整个项目的统一终端交互入口。
它使用 `rich` 构建 TUI，负责：

- 显示欢迎面板、状态栏和 slash 命令
- 接收用户任务输入
- 决定直接回复，还是调用技能
- 最外层只暴露两个高层工具：`vlm_observe` 和 `robot_act`
- 渐进式渲染 Thinking、工具结果和最终回复

当前只保留文字 TUI 入口，不再提供语音交互模式。

## 渐进式渲染流程

用户输入后的典型输出顺序：

```text
1. Thinking 面板        — agent 思考过程
2. vlm_observe 面板    — 环境观测结果
3. Thinking 面板        — 基于观测的进一步决策
4. robot_act 内部日志  — 规划和执行过程的流式输出
5. robot_act 摘要面板  — total_tasks / success_count / failure_count
6. Assistant 面板      — 最终文本回复
```

当前工具来源：

- `vlm_observe`
  来自 `Robot_Module` 的视觉注册链
- `robot_act`
  来自 `Robot_Module/agent_tools.py`

## 日志抑制

- `llm_core.py` / `llm_lowlevel.py` 的 stdout 输出通过 `agent_tools.py` 的 `_StreamingBuffer` 机制转发到终端
- `Excu_Module` 的 stderr 输出（`[go2.skill]`、`[go2.speech]`）在交互模式下被抑制
- `agent_tools.py` 内部使用 `redirect_stdout` 捕获 stdout
- `interactive.py` 使用 `redirect_stderr` 抑制噪音

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
- `/tools`
- `/reset`
- `/status`
- `/vlm`
- `/quit`

## 输入输出

- 输入：终端文字指令
- 输出：TUI 面板化展示，按执行顺序渐进渲染

## 当前工作原则

- 纯对话任务应直接回复
- 涉及环境判断时优先调用 `vlm_observe`
- 涉及真实动作时调用 `robot_act`
- `robot_act` 内部会继续进入 `LLM_Module -> Robot_Module -> Excu_Module -> Comm_Module` 这条链
