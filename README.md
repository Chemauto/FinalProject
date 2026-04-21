# FinalProject

FinalProject 当前处于重构阶段。旧项目已迁移到：

```text
/home/robot/work/backup
```

当前目录保留重构说明、文件夹级说明文档和最小可运行的 LLM 机器人任务规划框架。

## 当前功能

用户在 TUI 输入自然语言任务，LLM 通过 tool calling 选择并调用技能（Nav / walk / Push / climb），Executor 执行技能并实时显示状态。

- `Planner/llm_core.py`：调用 LLM，支持 tool calling 和流式输出。
- `Planner/prompts/planner_prompt.yaml`：保存模型名、系统提示词、默认用户提示词。
- `Executor/tools.py`：用 FastMCP 注册 4 个技能，提供 tool definitions 给 LLM。
- `Executor/executor.py`：根据 LLM 返回的 tool_calls 逐个执行技能。
- `Executor/skills.py`：手动实现 `Nav`、`walk`、`Push`、`climb`。
- `Executor/state.py`：保存假机器人状态。
- `Executor/demo_executor.py`：固定步骤 demo，用于不调用 LLM 的本地测试。
- `Tui/tui.py`：终端交互主循环，普通输入走规划执行，`/demo` 走固定步骤。
- `Tui/session.py`：保存 LLM `messages` 和 UI `chat_items`。
- `Tui/render.py`：Rich 显示，增量渲染不重复打印。
- `Tui/commands.py`：处理 slash commands。
- `Tui/gateway.py`：本地网关，隔离 TUI 和 LLM 调用。
- `Tui/stream.py`：维护流式消息状态。
- `Tui/history.py`：保存和恢复最近一次 TUI 会话。

## 目录结构

```text
FinalProject/
├── CLAUDE.md
├── Codex.md
├── README.md
├── Planner/
│   ├── README.md
│   ├── llm_core.py
│   └── prompts/
│       └── planner_prompt.yaml
├── Executor/
│   ├── README.md
│   ├── __init__.py
│   ├── state.py
│   ├── skills.py
│   ├── demo_executor.py
│   ├── executor.py
│   └── tools.py
└── Tui/
    ├── README.md
    ├── commands.py
    ├── gateway.py
    ├── history.py
    ├── render.py
    ├── session.py
    ├── stream.py
    └── tui.py
```

## 环境变量

```env
MODEL_API_KEY=你的API Key
MODEL_BASE_URL=OpenAI兼容接口地址
```

当前代码会自动读取 `.env`。

## 安装依赖

```bash
pip install openai python-dotenv pyyaml rich prompt_toolkit mcp
```

## 运行

```bash
cd Tui
python tui.py
```

## TUI 命令

```text
/help    查看帮助
/demo    演示固定步骤执行（不调用LLM）
/load    恢复最近会话
/history 显示历史文件路径
/reset   清空当前上下文
/quit    退出
```

普通输入会调用 LLM，LLM 通过 tool calling 选择技能并自动执行。

会话历史自动保存到 `.tui_history/latest.json`，该目录不会上传到 git。

## 当前架构

```text
用户输入
-> make_plan(messages) 带工具定义调 LLM
-> LLM 返回 tool_calls 或文本
-> 如果 tool_calls: run_plan() 逐个执行技能
-> 如果文本: 直接显示
-> 通过 emit 实时显示到 TUI
```

```text
/demo 命令
-> run_demo_task(emit) 固定步骤
-> 不调用 LLM
```

## 机器人技能

通过 FastMCP 注册，LLM 通过 tool calling 调用：

```text
nav(x, y, z)              0.5m/s直线导航
walk_skill(direction, v)   按方向和速度移动
push(x, y, z)             0.2m/s推动箱子
climb(height)              0.1m/s攀爬，最高0.3m
```

执行状态约 5Hz 输出一次（每 0.2s 一次），通过 `update_last_status` 覆盖最后一条状态，不刷屏。

## 设计原则

- 代码保持简洁，先实现最小可运行链路。
- TUI 只负责交互和显示。
- Planner 只负责 prompt 加载和 LLM 调用。
- 技能通过 MCP 注册，LLM 通过 tool calling 选择，不在 prompt 里写技能说明。
- 后续重构参考 `Codex.md`，逐步恢复机器人规划、感知、执行模块。
