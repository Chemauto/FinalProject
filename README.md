# FinalProject

FinalProject 当前处于重构阶段。旧项目已迁移到：

```text
/home/robot/work/backup
```

当前目录保留重构说明、文件夹级说明文档和最小可运行的 LLM 对话框架。

## 当前功能

- `Planner/llm_core.py`：读取 prompt 和 `.env`，调用 OpenAI 兼容 LLM API。
- `Planner/prompts/planner_prompt.yaml`：保存模型名、系统提示词、默认用户提示词。
- `Tui/tui.py`：终端对话 UI，支持上下文和流式输出。
- `Tui/session.py`：保存 LLM `messages` 和 UI `chat_items`。
- `Tui/gateway.py`：本地网关，隔离 TUI 和 LLM 调用。
- `Tui/commands.py`：处理 `/help`、`/load`、`/history`、`/reset`、`/quit`。
- `Tui/render.py`：负责 Rich UI 显示和完整聊天记录渲染。
- `Tui/stream.py`：维护简版流式消息状态。
- `Tui/history.py`：保存和恢复最近一次 TUI 会话。
- `Executor/state.py`：保存手动技能使用的假机器人状态。
- `Executor/skills.py`：手动实现 `Nav`、`walk`、`Push`、`climb`。
- `Executor/demo_executor.py`：执行 demo 任务并向 TUI 发送事件。
- `Planner/README.md`：Planner 文件夹说明。
- `Tui/README.md`：Tui 文件夹说明。
- `Executor/README.md`：Executor 文件夹说明。
- `Codex.md`：旧项目完整分析和后续重构路线。

## 目录结构

```text
FinalProject/
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
│   └── demo_executor.py
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
pip install openai python-dotenv pyyaml rich prompt_toolkit
```

## 运行

```bash
cd Tui
python tui.py
```

## TUI 命令

```text
/help   查看帮助
/demo   演示机器人事件显示
/load   恢复最近会话
/history 显示历史文件路径
/reset  清空当前上下文
/quit   退出
```

会话历史自动保存到 `.tui_history/latest.json`，该目录不会上传到 git。

## 当前架构

```text
Tui/tui.py
-> Tui/commands.py
-> Tui/session.py
-> Executor/demo_executor.py
-> Tui/gateway.py
-> Planner/llm_core.py
-> LLM API
```

`Tui` 不直接创建模型客户端，只通过 `gateway.py` 调用 `llm_core.py`。

## 机器人事件

执行层通过 `emit(type, content)` 向 TUI 发送事件：

```text
plan    任务计划
tool    工具调用，如 Nav / walk / Push / climb
status  执行状态
error   错误信息
```

`/demo` 会演示一个障碍场景：机器人从 `(0,0)` 去 `(0,8)`，中间有 0.5m 障碍，`climb` 最高 0.3m，因此先 `Push` 箱子辅助，再 `climb`，最后继续 `Nav`。

当前 `/demo` 已经不是单纯打印文本，而是调用手动技能并更新假状态：

```text
Nav(x, y, z)       0.5m/s直线导航
walk(direction, v) 按速度v移动，front增加x
Push(x, y, z)      0.2m/s推动箱子
climb(height)      0.1m/s攀爬，最高0.3m
```

执行状态约5Hz输出一次，即每0.2s一次。例如 `walk("front", 0.2)` 会在0.2s后到 `robot=(0.04, 0, 0)`，1.0s后到 `robot=(0.2, 0, 0)`。

## 设计原则

- 代码保持简洁，先实现最小可运行链路。
- TUI 只负责交互和显示。
- Planner 只负责 prompt 加载和 LLM 调用。
- 后续重构参考 `Codex.md`，逐步恢复机器人规划、感知、执行模块。
