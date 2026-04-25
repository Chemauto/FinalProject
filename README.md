# FinalProject

FinalProject 当前处于重构阶段。旧项目已迁移到：

```text
/home/robot/work/backup
```

当前目录保留重构说明、文件夹级说明文档和最小可运行的 LLM 机器人任务规划框架。

## 当前功能

用户在 TUI 输入自然语言任务，LLM 通过 tool calling 选择并调用观察或动作技能。Vision 负责环境观察，Executor 通过 WebSocket 启动机器人技能并接收服务器状态和反馈。

- `Planner/llm_core.py`：调用 LLM，支持 tool calling 和流式输出。
- `Planner/prompts/planner_prompt.yaml`：保存模型名、系统提示词、默认用户提示词。
- `Vision/`：调用 VLM 观察环境，输出 `visual_context` 和 `scene_facts`。
- `Executor/tools.py`：用 FastMCP 注册观察和动作技能，提供 tool definitions 给 LLM。
- `Executor/executor.py`：根据 LLM 返回的 tool_calls 逐个执行技能。
- `Executor/skills.py`：发送 WebSocket 技能启动信号。
- `Executor/robot_ws.py`：发送技能命令，接收服务器状态流和最终反馈。
- `Executor/state.py`：保存服务器传来的最新状态和反馈。
- `Tui/tui.py`：终端交互主循环，普通输入走规划执行，
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
├── Vision/
│   ├── __init__.py
│   ├── image_source.py
│   ├── vlm.py
│   ├── vlm_utils.py
│   └── prompts/
│       └── VlmPrompt.yaml
├── Executor/
│   ├── README.md
│   ├── __init__.py
│   ├── robot_ws.py
│   ├── state.py
│   ├── skills.py
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
ROBOT_WS_URL=ws://127.0.0.1:8765
ROBOT_WS_TIMEOUT_SEC=60
VISION_IMAGE_PATH=/path/to/current_camera.png
VISION_MODEL=qwen3-vl-flash-2026-01-22
```

当前代码会自动读取 `.env`。

## 安装依赖

```bash
pip install openai python-dotenv pyyaml rich prompt_toolkit mcp websockets
```

## 运行

```bash
cd Tui
python tui.py
```

## TUI 命令

```text
/help    查看帮助
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
-> LLM 返回 observe/nav/walk_skill/push/climb 或文本
-> 如果 tool_calls: run_plan() 逐个执行工具
-> observe 调用 Vision 获取 scene_facts
-> 动作技能通过 WebSocket 发启动信号
-> 服务器持续推送 state，最终返回 feedback
-> 如果文本: 直接显示
-> 通过 emit 实时显示到 TUI，并把结果摘要喂回 LLM
```

## WebSocket 协议

客户端发送技能启动消息：

```json
{"type": "command", "action_id": "nav-xxxx", "skill": "nav", "args": {"x": 5, "y": 0, "z": 0}}
```

服务器持续推送状态：

```json
{"type": "state", "robot": {"x": 1.2, "y": 0, "z": 0}, "box": {"x": 0, "y": 1, "z": 0}, "current_skill": "nav"}
```

服务器返回最终反馈：

```json
{"type": "feedback", "action_id": "nav-xxxx", "skill": "nav", "signal": "SUCCESS", "message": "arrived"}
```



## 机器人技能

通过 FastMCP 注册，LLM 通过 tool calling 调用：

```text
observe(image_path="")       观察环境，返回视觉语义和scene_facts
nav(x, y, z)              0.5m/s直线导航
walk_skill(direction, v)   按方向和速度移动
push(x, y, z)             0.2m/s推动箱子
climb(height)              0.1m/s攀爬，最高0.3m
```

执行状态由 WebSocket 服务器推送，通过 `update_last_status` 覆盖最后一条状态，不刷屏。

## 设计原则

- 代码保持简洁，先实现最小可运行链路。
- TUI 只负责交互和显示。
- Planner 只负责 prompt 加载和 LLM 调用。
- 技能通过 MCP 注册，LLM 通过 tool calling 选择，不在 prompt 里写技能说明。
- 后续重构参考 `Codex.md`，逐步恢复机器人规划、感知、执行模块。
