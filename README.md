# FinalProject

FinalProject 当前处于重构阶段。旧项目已迁移到：

```text
/home/robot/work/backup
```

当前目录保留重构说明和最小可运行的 LLM 对话框架。

## 当前功能

- `Planner/llm_core.py`：读取 prompt 和 `.env`，调用 OpenAI 兼容 LLM API。
- `Planner/prompts/planner_prompt.yaml`：保存模型名、系统提示词、默认用户提示词。
- `Tui/tui.py`：终端对话 UI，支持上下文和流式输出。
- `Tui/session.py`：保存当前会话 `messages`。
- `Tui/gateway.py`：本地网关，隔离 TUI 和 LLM 调用。
- `Tui/commands.py`：处理 `/help`、`/reset`、`/quit`。
- `Tui/render.py`：负责 Rich UI 显示。
- `Codex.md`：旧项目完整分析和后续重构路线。

## 目录结构

```text
FinalProject/
├── Codex.md
├── README.md
├── Planner/
│   ├── llm_core.py
│   └── prompts/
│       └── planner_prompt.yaml
└── Tui/
    ├── commands.py
    ├── gateway.py
    ├── render.py
    ├── session.py
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
/reset  清空当前上下文
/quit   退出
```

## 当前架构

```text
Tui/tui.py
-> Tui/commands.py
-> Tui/session.py
-> Tui/gateway.py
-> Planner/llm_core.py
-> LLM API
```

`Tui` 不直接创建模型客户端，只通过 `gateway.py` 调用 `llm_core.py`。

## 设计原则

- 代码保持简洁，先实现最小可运行链路。
- TUI 只负责交互和显示。
- Planner 只负责 prompt 加载和 LLM 调用。
- 后续重构参考 `Codex.md`，逐步恢复机器人规划、感知、执行模块。
