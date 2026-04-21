# Tui

## 作用

`Tui` 是当前项目的终端交互层。

它负责：

- 读取用户输入
- 处理命令
- 调用 LLM
- 调用 demo 执行层
- 显示聊天和机器人事件
- 保存/恢复最近会话

## 文件

```text
tui.py
commands.py
gateway.py
history.py
render.py
session.py
stream.py
```

## 运行

```bash
cd /home/robot/work/FinalProject/Tui
python tui.py
```

## 命令

```text
/help     查看帮助
/demo     演示机器人事件
/load     恢复最近会话
/history  显示历史路径
/reset    清空上下文
/quit     退出
```

## 主要模块

`tui.py` 是主循环。

`commands.py` 解析 slash commands。

`gateway.py` 隔离 TUI 和 LLM 调用。

`history.py` 保存和加载 `.tui_history/latest.json`。

`render.py` 负责 Rich 显示。

`session.py` 保存：

```text
messages   给 LLM 用
chat_items 给 TUI 显示用
```

`stream.py` 保存 LLM 流式输出状态。

## 事件显示

支持的 `chat_items` 类型：

```text
user
assistant
system
command
plan
tool
status
error
```

`status` 会覆盖最后一条状态，避免 5Hz 状态刷屏。

## 当前边界

`Tui` 只负责交互和显示。

机器人技能在 `Executor`。

LLM 调用在 `Planner`。
