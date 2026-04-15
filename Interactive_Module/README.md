# Interactive_Module

## 作用

`Interactive_Module/interactive.py` 是整个项目的统一终端交互入口。
它现在使用 `rich` 构建 TUI，负责：

- 显示欢迎面板、状态栏和 slash 命令
- 接收用户任务输入
- 以“机器人智能体”身份决定直接回复，还是调用技能
- 最外层只暴露两个高层技能：`vlm_observe` 和 `robot_act`
- 展示工具调用结果和最终回复

当前只保留文字 TUI 入口，不再提供语音交互模式。

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
- 输出：TUI 面板化展示的工具调用结果与最终回复
