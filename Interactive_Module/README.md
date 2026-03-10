# Interactive_Module

## 作用

`Interactive_Module/interactive.py` 是整个项目的命令行交互入口。
它负责接收用户指令，调用 `LLM_Module` 做任务规划与执行，再调用 `Robot_Module` 的工具完成动作。
`Interactive_Module/voice_interactive.py` 提供语音输入入口，先把语音转文字，再复用同样的执行流程。

## 依赖

- `openai`
- `pyyaml`
- 可选：`python-dotenv`
- 系统录音命令 `arecord`
- 项目根目录 `.env` 中需要有 `Test_API_KEY`

## 使用

在项目根目录运行：

```bash
python3 Interactive_Module/interactive.py
python3 Interactive_Module/voice_interactive.py
```

## 输入输出

- 输入：终端文字指令或麦克风语音
- 输出：识别文本、任务规划、工具调用和执行结果
