# FinalProject

## 作用

这是一个基于双层 LLM 的机器人控制项目。
用户通过 `Interactive_Module` 输入自然语言指令，`LLM_Module` 负责规划与执行，`Robot_Module` 提供工具，`Sim_Module` 负责仿真显示，`VLM_Module` 负责图片描述。

## 依赖

- `openai`
- `pyyaml`
- `pygame`
- 可选：`python-dotenv`、`opencv-python`
- 项目根目录 `.env` 中需要有 `Test_API_KEY`

## 使用

常见启动方式：

```bash
python3 Sim_Module/sim2d/simulator.py
python3 Interactive_Module/interactive.py
```

## 输入输出

- 输入：用户自然语言指令、摄像头或图片、仿真环境事件
- 输出：任务规划结果、工具执行结果、仿真画面和图片描述
