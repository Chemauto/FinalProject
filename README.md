# LLMPlanner

LLMPlanner 是一个面向腿足机器人 IsaacLab 场景的自然语言任务规划系统。它将用户指令转换为结构化机器人技能调用，通过 WebSocket 技能桥执行任务，并在失败或超时时触发重规划。本项目只负责高层任务规划与技能调度，不直接实现底层运动控制。仿真器脚本、训练好的策略模型和机器人侧技能执行服务请先克隆：

```bash
git clone https://github.com/Chemauto/llm-legged-lab.git
```

## 功能特点

- 通过终端 TUI 输入自然语言任务。
- 结合 VLM 感知结果和 WebSocket 机器人状态理解环境。
- 使用大语言模型进行任务规划，并通过 tool calling 输出技能队列。
- 支持队列式执行：先观察，再一次性规划完整动作序列。
- 技能命令采用 `action_id / skill / args` 结构。
- 支持场景 `0~4` 的自动导航评测。
- 支持按场景统计成功率、耗时和误差，并生成论文图。

## 系统架构

```text
用户指令 -> TUI -> Planner -> Vision -> Executor
        -> WebSocket / ROS2 / IsaacLab 技能服务端
        -> 状态反馈与失败重规划
```

- `Planner/`：提示词加载、大模型调用、tool calls 解析。
- `Vision/`：视觉观察、机器人状态读取、结构化场景事实融合。
- `Executor/`：工具注册、技能封装、WebSocket 通信和队列执行。
- `Tui/`：终端交互、会话状态和运行时调度。
- `scripts/`：自动评测、结果汇总和图表生成。
- `eval_targets/`：导航评测的固定真值目标。

## 安装依赖

```bash
cd LLMPlanner
pip install -r requirements.txt
```

## 环境变量

```env
可以让AI阅读Project.md
MODEL_API_KEY=你的模型API Key
MODEL_BASE_URL=https://你的OpenAI兼容接口/v1
ROBOT_WS_URL=ws://127.0.0.1:8765
于Vision/vlm.py,修改视觉语言模型
```

## 启动仿真器

```bash
git clone https://github.com/Chemauto/llm-legged-lab.git
cd llm-legged-lab
```

然后按照该仓库说明启动 IsaacLab 仿真器和 WebSocket 技能服务。LLMPlanner 默认连接地址为 `ws://127.0.0.1:8765`。

## 运行 LLMPlanner

```bash
cd LLMPlanner
conda activate ros2_env
python Tui/tui.py
```

```text
/connect
导航到目标点
```

常用命令：`/help`、`/connect`、`/load`、`/history`、`/reset`、`/quit`。

## 技能接口

主要技能包括：`observe` 观察场景，`nav` 导航，`walk_skill` 行走，`push` 推箱，`climb` 攀爬。

WebSocket 命令示例：

```json
{"type": "command", "action_id": "nav-xxxx", "skill": "nav", "args": {"x": 4.0, "y": 0.0, "z": 0.3}}
```

## 自动评测

```bash
python scripts/evaluate_navigation.py --target-json eval_targets/navigation_targets.json --scenes 0 1 2 3 4 --trials 25 --instruction "导航到目标点" --threshold 0.3
python scripts/summarize_navigation_eval.py eval_results/navigation_eval_xxx.md
```

## 开源协议

本项目用于科研和教学。公开发布前请在仓库中补充 `LICENSE` 文件；仿真器脚本和训练好的模型需遵循 `https://github.com/Chemauto/llm-legged-lab` 的开源协议。
