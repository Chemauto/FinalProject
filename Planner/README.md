# Planner

## 作用

`Planner` 目前负责 LLM 调用和提示词配置。

当前它还没有真正生成机器人任务计划，只是提供最小可运行的 LLM API 入口。

## 文件

```text
llm_core.py
prompts/planner_prompt.yaml
```

## llm_core.py

负责：

- 读取 `.env`
- 读取 `planner_prompt.yaml`
- 创建 OpenAI 兼容客户端
- 提供 `chat(messages)`
- 提供 `stream_chat(messages)`

使用的环境变量：

```text
MODEL_API_KEY
MODEL_BASE_URL
```

## prompts/planner_prompt.yaml

保存：

- `model`
- `system_prompt`
- `user_prompt`

当前默认模型是：

```text
glm-5.1
```

## 当前边界

`Planner` 只负责 LLM 调用。

它不负责：

- TUI 显示
- 技能执行
- 状态反馈
- ROS2 对接

后续可以在这里加入结构化任务规划，让 LLM 输出 `Nav / walk / Push / climb` 的步骤列表。
