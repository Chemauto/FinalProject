# LLM_Module

## 作用

`LLM_Module/llm_core.py` 是双层 LLM 的统一入口。
`LLM_Module/llm_highlevel.py` 负责把用户指令拆成子任务。
`LLM_Module/llm_lowlevel.py` 负责根据子任务选择工具并执行。

## 提示词

- `LLM_Module/prompts/highlevel_prompt.yaml`：上层规划提示词
- `LLM_Module/prompts/lowlevel_prompt.yaml`：下层执行提示词

## 依赖

- `openai`
- `pyyaml`
- 项目根目录 `.env` 中需要有 `Test_API_KEY`

## 使用

通常由 `Interactive_Module/interactive.py` 调用：

```bash
python3 Interactive_Module/interactive.py
```

## 输入输出

- 输入：用户自然语言指令、可用工具列表、上一步执行结果
- 输出：上层输出任务序列，下层输出工具调用结果
