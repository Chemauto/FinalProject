from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Any, Callable

import yaml


class LowLevelExecutor:
    """下层 LLM：负责根据单个任务调用工具。"""

    def __init__(self, client, model: str, prompt_path: str | None = None):
        self.client = client
        self.model = model
        root = Path(__file__).resolve().parent
        self.prompt_path = Path(prompt_path) if prompt_path else root / "prompts" / "lowlevel_prompt.yaml"

    def load_prompts(self) -> dict[str, str]:
        # Prefer project-specific prompt from registry
        from Excu_Module.skill_registry import get_lowlevel_prompt_path
        registry_path = get_lowlevel_prompt_path()
        effective_path = registry_path if registry_path else self.prompt_path

        if not effective_path.exists():
            return {
                "system_prompt": "你是一个机器人控制助手。根据子任务描述，调用相应的工具函数。",
                "user_prompt": (
                "执行任务：{task_description}\n"
                "任务类型：{task_type}\n"
                "建议函数：{suggested_function}\n"
                "规划依据：{planning_reason}\n\n"
                "参数上下文：{parameter_context}\n"
                "已计算参数：{calculated_parameters}\n\n"
                "当前视觉上下文：{visual_context}\n\n"
                "上一步的结果：{previous_result}"
            ),
            }
        data = yaml.safe_load(effective_path.read_text(encoding="utf-8")) or {}
        return {
            "system_prompt": str(data.get("system_prompt", "")).strip(),
            "user_prompt": str(data.get("user_prompt", "")).strip(),
        }

    def execute_single_task(
        self,
        task_info: dict | str,
        tools: list[dict],
        execute_tool_fn: Callable,
        previous_result: Any = None,
        visual_context: str | None = None,
    ) -> dict:
        if isinstance(task_info, dict):
            task_description = task_info.get("task", "")
            task_type = task_info.get("type", "未分类")
            suggested_function = task_info.get("function", "待LLM决定")
            planning_reason = task_info.get("reason", "未提供规划依据")
            parameter_context = task_info.get("parameter_context", {})
            calculated_parameters = task_info.get("calculated_parameters", {})
        else:
            task_description = task_info
            task_type = "未分类"
            suggested_function = "待LLM决定"
            planning_reason = "未提供规划依据"
            parameter_context = {}
            calculated_parameters = {}

        print(f"\n{'─' * 50}\n⚙️  [下层LLM执行] {task_description}\n{'─' * 50}")
        try:
            if suggested_function != "待LLM决定" and isinstance(calculated_parameters, dict) and calculated_parameters:
                function_args = dict(calculated_parameters)
                function_args.setdefault("speech", task_description)
                print(f"🧮 [参数计算] 使用已计算参数执行 {suggested_function}: {function_args}")
                result = execute_tool_fn(suggested_function, function_args)
                return self._build_execution_result(
                    result=result,
                    function_name=suggested_function,
                    task_description=task_description,
                    task_type=task_type,
                    suggested_function=suggested_function,
                    planning_reason=planning_reason,
                    response_content="",
                )

            prompts = self.load_prompts()

            user_prompt = prompts["user_prompt"].format(
                task_description=task_description,
                task_type=task_type,
                suggested_function=suggested_function,
                planning_reason=planning_reason,
                parameter_context=parameter_context if parameter_context else "无",
                calculated_parameters=calculated_parameters if calculated_parameters else "无",
                visual_context=visual_context if visual_context else "无",
                previous_result=previous_result if previous_result is not None else "无",
            )

            completion = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": prompts["system_prompt"]},
                    {"role": "user", "content": user_prompt},
                ],
                tools=tools,
                tool_choice="auto",
                extra_body={"enable_thinking": False},
            )

            response_message = completion.choices[0].message
            response_content = (response_message.content or "").strip()

            tool_calls = response_message.tool_calls
            if not tool_calls:
                print("[跳过] 无效指令或无法识别的操作")
                return {"success": False, "action": "none", "error": "No tool called"}

            tool_call = tool_calls[0]
            function_name = tool_call.function.name
            function_args = json.loads(tool_call.function.arguments)

            # Delegate parameter normalization to skill via registry
            from Excu_Module.skill_registry import get_skill
            skill = get_skill(function_name)
            if skill is not None:
                function_args, correction_message = skill.normalize_tool_arguments(
                    function_args, task_description, previous_result,
                )
            else:
                correction_message = None

            if correction_message:
                print(f"🛠️  [参数修正] {correction_message}")
            if suggested_function != "待LLM决定" and function_name != suggested_function:
                print(f"⚠️  [函数偏差] 规划建议 {suggested_function}，实际调用 {function_name}")
            print(f"🔧 [工具调用] {function_name}({function_args})")
            result = execute_tool_fn(function_name, function_args)
            return self._build_execution_result(
                result=result,
                function_name=function_name,
                task_description=task_description,
                task_type=task_type,
                suggested_function=suggested_function,
                planning_reason=planning_reason,
                response_content=response_content,
            )
        except Exception as error:
            print(f"\n❌ [错误] 执行失败: {error}")
            return {"success": False, "error": str(error), "task": task_description}

    @staticmethod
    def _build_execution_result(
        result: dict[str, Any],
        function_name: str,
        task_description: str,
        task_type: str,
        suggested_function: str,
        planning_reason: str,
        response_content: str,
    ) -> dict[str, Any]:
        if result.get("error"):
            return {
                "success": False,
                "action": function_name,
                "task": task_description,
                "task_type": task_type,
                "suggested_function": suggested_function,
                "planning_reason": planning_reason,
                "llm_message": response_content,
                "result": result,
                "feedback": {
                    "signal": "FAILURE",
                    "skill": function_name,
                    "message": result["error"],
                },
            }

        feedback = result.get("feedback", {})
        if feedback:
            signal = feedback.get("signal", "UNKNOWN")
            message = feedback.get("message", "")
            print(f"📨 [执行反馈] {signal} - {message}")

        if result and result.get("delay"):
            delay = result["delay"]
            print(f"⏳ [等待] 执行时间: {delay:.1f}秒", end="", flush=True)
            steps = max(1, int(delay))
            for _ in range(steps):
                time.sleep(delay / steps)
                print(".", end="", flush=True)
            print(" ✅ 完成!")

        return {
            "success": result.get("success", True),
            "action": function_name,
            "task": task_description,
            "task_type": task_type,
            "suggested_function": suggested_function,
            "planning_reason": planning_reason,
            "llm_message": response_content,
            "feedback": feedback,
            "result": result,
        }
