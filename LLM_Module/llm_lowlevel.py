from __future__ import annotations

import json
import re
import sys
import time
from pathlib import Path
from typing import Any, Callable

import yaml


class LowLevelExecutor:
    """下层 LLM：负责根据单个任务调用工具。"""
    CLIMB_LIMIT_METERS = 0.3

    def __init__(self, client, model: str, prompt_path: str | None = None):
        self.client = client
        self.model = model
        root = Path(__file__).resolve().parent
        self.prompt_path = Path(prompt_path) if prompt_path else root / "prompts" / "lowlevel_prompt.yaml"

    def load_prompts(self) -> dict[str, str]:
        if not self.prompt_path.exists():
            return {
                "system_prompt": "你是一个机器人控制助手。根据子任务描述，调用相应的工具函数。",
                "user_prompt": (
                    "执行任务：{task_description}\n"
                    "任务类型：{task_type}\n"
                    "建议函数：{suggested_function}\n"
                    "规划依据：{planning_reason}\n\n"
                    "当前视觉上下文：{visual_context}\n\n"
                    "上一步的结果：{previous_result}"
                ),
            }
        data = yaml.safe_load(self.prompt_path.read_text(encoding="utf-8")) or {}
        return {
            "system_prompt": str(data.get("system_prompt", "")).strip(),
            "user_prompt": str(data.get("user_prompt", "")).strip(),
        }

    @staticmethod
    def _to_float(value: Any) -> float | None:
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _extract_numeric_values(text: str, unit: str = "米") -> list[float]:
        if not text:
            return []
        return [float(v) for v in re.findall(rf"(\d+(?:\.\d+)?)\s*{unit}", text)]

    @classmethod
    def _extract_previous_numeric_value(
        cls,
        previous_result: Any,
        candidate_keys: tuple[str, ...],
    ) -> float | None:
        if not isinstance(previous_result, dict):
            return None

        for key in candidate_keys:
            value = cls._to_float(previous_result.get(key))
            if value is not None and value > 0:
                return value
        return None

    @classmethod
    def _normalize_relative_numeric_argument(
        cls,
        requested: float,
        task_description: str,
        previous_result: Any,
        upper_bound: float,
        context_keyword: str | None = None,
        candidate_keys: tuple[str, ...] = (),
    ) -> float | None:
        if requested <= upper_bound:
            return None

        numeric_values = cls._extract_numeric_values(task_description)
        previous_value = cls._extract_previous_numeric_value(previous_result, candidate_keys)
        candidate_value = None

        if context_keyword and context_keyword not in task_description:
            return None

        if numeric_values:
            target_value = max(numeric_values)

            if previous_value is not None and target_value > previous_value:
                remaining_value = round(target_value - previous_value, 3)
                if 0 < remaining_value <= upper_bound:
                    candidate_value = remaining_value

            if candidate_value is None and len(numeric_values) >= 2:
                base_value = min(numeric_values)
                remaining_value = round(max(numeric_values) - base_value, 3)
                if 0 < remaining_value <= upper_bound:
                    candidate_value = remaining_value

        return candidate_value

    @classmethod
    def _normalize_tool_arguments(
        cls,
        function_name: str,
        function_args: dict[str, Any],
        task_description: str,
        previous_result: Any,
    ) -> tuple[dict[str, Any], str | None]:
        if function_name != "climb":
            return function_args, None

        requested = cls._to_float(function_args.get("height"))
        if requested is None:
            return function_args, None

        candidate_value = cls._normalize_relative_numeric_argument(
            requested=requested,
            task_description=task_description,
            previous_result=previous_result,
            upper_bound=cls.CLIMB_LIMIT_METERS,
            context_keyword="箱子",
            candidate_keys=("height", "box_height"),
        )
        if candidate_value is None:
            return function_args, None

        repaired_args = dict(function_args)
        repaired_args["height"] = candidate_value
        message = (
            f"{function_name} 参数 height 从 {requested:.2f} 米修正为 {candidate_value:.2f} 米，"
            f"按单步相对高度执行（上限 {cls.CLIMB_LIMIT_METERS:.2f} 米）"
        )
        return repaired_args, message

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
        else:
            task_description = task_info
            task_type = "未分类"
            suggested_function = "待LLM决定"
            planning_reason = "未提供规划依据"

        print(f"\n{'─' * 50}\n⚙️  [下层LLM执行] {task_description}\n{'─' * 50}")
        try:
            prompts = self.load_prompts()
            # if previous_result is not None:
            #     print(f"[LLM] 上一步结果: {previous_result}", file=sys.stderr)
            # else:
            #     print(f"[LLM] 没有上一步结果", file=sys.stderr)

            user_prompt = prompts["user_prompt"].format(
                task_description=task_description,
                task_type=task_type,
                suggested_function=suggested_function,
                planning_reason=planning_reason,
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
            # if response_content:
            #     print(f"💭 [执行说明] {response_content}")

            tool_calls = response_message.tool_calls
            if not tool_calls:
                print("[跳过] 无效指令或无法识别的操作")
                return {"success": False, "action": "none", "error": "No tool called"}

            tool_call = tool_calls[0]
            function_name = tool_call.function.name
            function_args = json.loads(tool_call.function.arguments)
            function_args, correction_message = self._normalize_tool_arguments(
                function_name=function_name,
                function_args=function_args,
                task_description=task_description,
                previous_result=previous_result,
            )
            if correction_message:
                print(f"🛠️  [参数修正] {correction_message}")
            if suggested_function != "待LLM决定" and function_name != suggested_function:
                print(f"⚠️  [函数偏差] 规划建议 {suggested_function}，实际调用 {function_name}")
            print(f"🔧 [工具调用] {function_name}({function_args})")
            result = execute_tool_fn(function_name, function_args)
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
        except Exception as error:
            print(f"\n❌ [错误] 执行失败: {error}")
            return {"success": False, "error": str(error), "task": task_description}
