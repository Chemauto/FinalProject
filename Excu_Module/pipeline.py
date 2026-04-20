#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Excu_Module/pipeline.py — 执行流水线（两层架构：规划 → 直接执行）。"""

from __future__ import annotations

import threading
from typing import Any, Callable

PipelineEventCallback = Callable[[str, dict[str, Any]], None]

_PROGRESS_POLL_SEC = 1.0


def _execute_with_progress(
    execute_tool_fn: Callable,
    function_name: str,
    params: dict,
    on_event: PipelineEventCallback,
) -> dict[str, Any]:
    """在后台线程执行技能，主线程每 2s 轮询机器人坐标并通过 on_event 回传。"""
    result_box: dict[str, Any] = {}
    error_box: dict[str, BaseException] = {}

    def _worker():
        try:
            result_box["value"] = execute_tool_fn(function_name, params)
        except BaseException as exc:
            error_box["value"] = exc

    thread = threading.Thread(target=_worker, daemon=True)
    thread.start()

    while thread.is_alive():
        thread.join(timeout=_PROGRESS_POLL_SEC)
        if thread.is_alive():
            try:
                from Hardware_Module import get_state
                from Excu_Module.state import summarize_state
                state = get_state()
                if state and isinstance(state, dict):
                    summary = summarize_state(state) or {}
                    on_event("step_progress", {"pose": summary.get("robot_pose")})
            except Exception:
                pass

    if "value" in error_box:
        raise error_box["value"]
    return result_box.get("value", {})


def assess_result(result: dict[str, Any]) -> tuple[bool, str]:
    """评估单步执行结果。"""
    feedback = result.get("feedback") or {}
    validation = _extract_execution_validation(result)

    if validation:
        summary = str(validation.get("summary") or feedback.get("message") or "").strip()
        verified = validation.get("verified")
        meets_requirements = validation.get("meets_requirements")

        if verified is True and meets_requirements is True:
            return True, summary or "动作已通过真实状态校验"
        if verified is False:
            return False, summary or "未获取到真实执行数据，无法确认动作是否满足要求"
        if meets_requirements is False:
            return False, summary or "动作未满足任务要求"

    signal = str(feedback.get("signal") or "").upper()
    message = str(feedback.get("message") or result.get("error") or "当前步骤执行失败").strip()
    return bool(result.get("success")) and signal == "SUCCESS", message


def _extract_execution_validation(result: dict[str, Any]) -> dict[str, Any]:
    feedback = result.get("feedback")
    if isinstance(feedback, dict):
        validation = feedback.get("validation")
        if isinstance(validation, dict):
            return validation

    tool_output = result.get("result")
    if isinstance(tool_output, dict):
        raw_result = tool_output.get("result")
        if isinstance(raw_result, dict):
            execution_feedback = raw_result.get("execution_feedback")
            if isinstance(execution_feedback, dict):
                validation = execution_feedback.get("validation")
                if isinstance(validation, dict):
                    return validation
    return {}


def _build_task_understanding(
    user_input: str,
    tasks: list[dict],
    planner_summary: str,
) -> str:
    if not tasks:
        return ""
    summary = planner_summary or "已完成任务规划"
    function_chain = " -> ".join(task.get("function", "待定") for task in tasks)
    primary_reason = tasks[0].get("reason", "")
    return "\n".join(
        [
            f"环境判断: {_build_environment_judgment(tasks)}",
            f"任务目标: {user_input}",
            f"技能决策: {summary}",
            f"选择原因: {primary_reason}",
            f"函数链: {function_chain}",
        ]
    )


def _build_environment_judgment(tasks: list[dict]) -> str:
    if not tasks:
        return "未获取到有效环境判断"
    first_task = tasks[0].get("task", "")
    if "。先" in first_task:
        return first_task.split("。先", 1)[0]
    if "，先" in first_task:
        return first_task.split("，先", 1)[0]
    return first_task


def run_pipeline(
    user_input: str,
    planner,
    execute_tool_fn: Callable,
    tools: list[dict] | None = None,
    visual_context: str | None = None,
    scene_facts: dict[str, Any] | None = None,
    object_facts: dict[str, Any] | None = None,
    agent_thought: str = "",
    robot_state: dict[str, Any] | None = None,
    max_replans: int = 0,
    on_event: PipelineEventCallback | None = None,
) -> list[dict]:
    """编排 plan -> parameter -> execute 的完整流水线。

    两层架构：高层 LLM 直接输出函数名+参数 → pipeline 直接执行。
    """
    if on_event:
        on_event("plan_start", {"user_input": user_input})
    else:
        print("\n" + "█" * 60 + f"\n📥 [用户输入] {user_input}\n" + "█" * 60)
    try:
        results: list[dict[str, Any]] = []
        plan_input = user_input

        for attempt in range(max_replans + 1):
            tasks, plan_meta = planner.plan_tasks(
                plan_input, agent_thought, tools or [], visual_context,
                scene_facts=scene_facts,
                object_facts=object_facts,
                robot_state=robot_state,
                on_event=on_event,
            )
            if not tasks:
                break

            active_tasks = planner.annotate_tasks(tasks, object_facts)
            task_understanding = _build_task_understanding(
                user_input, active_tasks, planner.last_summary,
            )
            if task_understanding:
                thought_prefix = f"顶层思考: {agent_thought}\n" if agent_thought else ""
                thinking_text = f"{thought_prefix}{task_understanding}"
                if on_event:
                    on_event("plan_thinking", {"thinking": thinking_text})
                else:
                    print("\n" + "─" * 60 + f"\n👁️ [LLM思考]\n{thinking_text}\n" + "─" * 60)

            if on_event:
                on_event("execute_start", {})
            else:
                print("\n" + "█" * 60 + "\n🤖 [开始执行]\n" + "█" * 60)
            should_replan = False

            for idx, task in enumerate(active_tasks, start=1):
                function_name = task.get("function", "")
                params = task.get("params") or {}
                task_desc = task.get("task", "")

                # 确保 speech 参数
                params.setdefault("speech", task_desc)

                if on_event:
                    on_event("step_start", {
                        "idx": idx,
                        "total": len(active_tasks),
                        "function": function_name,
                        "params": params,
                        "reason": task.get("reason", ""),
                    })
                else:
                    print(f"\n【步骤 {idx}/{len(active_tasks)}】")
                    print(f"📌 [函数] {function_name}")
                    print(f"📝 [参数] {params}")
                    print(f"📋 [依据] {task.get('reason', '')}")

                # 直接调用技能函数（有 on_event 时在后台线程执行，主线程轮询坐标）
                if on_event:
                    result = _execute_with_progress(execute_tool_fn, function_name, params, on_event)
                else:
                    result = execute_tool_fn(function_name, params)

                task_success, assessment_message = assess_result(result)
                result["success"] = task_success
                result["action"] = function_name
                result["task"] = task_desc
                if assessment_message:
                    result["assessment_message"] = assessment_message
                results.append(result)

                if task_success:
                    if on_event:
                        on_event("step_done", {"idx": idx, "success": True, "message": assessment_message or "步骤执行成功"})
                    elif assessment_message:
                        print(f"✅ [结果校验] {assessment_message}")
                    continue

                failure_message = (
                    assessment_message
                    or (result.get("feedback") or {}).get("message")
                    or result.get("error")
                    or "当前步骤执行失败"
                )
                if attempt < max_replans:
                    if on_event:
                        on_event("replan", {"message": failure_message})
                    else:
                        print(f"\n🔁 [重规划] {failure_message}")
                    plan_input = f"{user_input}\n上次执行失败：{failure_message}"
                    should_replan = True
                else:
                    if on_event:
                        on_event("step_done", {"idx": idx, "success": False, "message": failure_message})
                    else:
                        print(f"\n⚠️  [中止] {failure_message}")
                break

            if not should_replan:
                break

        if on_event:
            on_event("pipeline_done", {"results": results})
        else:
            print("\n" + "█" * 60 + "\n✅ [执行完成] 任务总结\n" + "█" * 60)
            for idx, result in enumerate(results, 1):
                status = "✅ 成功" if result.get("success") else "❌ 失败"
                action = result.get("action", "未调用")
                task_label = result.get("task", "未记录任务")
                print(f"  {idx}. {task_label} -> {action} - {status}")
        return results
    except Exception as error:
        if on_event:
            on_event("pipeline_error", {"error": f"{type(error).__name__}: {error}"})
        else:
            print(f"\n❌ [错误] {type(error).__name__}: {error}")
            import traceback
            traceback.print_exc()
        return []
