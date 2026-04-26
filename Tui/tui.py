from pathlib import Path
import sys
import json
import logging
import re
from prompt_toolkit import prompt as input_prompt
from prompt_toolkit.history import FileHistory
from rich.console import Console

sys.path.append(str(Path(__file__).resolve().parents[1]))

from Executor.executor import run_plan
from Executor.robot_ws import check_connection
from Tui.commands import handle_command
from Tui.gateway import LocalChatGateway
from Tui.history import history_path, load_history, save_history
from Tui.render import render_chat, render_item, show_help, show_status, show_welcome
from Tui.session import add_command, add_error, add_system, add_user, chat_items, connection_status, emit, messages, reset, update_connection_status, update_last_status
from Tui.stream import StreamItem
from Planner.llm_core import prompt, make_plan

gateway = LocalChatGateway()
console = Console()
history = FileHistory(str(Path.home() / ".finalproject_tui_history"))
logging.getLogger("httpx").setLevel(logging.WARNING)
#创建本地网关

def summarize_plan_results(plan_results, latest_state, latest_feedback):
    payload = {
        "steps": plan_results or [],
        "latest_state": latest_state,
        "latest_feedback": latest_feedback,
    }
    return json.dumps(payload, ensure_ascii=False)
#把工具执行结果压缩成一条消息，交给LLM继续判断


def format_connection_result(result):
    signal = result.get("signal") or "UNKNOWN"
    lines = [f"连接结果: {signal}", f"地址: {result.get('ws_url', 'unknown')}"]
    message = result.get("message")
    if message:
        lines.append(f"消息: {message}")
    lines.append(f"状态源就绪: {'yes' if result.get('status_json_ready') else 'no'}")
    if result.get("current_skill"):
        lines.append(f"当前技能: {result.get('current_skill')}")
    if result.get("model_use") is not None:
        lines.append(f"model_use: {result.get('model_use')}")
    if result.get("start") is not None:
        lines.append(f"start: {result.get('start')}")
    return "\n".join(lines)
#把健康检查结果整理成TUI里易读的文本

ACTION_TOOLS = {"nav", "nav_climb", "walk_skill", "push", "climb"}

def has_action_tool(tool_calls):
    return any(tc["name"] in ACTION_TOOLS for tc in tool_calls)
#判断本轮是否包含动作工具

def incomplete_tool_batch(content, tool_calls):
    steps = re.findall(r"(?m)^\s*\d+[.、)]\s*", str(content or ""))
    return len(steps) > len(tool_calls) and has_action_tool(tool_calls)
#正文列了多步动作但tool_calls不足时，要求LLM补齐工具调用

def short_status(content):
    text = str(content).replace("\n", " ")
    limit = max(20, console.width - 10)
    return text if len(text) <= limit else text[:limit - 1] + "…"
#状态行过长时截断

show_welcome(console, prompt["model"], connection_status)

while True:
    show_status(console, messages, connection_status)
    #进入主循环，每轮显示状态信息
    user_input = input_prompt("You> ", history=history).strip()
    #读取输入
    command = handle_command(user_input, reset)
    if command["type"] == "quit":
        add_system(command["message"])
        render_chat(console, chat_items, prompt["model"], connection_status)
        break
    if command["type"] == "handled":
        if user_input == "/help":
            show_help(console)
        elif user_input == "/reset":
            render_chat(console, chat_items, prompt["model"], connection_status)
        else:
            add_command(command["message"])
        render_chat(console, chat_items, prompt["model"], connection_status)
        continue
    if command["type"] == "connect":
        result = check_connection()
        update_connection_status(result)
        add_system(format_connection_result(result))
        render_chat(console, chat_items, prompt["model"], connection_status)
        continue
    if command["type"] == "load":
        try:
            loaded_messages, loaded_items = load_history()
            messages[:] = loaded_messages
            chat_items[:] = loaded_items
            add_system("已恢复最近会话")
        except FileNotFoundError:
            add_error("还没有历史会话")
        except Exception as error:
            add_error(str(error))
        render_chat(console, chat_items, prompt["model"], connection_status)
        continue
    if command["type"] == "history":
        add_command(str(history_path()))
        render_chat(console, chat_items, prompt["model"], connection_status)
        continue

    if not user_input:
        continue
    #把输入加载给LLM
    add_user(user_input)
    render_item(console, {"type": "user", "content": user_input})

    _sa = [False]
    def plan_emit(item_type, content):
        if item_type == "status":
            update_last_status(content)
            print(f"\r\033[2K[Status] {short_status(content)}", end="", flush=True)
            _sa[0] = True
        else:
            if _sa[0]:
                print()
                _sa[0] = False
            emit(item_type, content)
            render_item(console, {"type": item_type, "content": content})
    #执行时status覆盖当前行实时更新，其他事件正常打印

    try:
        observed = False
        for _ in range(10):
            #最多循环10轮，防止无限调用
            result = make_plan(messages)
            if result["type"] != "plan":
                messages.append({"role": "assistant", "content": result["content"]})
                emit("assistant", result["content"])
                render_item(console, {"type": "assistant", "content": result["content"]})
                break
            #LLM返回文本则显示并结束循环

            if result.get("content"):
                emit("plan", result["content"])
                render_item(console, {"type": "plan", "content": result["content"]})
            tool_calls = result["tool_calls"]
            if has_action_tool(tool_calls) and not observed:
                tool_calls = [{"name": "observe", "args": {}}]
            elif incomplete_tool_batch(result.get("content"), tool_calls):
                messages.append({"role": "assistant", "content": result.get("content") or ""})
                messages.append({"role": "user", "content": "你刚才列出了多步动作计划，但没有把每一步都放进tool_calls。请一次性返回完整tool_calls序列，不要只调用第一步。"})
                continue
            plan_results = run_plan(tool_calls, plan_emit)
            if any(item["name"] == "observe" and item["signal"] != "FAILURE" for item in plan_results):
                observed = True
            if _sa[0]:
                print()
                _sa[0] = False
            #status行结束后换行
            steps = ", ".join(f"{tc['name']}({tc['args']})" for tc in tool_calls)
            from Executor.state import format_feedback, format_latest_state
            tool_result = summarize_plan_results(plan_results, format_latest_state(), format_feedback())
            messages.append({"role": "assistant", "content": f"已执行: {steps}\n结果: {tool_result}"})
            messages.append({"role": "user", "content": f"执行结果: {tool_result}，请决定下一步"})
            #执行tool_calls，把结果发回messages让LLM决定下一步
        save_history(messages, chat_items)
    except Exception as error:
        add_error(str(error))
        save_history(messages, chat_items)
        render_chat(console, chat_items, prompt["model"], connection_status)
