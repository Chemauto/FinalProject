from pathlib import Path
import sys
from prompt_toolkit import prompt as input_prompt
from prompt_toolkit.history import FileHistory
from rich.console import Console

sys.path.append(str(Path(__file__).resolve().parents[1]))
from Executor.demo_executor import run_demo_task
from Executor.executor import run_plan
from Tui.commands import handle_command
from Tui.gateway import LocalChatGateway
from Tui.history import history_path, load_history, save_history
from Tui.render import render_chat, render_item, show_help, show_status, show_welcome
from Tui.session import add_command, add_error, add_system, add_user, chat_items, emit, messages, reset, update_last_status
from Tui.stream import StreamItem
from Planner.llm_core import prompt, make_plan

gateway = LocalChatGateway()
console = Console()
history = FileHistory(str(Path.home() / ".finalproject_tui_history"))
#创建本地网关

show_welcome(console, prompt["model"])

while True:
    show_status(console, messages)
    user_input = input_prompt("You> ", history=history).strip()
    command = handle_command(user_input, reset)
    if command["type"] == "quit":
        add_system(command["message"])
        render_chat(console, chat_items)
        break
    if command["type"] == "handled":
        if user_input == "/help":
            show_help(console)
        elif user_input == "/reset":
            render_chat(console, chat_items)
        else:
            add_command(command["message"])
            render_chat(console, chat_items)
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
        render_chat(console, chat_items)
        continue
    if command["type"] == "history":
        add_command(str(history_path()))
        render_chat(console, chat_items)
        continue
    if command["type"] == "demo":
        _sa = [False]
        def demo_emit(item_type, content):
            if item_type == "status":
                update_last_status(content)
                print(f"\r\033[K[Status] {content}", end="", flush=True)
                _sa[0] = True
            else:
                if _sa[0]:
                    print()
                    _sa[0] = False
                emit(item_type, content)
                render_item(console, {"type": item_type, "content": content})
        #demo执行时status覆盖当前行实时更新，其他事件正常打印

        run_demo_task(demo_emit)
        if _sa[0]:
            print()
        save_history(messages, chat_items)
        continue
    if not user_input:
        continue

    add_user(user_input)
    render_item(console, {"type": "user", "content": user_input})

    _sa = [False]
    def plan_emit(item_type, content):
        if item_type == "status":
            update_last_status(content)
            print(f"\r\033[K[Status] {content}", end="", flush=True)
            _sa[0] = True
        else:
            if _sa[0]:
                print()
                _sa[0] = False
            emit(item_type, content)
            render_item(console, {"type": item_type, "content": content})
    #执行时status覆盖当前行实时更新，其他事件正常打印

    try:
        for _ in range(10):
            #最多循环10轮，防止无限调用
            result = make_plan(messages)
            if result["type"] != "plan":
                messages.append({"role": "assistant", "content": result["content"]})
                emit("assistant", result["content"])
                render_item(console, {"type": "assistant", "content": result["content"]})
                break
            #LLM返回文本则显示并结束循环

            run_plan(result["tool_calls"], plan_emit)
            if _sa[0]:
                print()
                _sa[0] = False
            #status行结束后换行
            steps = ", ".join(f"{tc['name']}({tc['args']})" for tc in result["tool_calls"])
            from Executor.state import fmt_robot, fmt_box
            tool_result = f"{fmt_robot()}, {fmt_box()}"
            messages.append({"role": "assistant", "content": f"已执行: {steps}\n结果: {tool_result}"})
            messages.append({"role": "user", "content": f"执行结果: {tool_result}，请决定下一步"})
            #执行tool_calls，把结果发回messages让LLM决定下一步
        save_history(messages, chat_items)
    except Exception as error:
        add_error(str(error))
        save_history(messages, chat_items)
        render_chat(console, chat_items)
