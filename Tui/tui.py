from pathlib import Path
import sys
from prompt_toolkit import prompt as input_prompt
from prompt_toolkit.history import FileHistory
from rich.console import Console

sys.path.append(str(Path(__file__).resolve().parents[1]))
from Tui.commands import handle_command
from Tui.gateway import LocalChatGateway
from Tui.history import history_path, load_history, save_history
from Tui.render import render_chat, show_help, show_status, show_welcome, start_assistant
from Tui.session import add_command, add_error, add_plan, add_status, add_system, add_tool, add_user, chat_items, messages, reset
from Tui.stream import StreamItem
from Planner.llm_core import prompt

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
        add_plan("1. 观察当前环境\n2. 生成移动计划\n3. 执行并检查状态")
        add_tool("vlm_observe")
        add_status("机器人状态：等待执行")
        save_history(messages, chat_items)
        render_chat(console, chat_items)
        continue
    if not user_input:
        continue

    add_user(user_input)
    render_chat(console, chat_items)
    start_assistant(console)
    try:
        stream_item = StreamItem()
        chat_items.append(stream_item.data)
        for text in gateway.stream_chat(messages):
            console.print(text, end="")
            stream_item.add(text)
        stream_item.done()
        console.print()
        messages.append({"role": "assistant", "content": stream_item.data["content"]})
        save_history(messages, chat_items)
        #流式发送消息并把完整回复加入上下文
    except Exception as error:
        add_error(str(error))
        save_history(messages, chat_items)
        render_chat(console, chat_items)
