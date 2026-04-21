from pathlib import Path
import sys
from prompt_toolkit import prompt as input_prompt
from rich.console import Console

sys.path.append(str(Path(__file__).resolve().parents[1]))
from Tui.commands import handle_command
from Tui.gateway import LocalChatGateway
from Tui.render import show_command, show_error, show_status, show_system, show_user, show_welcome, start_assistant
from Tui.session import messages, reset
from Planner.llm_core import prompt

gateway = LocalChatGateway()
console = Console()
#创建本地网关

show_welcome(console, prompt["model"])

while True:
    show_status(console, messages)
    user_input = input_prompt("You> ").strip()
    command = handle_command(user_input, reset)
    if command["type"] == "quit":
        show_system(console, command["message"])
        break
    if command["type"] == "handled":
        show_command(console, command["message"])
        continue
    if not user_input:
        continue

    show_user(console, user_input)
    messages.append({"role": "user", "content": user_input})
    start_assistant(console)
    try:
        reply = ""
        for text in gateway.stream_chat(messages):
            console.print(text, end="")
            reply += text
        console.print()
        messages.append({"role": "assistant", "content": reply})
        #流式发送消息并把完整回复加入上下文
    except Exception as error:
        show_error(console, str(error))
