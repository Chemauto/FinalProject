from pathlib import Path
import sys

sys.path.append(str(Path(__file__).resolve().parents[1]))
from Planner.llm_core import chat, prompt

messages = [{"role": "system", "content": prompt["system_prompt"]}]
#保存上下文，后续每轮都会追加用户和模型消息

while True:
    user_input = input("You> ").strip()
    if user_input in ("/quit", "/exit", "退出"):
        break
    if not user_input:
        continue

    messages.append({"role": "user", "content": user_input})
    reply = chat(messages)
    print("AI>", reply)
    messages.append({"role": "assistant", "content": reply})
    #发送消息并把回复加入上下文
