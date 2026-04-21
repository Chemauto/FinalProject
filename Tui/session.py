from Planner.llm_core import prompt

messages = [{"role": "system", "content": prompt["system_prompt"]}]
chat_items = [{"type": "system", "content": "会话已开始"}]
#messages给LLM用，chat_items给TUI显示用

def add_user(text):
    messages.append({"role": "user", "content": text})
    chat_items.append({"type": "user", "content": text})
#添加用户消息

def add_system(text):
    chat_items.append({"type": "system", "content": text})
#添加系统消息

def add_command(text):
    chat_items.append({"type": "command", "content": text})
#添加命令消息

def add_plan(text):
    chat_items.append({"type": "plan", "content": text})
#添加任务计划消息

def add_tool(text):
    chat_items.append({"type": "tool", "content": text})
#添加工具调用消息

def add_status(text):
    chat_items.append({"type": "status", "content": text})
#添加机器人状态消息

def add_error(text):
    chat_items.append({"type": "error", "content": text})
#添加错误消息

def reset():
    messages.clear()
    chat_items.clear()
    messages.append({"role": "system", "content": prompt["system_prompt"]})
    chat_items.append({"type": "system", "content": "上下文已清空"})
#重置上下文
