from Planner.llm_core import prompt

messages = [{"role": "system", "content": prompt["system_prompt"]}]
chat_items = [{"type": "system", "content": "会话已开始"}]
connection_status = {}
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

def add_error(text):
    chat_items.append({"type": "error", "content": text})
#添加错误消息

def emit(item_type, content):
    chat_items.append({"type": item_type, "content": content})
#执行层事件入口

def update_last_status(content):
    if chat_items and chat_items[-1]["type"] == "status":
        chat_items[-1]["content"] = content
    else:
        emit("status", content)
#更新最后一条状态，避免实时状态刷屏

def update_connection_status(result):
    connection_status.clear()
    connection_status.update(dict(result or {}))
#保存最近一次/connect的机器人服务连接状态，供TUI状态栏显示

def reset():
    messages.clear()
    chat_items.clear()
    messages.append({"role": "system", "content": prompt["system_prompt"]})
    chat_items.append({"type": "system", "content": "上下文已清空"})
#重置上下文
