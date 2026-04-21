def handle_command(text, reset):
    if text in ("/quit", "/exit", "退出"):
        return {"type": "quit", "message": "退出TUI"}
    if text == "/reset":
        reset()
        return {"type": "handled", "message": "上下文已清空"}
    if text == "/help":
        return {"type": "handled", "message": "/help  查看帮助\n/demo  演示机器人事件\n/reset 清空上下文\n/quit  退出"}
    if text == "/demo":
        return {"type": "demo"}
    return {"type": "message"}
#处理TUI命令
