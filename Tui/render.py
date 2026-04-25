from rich.panel import Panel

def show_welcome(console, model):
    commands = "/help /connect /load /history /reset /quit"
    console.print(Panel(f"Model: {model}\nCommands: {commands}\nContext: on\nStreaming: on", title="FinalProject TUI", border_style="cyan"))
#显示欢迎信息

def show_status(console, messages):
    turns = max(0, (len(messages) - 1) // 2)
    console.print(f"[dim]Context: {turns} turns[/dim]")
#显示上下文轮数

def render_item(console, item):
    styles = {
        "user": ("User", "cyan"),
        "assistant": ("Assistant", "green"),
        "system": ("System", "yellow"),
        "error": ("Error", "red"),
        "command": ("Command", "blue"),
        "plan": ("Plan", "magenta"),
        "tool": ("Tool", "white"),
        "status": ("Status", "blue"),
    }
    title, style = styles.get(item["type"], ("Message", "white"))
    console.print(Panel(item["content"], title=title, border_style=style))
#显示一条聊天记录

def render_chat(console, chat_items):
    console.clear()
    for item in chat_items:
        render_item(console, item)
#重绘完整聊天记录

def show_help(console):
    text = "/help    查看帮助\n/connect 检查机器人服务连接\n/load    恢复最近会话\n/history 查看历史路径\n/reset   清空上下文\n/quit    退出\n\n机器人事件会显示为 plan / tool / status / error"
    console.print(Panel(text, title="Help", border_style="blue"))
#显示帮助面板

def start_assistant(console):
    console.print("[bold green]Assistant>[/bold green] ", end="")
#开始流式显示助手回复
