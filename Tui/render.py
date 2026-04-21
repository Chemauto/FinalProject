from rich.panel import Panel

def show_welcome(console, model):
    console.print(Panel(f"Model: {model}\nCommands: /help /reset /quit\nContext: on\nStreaming: on", title="FinalProject TUI", border_style="cyan"))
#显示欢迎信息

def show_status(console, messages):
    turns = max(0, (len(messages) - 1) // 2)
    console.print(f"[dim]Context: {turns} turns[/dim]")
#显示上下文轮数

def show_user(console, text):
    console.print(Panel(text, title="User", border_style="cyan"))
#显示用户输入

def show_command(console, text):
    console.print(Panel(text, title="Command", border_style="blue"))
#显示命令结果

def show_system(console, text):
    console.print(Panel(text, title="System", border_style="yellow"))
#显示系统消息

def show_error(console, text):
    console.print(Panel(text, title="Error", border_style="red"))
#显示错误

def start_assistant(console):
    console.print("[bold green]Assistant>[/bold green] ", end="")
#开始流式显示助手回复
