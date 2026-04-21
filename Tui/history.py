import json
from pathlib import Path

HISTORY_DIR = Path(__file__).resolve().parents[1] / ".tui_history"

def history_path():
    return HISTORY_DIR / "latest.json"
#当前会话历史文件

def save_history(messages, chat_items):
    HISTORY_DIR.mkdir(exist_ok=True)
    data = {"messages": messages, "chat_items": chat_items}
    history_path().write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
#保存LLM上下文和TUI显示历史

def load_history():
    data = json.loads(history_path().read_text(encoding="utf-8"))
    return data["messages"], data["chat_items"]
#加载最近一次会话
